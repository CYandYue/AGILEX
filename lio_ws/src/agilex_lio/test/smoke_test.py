#!/usr/bin/env python3
"""End-to-end synthetic stationary room test; never starts hardware drivers.

Run after sourcing lio_ws/devel/setup.bash. Exercises both driver2 message
streams, the frame relay, IMU initialization, scan matching and TF composition.
This is software integration evidence, not a real-world accuracy benchmark.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--imu-source', choices=['hi226', 'mid360'], default='hi226')
    parser.add_argument('--method', choices=['bundle', 'async', 'adaptive'], default='bundle')
    parser.add_argument('--yaw-rate', type=float, default=0.0,
                        help='Known rotation about the IMU origin after two stationary seconds, rad/s')
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    logs = Path(tempfile.mkdtemp(prefix='agilex-lio-smoke-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_LOG_DIR=str(logs))
    os.environ.pop('ROS_HOSTNAME', None)
    processes, handles = [], []
    try:
        def start(cmd, filename):
            f = open(logs / filename, 'w')
            handles.append(f)
            p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(p)
            return p
        start(['roscore', '-p', str(port)], 'master.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                if master.getPid('/smoke')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('ROS master did not start')
        launch = start(['roslaunch', 'agilex_lio', 'mapping.launch', 'start_drivers:=false',
                        'imu_source:=' + args.imu_source, 'method:=' + args.method], 'mapping.log')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        rospy.init_node('lio_smoke', disable_signals=True)
        odoms, relayed = [], []
        rospy.Subscriber('/lio/odometry', Odometry, odoms.append, queue_size=100)
        rospy.Subscriber('/sensors/mid360_a/lidar', CustomMsg,
                         lambda msg: relayed.append((msg.header.frame_id, msg.header.stamp.to_nsec(), msg.points[0].offset_time)), queue_size=10)
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        pubs = [rospy.Publisher('/livox/lidar_192_168_1_' + ip, CustomMsg, queue_size=30) for ip in ('113', '154')]
        imu_topic = '/imu/data_raw' if args.imu_source == 'hi226' else '/livox/imu_192_168_1_113'
        imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=500)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError('Mapping launch exited')
            if all(p.get_num_connections() for p in pubs) and imu_pub.get_num_connections() and rospy.has_param('/lio/mapping/extrinsic_T'):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Input subscriptions did not connect')
        time.sleep(1)

        params = rospy.get_param('/lio/mapping')
        r1 = np.array(params['extrinsic_R']).reshape(3, 3)
        t1 = np.array(params['extrinsic_T'])
        r12 = np.array(params['extrinsic_R_L2_wrt_L1']).reshape(3, 3)
        t12 = np.array(params['extrinsic_T_L2_wrt_L1'])
        # Four walls, floor and ceiling in the stationary IMU reference frame.
        horizontal = np.linspace(-3, 3, 28)
        vertical = np.linspace(-1, 2.5, 20)
        world = np.array([(x, y, z) for x in (-4, 4) for y in horizontal for z in vertical]
                         + [(x, y, z) for y in (-4, 4) for x in horizontal for z in vertical]
                         + [(x, y, z) for z in (-1.5, 3) for x in horizontal for y in horizontal])
        l1 = (world - t1) @ r1
        l2 = (l1 - t12) @ r12
        messages = []
        for xyz in (l1, l2):
            msg = CustomMsg()
            msg.header.frame_id = 'livox_frame'
            msg.points = [CustomPoint(offset_time=int(i * 99000000 / (len(xyz)-1)), x=p[0], y=p[1], z=p[2],
                                      reflectivity=100, tag=0, line=i % 4) for i, p in enumerate(xyz)]
            msg.point_num = len(msg.points)
            messages.append(msg)
        imu = Imu()
        imu.header.frame_id = 'imu_link' if args.imu_source == 'hi226' else 'livox_frame'
        imu.linear_acceleration.z = 9.8 if args.imu_source == 'hi226' else 1.0
        imu.orientation.w = 1
        epoch = rospy.Time.now().to_sec()
        start_wall = time.monotonic()
        sent_stamps = set()
        final_end = None
        # Simulate one device starting half a second earlier. These stale
        # scans must be discarded before pairing the current streams.
        for i in range(5):
            messages[0].header.stamp = rospy.Time.from_sec(epoch - .5 + .1*i)
            messages[0].timebase = messages[0].header.stamp.to_nsec()
            sent_stamps.add(messages[0].header.stamp.to_nsec())
            pubs[0].publish(messages[0])
        # 10 Hz per lidar, second lidar scans start 20 ms later. IMU at 200 Hz.
        for step in range(1800):
            stamp = epoch + step * .005
            imu.header.stamp = rospy.Time.from_sec(stamp)
            imu.angular_velocity.z = args.yaw_rate if step >= 400 else 0.0
            imu_pub.publish(imu)
            if step % 20 == 0:
                for i, (pub, msg) in enumerate(zip(pubs, messages)):
                    msg.header.stamp = rospy.Time.from_sec(stamp + .02 * i)
                    msg.timebase = msg.header.stamp.to_nsec()
                    msg.lidar_id = i
                    if i == 0:
                        sent_stamps.add(msg.header.stamp.to_nsec())
                    if args.yaw_rate:
                        # Each point sees the fixed room at its own physical
                        # acquisition time, including the second lidar's delay.
                        point_times = step * .005 + .02*i + np.array([p.offset_time for p in msg.points]) / 1e9
                        angles = args.yaw_rate * np.maximum(point_times - 2.0, 0.0)
                        c, s = np.cos(angles), np.sin(angles)
                        body_xyz = np.column_stack((c*world[:, 0]+s*world[:, 1],
                                                    -s*world[:, 0]+c*world[:, 1], world[:, 2]))
                        scan_xyz = (body_xyz-t1) @ r1
                        if i == 1:
                            scan_xyz = (scan_xyz-t12) @ r12
                        for point, xyz in zip(msg.points, scan_xyz):
                            point.x, point.y, point.z = xyz
                    pub.publish(msg)
                final_end = stamp + .02 + .099
            if launch.poll() is not None:
                raise RuntimeError('Mapping process exited during input')
            remaining = start_wall + (step+1) * .005 - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        # Supply the IMU tail needed to complete the final pair.
        for step in range(1800, 1840):
            imu.header.stamp = rospy.Time.from_sec(epoch + step * .005)
            imu_pub.publish(imu)
            time.sleep(.005)
        time.sleep(2)
        assert len(odoms) >= 20, 'Too few odometry updates: %d' % len(odoms)
        last = odoms[-1]
        p = last.pose.pose.position
        q = last.pose.pose.orientation
        norm = math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)
        assert math.isfinite(norm) and norm < .02, 'IMU origin position error exceeded 2 cm: %s' % norm
        assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < .001
        expected_yaw = args.yaw_rate * max(final_end-epoch-2.0, 0.0)
        actual_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        yaw_error = math.atan2(math.sin(actual_yaw-expected_yaw), math.cos(actual_yaw-expected_yaw))
        assert abs(yaw_error) < .03, 'Rotation trajectory error: %s radians' % yaw_error
        assert all(b.header.stamp >= a.header.stamp for a, b in zip(odoms, odoms[1:])), 'Odometry timestamps went backward'
        assert abs(last.header.stamp.to_sec() - final_end) < .003, 'Pose timestamp does not represent the later scan end'
        assert 'Dropping unpaired lidar scan' in (logs / 'mapping.log').read_text(), 'Startup skew guard was not exercised'
        assert relayed and all(f == 'mid360_a_lidar' and ns in sent_stamps and offset == 0 for f, ns, offset in relayed)
        t = buffer.lookup_transform('body', 'camera_link', rospy.Time(0), rospy.Duration(3))
        body_l1 = np.eye(4); body_l1[:3, :3] = r1; body_l1[:3, 3] = t1
        mounts = rospy.get_param('/lio_mounts')
        from tf.transformations import euler_matrix
        base_l1 = euler_matrix(*mounts['base_to_lidar1']['rpy'])
        base_l1[:3, 3] = mounts['base_to_lidar1']['translation']
        base_c = euler_matrix(*mounts['base_to_camera']['rpy'])
        base_c[:3, 3] = mounts['base_to_camera']['translation']
        expected = body_l1 @ np.linalg.inv(base_l1) @ base_c
        actual = t.transform.translation
        assert np.allclose([actual.x, actual.y, actual.z], expected[:3, 3], atol=1e-6)
        result = dict(imu_source=args.imu_source, method=args.method, odometry_messages=len(odoms),
                      stationary_position_norm_m=norm, final_stamp_error_s=last.header.stamp.to_sec()-final_end,
                      final_position_m=[p.x, p.y, p.z],
                      yaw_rate_rad_s=args.yaw_rate, yaw_error_rad=yaw_error,
                      startup_skew_guard=True, logs=str(logs), status='PASS')
        (logs / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=5)
        for f in handles:
            f.close()
        print('Logs:', logs)


if __name__ == '__main__':
    main()
