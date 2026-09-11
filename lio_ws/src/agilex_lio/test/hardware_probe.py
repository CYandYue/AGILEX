#!/usr/bin/env python3
"""Short sensor-only probe on a private ROS master; stops every process it starts."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mapping', action='store_true')
    parser.add_argument('--imu-source', choices=['hi226','mid360'], default='hi226')
    args=parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    output = Path(tempfile.mkdtemp(prefix='agilex-lio-hardware-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(output))
    os.environ.pop('ROS_HOSTNAME', None)
    processes, files = [], []
    try:
        def start(command, log):
            f = open(output / log, 'w'); files.append(f)
            p = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(p)
            return p
        start(['roscore', '-p', str(port)], 'master.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            try:
                if master.getPid('/probe')[0] == 1: break
            except OSError: pass
            time.sleep(.1)
        else: raise RuntimeError('Master failed to start')
        import rospy
        from livox_ros_driver2.msg import CustomMsg
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        rospy.init_node('sensor_probe', disable_signals=True)
        data = {}
        topics = ['/imu/data_raw', '/livox/lidar_192_168_1_113', '/livox/lidar_192_168_1_154',
                  '/livox/imu_192_168_1_113', '/livox/imu_192_168_1_154']
        if args.mapping: topics.append('/lio/odometry')
        subscribers = []
        for topic in topics:
            data[topic] = []
            def callback(msg, key=topic):
                arrival = time.time()
                if '/lidar_' in key:
                    # Inspect serialized metadata without constructing ~20k
                    # Python CustomPoint objects for every incoming cloud.
                    raw = msg._buff
                    seq, sec, nsec, length = struct.unpack_from('<4I', raw)
                    frame = raw[16:16+length].decode()
                    offset = 16+length
                    point_num = struct.unpack_from('<I', raw, offset+8)[0]
                    count = struct.unpack_from('<I', raw, offset+16)[0]
                    end = struct.unpack_from('<I', raw, offset+20+(count-1)*19)[0] if count else 0
                    row={'stamp':sec+nsec/1e9,'arrival':arrival,'frame':frame,'points':point_num,'end_offset_ns':end}
                elif key == '/lio/odometry':
                    p=msg.pose.pose.position
                    row={'stamp':msg.header.stamp.to_sec(),'arrival':arrival,'frame':msg.header.frame_id,
                         'position_norm_m':math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)}
                else:
                    row = {'stamp':msg.header.stamp.to_sec(), 'arrival':arrival, 'frame':msg.header.frame_id}
                    a=msg.linear_acceleration; g=msg.angular_velocity
                    row['acc_norm'] = math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)
                    row['gyro_norm'] = math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)
                data[key].append(row)
            msg_type=rospy.AnyMsg if '/lidar_' in topic else (Odometry if topic == '/lio/odometry' else Imu)
            subscribers.append(rospy.Subscriber(topic, msg_type, callback, queue_size=1000, buff_size=4*1024*1024))
        start(['rosbag', 'record', '-O', str(output / 'sensors.bag'), *topics, '/tf', '/tf_static'], 'record.log')
        driver = start(['roslaunch','agilex_lio','mapping.launch' if args.mapping else 'sensors.launch',
                        'imu_source:='+args.imu_source,'camera:=false'], 'sensors.log')
        for _ in range(200):
            if driver.poll() is not None: raise RuntimeError('Sensor launch exited; see '+str(output))
            time.sleep(.1)
        summary = {}
        for topic, rows in data.items():
            rows = list(rows)
            stats = {'count':len(rows)}
            if rows:
                stamps=[r['stamp'] for r in rows]
                dt=[b-a for a,b in zip(stamps,stamps[1:])]
                stats.update(frame=rows[-1]['frame'], first_stamp=stamps[0], last_stamp=stamps[-1],
                             nonincreasing_intervals=sum(d<=0 for d in dt),
                             mean_arrival_minus_stamp_s=sum(r['arrival']-r['stamp'] for r in rows)/len(rows))
                if stamps[-1]>stamps[0]: stats['rate_hz']=(len(rows)-1)/(stamps[-1]-stamps[0])
                stats['last_arrival_minus_stamp_s'] = rows[-1]['arrival']-rows[-1]['stamp']
                for key in ('acc_norm','gyro_norm','points','end_offset_ns','position_norm_m'):
                    if key in rows[0]: stats['mean_'+key]=sum(r[key] for r in rows)/len(rows)
                if 'position_norm_m' in rows[-1]: stats['last_position_norm_m']=rows[-1]['position_norm_m']
            summary[topic]=stats
        (output / 'result.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2))
        if args.mapping and summary['/lio/odometry']['count'] < 5:
            raise RuntimeError('Mapping did not produce enough odometry updates')
        if args.mapping and (summary['/lio/odometry']['mean_arrival_minus_stamp_s'] > .5 or
                             summary['/lio/odometry']['last_arrival_minus_stamp_s'] > .5):
            raise RuntimeError('Odometry processing is lagging by more than 0.5 seconds')
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
        for f in files: f.close()
        print('Probe output:',output)


if __name__=='__main__': main()
