#!/usr/bin/env python3
"""Publish the configured sensor geometry; sensor_relay labels raw streams.

No point coordinates, IMU values, or timestamps are modified. The driver JSON
must keep all extrinsics at zero. The estimator owns lidar-to-IMU transforms.
"""
import json
import numpy as np
import rospy
import tf2_ros
from tf.transformations import euler_matrix, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped


def transform(translation, rotation):
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation).reshape(3, 3)
    out[:3, 3] = translation
    return out


def mount(values):
    out = euler_matrix(*values['rpy'])
    out[:3, 3] = values['translation']
    return out


def main():
    rospy.init_node('lio_sensor_frames')
    source = rospy.get_param('~imu_source')
    if source not in ('hi226', 'mid360'):
        raise ValueError('imu_source must be hi226 or mid360')
    with open(rospy.get_param('~lidar_config')) as f:
        lidars = json.load(f)['lidar_configs']
    if len(lidars) != 2 or any(any(float(v) != 0 for v in d['extrinsic_parameter'].values()) for d in lidars):
        raise ValueError('Expected two raw lidars with zero driver extrinsics')
    params = rospy.get_param('/lio/mapping')
    mounts = rospy.get_param('/lio_mounts')
    t_i_l1 = transform(params['extrinsic_T'], params['extrinsic_R'])
    if params['extrinsic_imu_to_lidars']:
        t_l1_l2 = np.linalg.inv(t_i_l1) @ transform(params['extrinsic_T2'], params['extrinsic_R2'])
    else:
        t_l1_l2 = transform(params['extrinsic_T_L2_wrt_L1'], params['extrinsic_R_L2_wrt_L1'])
    t_b_l1 = mount(mounts['base_to_lidar1'])
    internal = mount(mounts['internal_imu_to_lidar'])
    relations = [
        ('body', 'mid360_a_lidar', t_i_l1),
        ('mid360_a_lidar', 'mid360_b_lidar', t_l1_l2),
        ('mid360_a_lidar', 'base_link', np.linalg.inv(t_b_l1)),
        ('base_link', 'camera_link', mount(mounts['base_to_camera'])),
        ('base_link', 'imu_link', mount(mounts['base_to_external_imu'])),
        ('mid360_a_lidar', 'mid360_a_imu', np.linalg.inv(internal)),
        ('mid360_b_lidar', 'mid360_b_imu', np.linalg.inv(internal)),
    ]
    messages = []
    for parent, child, matrix in relations:
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = matrix[:3, 3]
        q = quaternion_from_matrix(matrix)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        messages.append(msg)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(messages)

    rospy.logwarn('Using vendor/nominal extrinsic initial values; time alignment and calibration accuracy are unverified. IMU: %s', source)
    rospy.spin()


if __name__ == '__main__':
    main()
