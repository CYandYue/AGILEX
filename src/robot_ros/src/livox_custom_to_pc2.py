#!/usr/bin/env python3
"""
将 livox_ros_driver2/CustomMsg 转换为 sensor_msgs/PointCloud2
用于 rosbag 回放时在 rviz 中可视化 MID360 点云
"""
import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg


def callback(msg):
    n = msg.point_num
    if n == 0:
        return

    # 打包字段：x, y, z (float32), intensity (float32), tag (uint8), line (uint8)
    # 用 float32 intensity 方便 rviz 着色
    data = np.zeros(n, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('tag', np.uint8),
        ('line', np.uint8),
        ('_pad', np.uint8, 2),   # 对齐到 4 字节
    ])

    pts = msg.points
    data['x']         = [p.x for p in pts]
    data['y']         = [p.y for p in pts]
    data['z']         = [p.z for p in pts]
    data['intensity'] = [p.reflectivity for p in pts]
    data['tag']       = [p.tag for p in pts]
    data['line']      = [p.line for p in pts]

    pc2 = PointCloud2()
    pc2.header = msg.header
    pc2.height = 1
    pc2.width = n
    pc2.is_dense = False
    pc2.is_bigendian = False
    pc2.point_step = data.dtype.itemsize   # 16 bytes
    pc2.row_step = pc2.point_step * n
    pc2.fields = [
        PointField('x',         0,  PointField.FLOAT32, 1),
        PointField('y',         4,  PointField.FLOAT32, 1),
        PointField('z',         8,  PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    pc2.data = data.tobytes()
    pub.publish(pc2)


if __name__ == '__main__':
    rospy.init_node('livox_custom_to_pc2')
    pub = rospy.Publisher('/livox/lidar_pc2', PointCloud2, queue_size=10)
    rospy.Subscriber('/livox/lidar', CustomMsg, callback, queue_size=10)
    rospy.loginfo('livox_custom_to_pc2 ready: /livox/lidar -> /livox/lidar_pc2')
    rospy.spin()
