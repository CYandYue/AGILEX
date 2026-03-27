#!/usr/bin/env python3
"""
RealSense D435i + Livox Mid360 数据采集脚本

采集内容：
  - RGB 图像
  - 相机深度图（对齐到彩色，来自 RealSense）
  - 相机内参
  - 相机位姿（odom 和 map 两份，TUM 格式）
  - 雷达原始点云（PCD 文件）
  - 雷达投影深度图（16-bit PNG，投影到相机像素坐标）

使用方法：
  1. 启动雷达：  roslaunch scout_bringup open_MID360lidar.launch
  2. 启动相机：  roslaunch realsense2_camera rs_camera.launch align_depth:=true
  3. 运行脚本：  python3 collect_data.py [输出目录]

输出目录结构：
  output_dir/
  ├── rgb/                    # RGB 图像 (PNG)
  ├── depth_camera/           # 相机深度图 (16-bit PNG, mm, 与 RGB 同尺寸)
  ├── lidar_pcd/              # 雷达原始点云 (binary PCD)
  ├── lidar_depth/            # 雷达投影到相机的深度图 (16-bit PNG, mm)
  ├── camera_intrinsics.yaml  # 相机内参
  ├── poses_odom_tum.txt      # 位姿: odom->camera (TUM 格式)
  └── poses_map_tum.txt       # 位姿: map->camera  (TUM 格式)
"""

import sys
import os
import signal
import socket
import urllib.parse
import yaml
import threading
import subprocess
import numpy as np
import cv2

# ── 修复 socket.getfqdn / gethostbyname 挂起问题 ─────────────────────────────
# rospy.init_node() 内部调用这些函数，若 DNS/mDNS 解析卡住会导致永久挂起
import socket as _socket
import threading as _threading

def _make_timeout_wrapper(orig_fn, timeout=2.0, fallback='localhost'):
    def _wrapper(*args, **kwargs):
        result = [fallback]
        def _call():
            try:
                result[0] = orig_fn(*args, **kwargs)
            except Exception:
                pass
        t = _threading.Thread(target=_call, daemon=True)
        t.start()
        t.join(timeout)
        return result[0]
    return _wrapper

_socket.getfqdn        = _make_timeout_wrapper(_socket.getfqdn)
_socket.gethostbyname  = _make_timeout_wrapper(_socket.gethostbyname, fallback='127.0.0.1')
_socket.gethostbyaddr  = _make_timeout_wrapper(_socket.gethostbyaddr, fallback=('localhost', [], ['127.0.0.1']))

# ── 环境变量必须在 import rospy 之前设置 ─────────────────────────────────────
def _setup_ros_env():
    if 'ROS_IP' not in os.environ and 'ROS_HOSTNAME' not in os.environ:
        os.environ['ROS_HOSTNAME'] = 'localhost'
        os.environ['ROS_IP']       = '127.0.0.1'
        print('[修复] 自动设置 ROS_HOSTNAME=localhost + ROS_IP=127.0.0.1')
    if 'ROS_MASTER_URI' not in os.environ:
        os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'

_setup_ros_env()

import rospy
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from cv_bridge import CvBridge


# ── ROS Master 检查 ───────────────────────────────────────────────────────────
def check_ros_master(timeout=3):
    master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    parsed = urllib.parse.urlparse(master_uri)
    host = parsed.hostname or 'localhost'
    port = parsed.port or 11311
    print(f'[检查] 正在连接 ROS Master: {master_uri} ...')
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.close()
        print('[检查] ROS Master 在线 ✓')
        return True
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f'[错误] 无法连接到 ROS Master: {e}')
        print('[提示] 请先运行: roscore')
        return False


# ── rospy.init_node 带超时 ────────────────────────────────────────────────────
def init_node_with_timeout(name, timeout=15):
    """在独立线程调用 init_node，超时则打印诊断并强制退出。"""
    err  = [None]
    done = threading.Event()

    def _init():
        try:
            rospy.init_node(name, anonymous=True, disable_signals=True)
        except Exception as e:
            err[0] = e
        finally:
            done.set()

    threading.Thread(target=_init, daemon=True).start()

    if not done.wait(timeout):
        print(f'\n[错误] rospy.init_node() 超时（>{timeout}s），常见原因：')
        print('  1. /etc/hosts 缺少主机名映射，请运行：')
        print('       echo "127.0.1.1 $(hostname)" | sudo tee -a /etc/hosts')
        print('  2. roscore 异常，请重启：')
        print('       pkill -f rosmaster && roscore')
        os._exit(1)

    if err[0]:
        raise err[0]


# ── PCD 保存（binary，兼容 PCL / Open3D）─────────────────────────────────────
def save_pcd_binary(points_xyzr: np.ndarray, filepath: str):
    """
    将 Nx4 float32 点云 [x, y, z, intensity] 保存为 binary PCD 文件。
    """
    n = len(points_xyzr)
    if n == 0:
        return
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    ).encode('ascii')
    with open(filepath, 'wb') as f:
        f.write(header)
        f.write(points_xyzr.astype(np.float32).tobytes())


# ── 话题类型检测 ──────────────────────────────────────────────────────────────
def get_topic_type(topic, timeout=5):
    try:
        r = subprocess.run(['rostopic', 'type', topic],
                           capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:
        return ''


# ── 主采集类 ──────────────────────────────────────────────────────────────────
class DataCollector:
    LIDAR_TOPIC   = '/livox/lidar'
    LIDAR_MAX_AGE = 0.15   # 与相机帧最大允许时间差（秒）
    LIDAR_BUF_MAX = 30     # 最多缓存帧数

    def __init__(self, output_dir):
        self.output_dir   = output_dir
        self.bridge       = CvBridge()
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')

        # 相机内参（收到 camera_info 后填充）
        self.K          = None
        self.img_width  = 0
        self.img_height = 0
        self._intrinsics_saved = False

        # 雷达缓冲
        self._lidar_lock  = threading.Lock()
        self._lidar_buf   = []          # [(stamp_sec, ndarray Nx4)]
        self._lidar_frame = None        # 雷达坐标系名称

        # TF
        self.tf_buffer   = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 创建输出目录
        for d in ['rgb', 'depth_camera', 'lidar_pcd', 'lidar_depth']:
            os.makedirs(os.path.join(output_dir, d), exist_ok=True)

        # 位姿文件
        self.pose_odom_file = self._open_pose_file('poses_odom_tum.txt', 'odom')
        self.pose_map_file  = self._open_pose_file('poses_map_tum.txt',  'map')

        # 统计
        self.frame_count      = 0
        self.odom_fail        = 0
        self.map_fail         = 0
        self.lidar_miss       = 0
        self._last_frame_count = 0

        # 订阅
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._camera_info_cb)
        self._subscribe_lidar()

        rgb_sub   = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self._image_cb)

        rospy.Timer(rospy.Duration(5.0), self._heartbeat_cb)

        print(f'[DataCollector] 初始化完成，输出目录: {output_dir}')
        print(f'  RGB        : /camera/color/image_raw')
        print(f'  相机深度   : /camera/aligned_depth_to_color/image_raw')
        print(f'  雷达       : {self.LIDAR_TOPIC}')
        print(f'  位姿(odom) : odom -> {self.camera_frame}')
        print(f'  位姿(map)  : map  -> {self.camera_frame}')

    # ── 工具 ──────────────────────────────────────────────────────────────────
    def _open_pose_file(self, filename, frame):
        path = os.path.join(self.output_dir, filename)
        f = open(path, 'w')
        f.write('# TUM format: timestamp tx ty tz qx qy qz qw\n')
        f.write(f'# {frame} -> {self.camera_frame}\n')
        return f

    def _write_pose(self, f, ts_sec, pose):
        if pose is not None:
            f.write(f'{ts_sec:.9f} '
                    f'{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} '
                    f'{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f}\n')
        else:
            f.write(f'{ts_sec:.9f} nan nan nan nan nan nan nan\n')

    def _lookup_pose(self, parent_frame, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                parent_frame, self.camera_frame, stamp, rospy.Duration(0.2))
            t = tf.transform.translation
            r = tf.transform.rotation
            return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except Exception:
            return None

    # ── 雷达订阅（自动检测消息类型）─────────────────────────────────────────
    def _subscribe_lidar(self):
        msg_type = get_topic_type(self.LIDAR_TOPIC)
        print(f'[雷达] 话题类型: {msg_type if msg_type else "未知（默认 PointCloud2）"}')

        if 'CustomMsg' in msg_type:
            pkg = 'livox_ros_driver2' if 'livox_ros_driver2' in msg_type else 'livox_ros_driver'
            try:
                mod = __import__(f'{pkg}.msg', fromlist=['CustomMsg'])
                CustomMsg = mod.CustomMsg
                rospy.Subscriber(self.LIDAR_TOPIC, CustomMsg, self._lidar_custom_cb)
                print(f'[雷达] 使用 {pkg}/CustomMsg 订阅')
                return
            except ImportError as e:
                print(f'[雷达] 警告: 无法导入 {pkg}.msg ({e})，回退到 PointCloud2')

        rospy.Subscriber(self.LIDAR_TOPIC, PointCloud2, self._lidar_pc2_cb)
        print('[雷达] 使用 sensor_msgs/PointCloud2 订阅')

    # ── 雷达回调 ──────────────────────────────────────────────────────────────
    def _push_lidar(self, stamp_sec, arr):
        with self._lidar_lock:
            self._lidar_buf.append((stamp_sec, arr))
            if len(self._lidar_buf) > self.LIDAR_BUF_MAX:
                self._lidar_buf.pop(0)

    def _lidar_pc2_cb(self, msg: PointCloud2):
        self._lidar_frame = msg.header.frame_id
        fields = {f.name for f in msg.fields}
        field_names = ('x', 'y', 'z', 'intensity') if 'intensity' in fields else ('x', 'y', 'z')
        pts = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        if not pts:
            return
        arr = np.array(pts, dtype=np.float32)
        if arr.shape[1] == 3:
            arr = np.hstack([arr, np.zeros((len(arr), 1), dtype=np.float32)])
        self._push_lidar(msg.header.stamp.to_sec(), arr)

    def _lidar_custom_cb(self, msg):
        self._lidar_frame = msg.header.frame_id
        pts = [(p.x, p.y, p.z, float(p.reflectivity))
               for p in msg.points
               if not (p.x == 0.0 and p.y == 0.0 and p.z == 0.0)]
        if not pts:
            return
        self._push_lidar(msg.header.stamp.to_sec(), np.array(pts, dtype=np.float32))

    def _get_nearest_lidar(self, ts_sec):
        """返回时间上最近的雷达帧，超过 LIDAR_MAX_AGE 则返回 None。"""
        with self._lidar_lock:
            if not self._lidar_buf:
                return None
            best = min(self._lidar_buf, key=lambda x: abs(x[0] - ts_sec))
        if abs(best[0] - ts_sec) > self.LIDAR_MAX_AGE:
            return None
        return best[1]

    # ── 相机内参回调 ──────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self._intrinsics_saved:
            return
        self.K          = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.img_width  = msg.width
        self.img_height = msg.height
        info = {
            'width': msg.width, 'height': msg.height,
            'distortion_model': msg.distortion_model,
            'K': list(msg.K), 'D': list(msg.D),
            'R': list(msg.R), 'P': list(msg.P),
        }
        with open(os.path.join(self.output_dir, 'camera_intrinsics.yaml'), 'w') as f:
            yaml.dump(info, f, default_flow_style=False)
        self._intrinsics_saved = True
        rospy.loginfo(f'[DataCollector] 内参已保存: '
                      f'fx={self.K[0,0]:.2f} fy={self.K[1,1]:.2f} '
                      f'cx={self.K[0,2]:.2f} cy={self.K[1,2]:.2f}')

    # ── 雷达投影 ──────────────────────────────────────────────────────────────
    def _project_lidar_to_depth(self, points_xyzr: np.ndarray, stamp) -> np.ndarray | None:
        """
        将雷达点云投影到相机平面，生成 16-bit 深度图（单位 mm）。
        若内参或 TF 不可用则返回 None。
        """
        if self.K is None or self._lidar_frame is None:
            return None

        # 查询 lidar_frame -> camera_color_optical_frame
        try:
            tf_s = self.tf_buffer.lookup_transform(
                self.camera_frame, self._lidar_frame, stamp, rospy.Duration(0.2))
        except Exception:
            return None

        # 四元数 -> 旋转矩阵
        q  = tf_s.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ])
        t = tf_s.transform.translation
        tvec = np.array([t.x, t.y, t.z])

        # 变换到相机坐标系
        xyz_cam = (R @ points_xyzr[:, :3].T).T + tvec   # Nx3

        # 只保留相机前方的点
        mask = xyz_cam[:, 2] > 0.05
        xyz_cam = xyz_cam[mask]
        if len(xyz_cam) == 0:
            return None

        # 投影
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        Z = xyz_cam[:, 2]
        u = (fx * xyz_cam[:, 0] / Z + cx).astype(np.int32)
        v = (fy * xyz_cam[:, 1] / Z + cy).astype(np.int32)

        # 过滤图像范围外的点
        valid = (u >= 0) & (u < self.img_width) & (v >= 0) & (v < self.img_height)
        u, v, Z = u[valid], v[valid], Z[valid]
        if len(u) == 0:
            return None

        # 创建深度图：同一像素保留最近的点（从远到近排序，近的覆盖远的）
        depth = np.zeros((self.img_height, self.img_width), dtype=np.uint16)
        order = np.argsort(Z)[::-1]
        depth[v[order], u[order]] = np.clip(Z[order] * 1000, 0, 65535).astype(np.uint16)
        return depth

    # ── 图像同步回调（核心）──────────────────────────────────────────────────
    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        stamp  = rgb_msg.header.stamp
        ts_str = f'{stamp.secs}_{stamp.nsecs:09d}'
        ts_sec = stamp.to_sec()

        # ---- RGB ----
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            cv2.imwrite(os.path.join(self.output_dir, 'rgb', f'{ts_str}.png'), rgb_img)
        except Exception as e:
            rospy.logwarn(f'RGB转换失败: {e}')
            return

        # ---- 相机深度图 ----
        try:
            cam_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if cam_depth.shape[:2] != rgb_img.shape[:2]:
                rospy.logwarn_once('相机深度图尺寸不匹配，强制缩放')
                cam_depth = cv2.resize(cam_depth,
                                       (rgb_img.shape[1], rgb_img.shape[0]),
                                       interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(os.path.join(self.output_dir, 'depth_camera', f'{ts_str}.png'), cam_depth)
        except Exception as e:
            rospy.logwarn(f'相机深度转换失败: {e}')
            return

        # ---- 雷达点云 & 投影深度图 ----
        pts = self._get_nearest_lidar(ts_sec)
        if pts is not None:
            save_pcd_binary(pts, os.path.join(self.output_dir, 'lidar_pcd', f'{ts_str}.pcd'))
            lidar_depth = self._project_lidar_to_depth(pts, stamp)
            if lidar_depth is not None:
                cv2.imwrite(
                    os.path.join(self.output_dir, 'lidar_depth', f'{ts_str}.png'),
                    lidar_depth)
        else:
            self.lidar_miss += 1
            if self.lidar_miss <= 5:
                rospy.logwarn(f'雷达数据缺失（帧 {self.frame_count}），'
                              f'请确认 {self.LIDAR_TOPIC} 正在发布')

        # ---- 位姿 ----
        odom_pose = self._lookup_pose('odom', stamp)
        if odom_pose is None:
            self.odom_fail += 1
        self._write_pose(self.pose_odom_file, ts_sec, odom_pose)

        map_pose = self._lookup_pose('map', stamp)
        if map_pose is None:
            self.map_fail += 1
        self._write_pose(self.pose_map_file, ts_sec, map_pose)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            rospy.loginfo(f'已采集 {self.frame_count} 帧 | '
                          f'雷达缺失: {self.lidar_miss} | '
                          f'odom失败: {self.odom_fail} | map失败: {self.map_fail}')

    # ── 心跳 ──────────────────────────────────────────────────────────────────
    def _heartbeat_cb(self, event):
        if self.frame_count == 0:
            print('[DataCollector] 等待图像数据... （请确认相机节点已启动）')
        elif self.frame_count == self._last_frame_count:
            print(f'[DataCollector] {self.frame_count} 帧，最近 5 秒无新帧（话题可能已停止）')
        else:
            print(f'[DataCollector] 运行中 | {self.frame_count} 帧 | '
                  f'雷达缺失: {self.lidar_miss} | '
                  f'odom失败: {self.odom_fail} | map失败: {self.map_fail}')
        self._last_frame_count = self.frame_count

    # ── 退出清理 ──────────────────────────────────────────────────────────────
    def shutdown(self):
        for f in [self.pose_odom_file, self.pose_map_file]:
            f.flush()
            f.close()
        rospy.loginfo(f'[DataCollector] 采集结束，共 {self.frame_count} 帧')
        rospy.loginfo(f'输出目录: {self.output_dir}/')
        rospy.loginfo(f'  ├── rgb/                    # RGB 图像')
        rospy.loginfo(f'  ├── depth_camera/           # 相机深度图 (16-bit PNG, mm)')
        rospy.loginfo(f'  ├── lidar_pcd/              # 雷达点云 (binary PCD)')
        rospy.loginfo(f'  ├── lidar_depth/            # 雷达投影深度图 (16-bit PNG, mm)')
        rospy.loginfo(f'  ├── camera_intrinsics.yaml')
        rospy.loginfo(f'  ├── poses_odom_tum.txt')
        rospy.loginfo(f'  └── poses_map_tum.txt')


# ── 入口 ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))

    output_dir = os.path.abspath(
        sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/collected_data'))

    if not check_ros_master():
        sys.exit(1)

    print('[初始化] 调用 rospy.init_node（最多等待 15 秒）...')
    print(f'  ROS_MASTER_URI = {os.environ.get("ROS_MASTER_URI")}')
    print(f'  ROS_HOSTNAME   = {os.environ.get("ROS_HOSTNAME", "未设置")}')
    print(f'  ROS_IP         = {os.environ.get("ROS_IP", "未设置")}')
    init_node_with_timeout('data_collector', timeout=15)
    print('[初始化] init_node 完成 ✓')

    try:
        collector = DataCollector(output_dir)
    except Exception as e:
        print(f'[错误] 初始化失败: {e}')
        sys.exit(1)

    rospy.on_shutdown(collector.shutdown)
    print(f'\n采集中... 按 Ctrl+C 停止\n')
    rospy.spin()
