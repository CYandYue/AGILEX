#!/usr/bin/env python3
"""
RealSense D435i + Livox Mid360 数据采集脚本 v2
两阶段方案：rosbag record 录制 → 离线提取
完全无需 rospy.init_node()，彻底解决节点初始化挂起问题

输出目录结构：
  <output_dir>/
  ├── rgb/                    # RGB 图像 (PNG)
  ├── depth_camera/           # 相机深度图 (16-bit PNG, mm, 与 RGB 同尺寸)
  ├── lidar_pcd/              # 雷达原始点云 (binary PCD)
  ├── lidar_depth/            # 雷达投影到相机的深度图 (16-bit PNG, mm)
  ├── camera_intrinsics.yaml  # 相机内参
  ├── poses_odom_tum.txt      # 位姿: odom->camera (TUM 格式)
  └── poses_map_tum.txt       # 位姿: map->camera  (TUM 格式)

用法：
  # 一键模式（录制完成后自动提取）：
  python3 collect_data_v2.py ~/my_dataset

  # 仅提取已有 bag（跳过录制）：
  python3 collect_data_v2.py extract ~/my_dataset.bag ~/my_dataset
"""

import sys
import os
import signal
import subprocess
import numpy as np
import cv2
import yaml

# ──────────────────────────── 配置 ────────────────────────────────────────────

RECORD_TOPICS = [
    '/camera/color/image_raw',
    '/camera/aligned_depth_to_color/image_raw',
    '/camera/color/camera_info',
    '/livox/lidar',
    '/tf',
    '/tf_static',
]

CAMERA_FRAME  = 'camera_color_optical_frame'
IMG_SLOP_NS   = int(0.05 * 1e9)   # RGB↔depth 同步最大时差 50ms
LIDAR_SLOP    = 0.15               # 图像↔雷达最大时差 150ms
MAX_IMG_BUF   = 10                 # 单流最大缓存帧数
MAX_LIDAR_BUF = 20                 # 雷达缓存帧数（约 2s @10Hz）

# ──────────────────────────── ROS Image → numpy（无需 cv_bridge）────────────

# encoding → (numpy dtype, channels)
_ENC_MAP = {
    'rgb8':    (np.uint8,   3),
    'bgr8':    (np.uint8,   3),
    'rgba8':   (np.uint8,   4),
    'bgra8':   (np.uint8,   4),
    'mono8':   (np.uint8,   1),
    '8UC1':    (np.uint8,   1),
    '8UC3':    (np.uint8,   3),
    '16UC1':   (np.uint16,  1),
    '16SC1':   (np.int16,   1),
    '32FC1':   (np.float32, 1),
}

def ros_image_to_numpy(msg) -> np.ndarray:
    """
    把 sensor_msgs/Image 消息转成 numpy 数组，不依赖 cv_bridge。
    返回 shape=(H, W) 或 (H, W, C)，BGR 格式（与 OpenCV 一致）。
    """
    enc = msg.encoding.lower()
    # 规范化
    enc_key = msg.encoding  # 保留原始大小写查表
    if enc_key not in _ENC_MAP:
        # 尝试小写
        enc_key = enc
    dtype, channels = _ENC_MAP.get(enc_key, (np.uint8, 1))

    raw = bytes(msg.data)  # bytes / bytearray / list 均兼容
    arr = np.frombuffer(raw, dtype=dtype)

    # 有时 step 与 width*channels*itemsize 不同（行对齐填充）
    itemsize  = np.dtype(dtype).itemsize
    row_bytes = msg.width * channels * itemsize
    if msg.step != row_bytes and msg.step > 0:
        # 去掉每行末尾的填充字节
        rows = []
        for r in range(msg.height):
            start = r * msg.step
            rows.append(raw[start: start + row_bytes])
        arr = np.frombuffer(b''.join(rows), dtype=dtype)

    if channels == 1:
        arr = arr.reshape(msg.height, msg.width)
    else:
        arr = arr.reshape(msg.height, msg.width, channels)
        # RGB → BGR
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = arr[:, :, ::-1].copy()
        elif msg.encoding in ('rgba8', 'RGBA8'):
            arr = arr[:, :, [2, 1, 0, 3]].copy()

    return arr


# ──────────────────────────── PCD 保存 ────────────────────────────────────────

def save_pcd_binary(points: np.ndarray, filepath: str):
    """保存 Nx4 float32 [x,y,z,intensity] 为 binary PCD 文件"""
    n = len(points)
    if n == 0:
        return
    header = (
        '# .PCD v0.7\nVERSION 0.7\n'
        'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\nDATA binary\n'
    ).encode('ascii')
    with open(filepath, 'wb') as f:
        f.write(header)
        f.write(points.astype(np.float32).tobytes())


# ──────────────────────────── TF 工具 ─────────────────────────────────────────

def load_tf_buffer(bag_path):
    """
    预加载 bag 中所有 /tf + /tf_static 到 BufferCore。
    tf2_py.BufferCore 不需要 rospy.init_node()。
    """
    import rosbag
    import tf2_py
    import genpy

    buf = tf2_py.BufferCore(genpy.Duration(3600 * 24))
    count = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=['/tf', '/tf_static']):
            for transform in msg.transforms:
                if topic == '/tf_static':
                    buf.set_transform_static(transform, 'bag')
                else:
                    buf.set_transform(transform, 'bag')
                count += 1
    print(f'[TF] 已加载 {count} 条变换')
    return buf


def lookup_pose(tf_buf, parent, child, stamp):
    """查询 parent→child 变换，返回 (tx,ty,tz,qx,qy,qz,qw) 或 None"""
    try:
        import tf2_py
        tfs = tf_buf.lookup_transform_core(parent, child, stamp)
        t = tfs.transform.translation
        r = tfs.transform.rotation
        return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
    except Exception:
        return None


def write_pose(f, ts: float, pose):
    if pose:
        f.write(f'{ts:.9f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} '
                f'{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f}\n')
    else:
        f.write(f'{ts:.9f} nan nan nan nan nan nan nan\n')


# ──────────────────────────── 雷达投影 ────────────────────────────────────────

def project_lidar_to_depth(pts, lidar_frame, K, W, H, stamp, tf_buf):
    """将雷达点云投影到相机平面，返回 16-bit 深度图 (mm) 或 None"""
    try:
        import tf2_py
        tfs = tf_buf.lookup_transform_core(CAMERA_FRAME, lidar_frame, stamp)
    except Exception:
        return None

    q = tfs.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    tv = tfs.transform.translation
    xyz = (R @ pts[:, :3].T).T + np.array([tv.x, tv.y, tv.z])

    front = xyz[:, 2] > 0.05
    xyz = xyz[front]
    if len(xyz) == 0:
        return None

    Z = xyz[:, 2]
    u = (K[0, 0] * xyz[:, 0] / Z + K[0, 2]).astype(np.int32)
    v = (K[1, 1] * xyz[:, 1] / Z + K[1, 2]).astype(np.int32)
    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, Z = u[valid], v[valid], Z[valid]
    if len(u) == 0:
        return None

    # 同一像素保留最近点（从远到近排序，近的覆盖）
    depth = np.zeros((H, W), dtype=np.uint16)
    order = np.argsort(Z)[::-1]
    depth[v[order], u[order]] = np.clip(Z[order] * 1000, 0, 65535).astype(np.uint16)
    return depth


# ──────────────────────────── 雷达消息解析 ────────────────────────────────────

def detect_lidar_msg_type(bag_path):
    """检测 /livox/lidar 的消息类型"""
    import rosbag
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, info in bag.get_type_and_topic_info().topics.items():
            if topic == '/livox/lidar':
                return info.msg_type
    return ''


def parse_lidar_msg(msg, lidar_msg_type):
    """解析雷达消息，返回 (stamp_sec, ndarray Nx4 float32, frame_id) 或 None"""
    if 'CustomMsg' in lidar_msg_type:
        pts = [(p.x, p.y, p.z, float(p.reflectivity))
               for p in msg.points
               if not (p.x == 0.0 and p.y == 0.0 and p.z == 0.0)]
        if not pts:
            return None
        return (msg.header.stamp.to_sec(),
                np.array(pts, dtype=np.float32),
                msg.header.frame_id)
    else:
        from sensor_msgs import point_cloud2 as pc2
        fields = {f.name for f in msg.fields}
        fnames = ('x', 'y', 'z', 'intensity') if 'intensity' in fields else ('x', 'y', 'z')
        pts = list(pc2.read_points(msg, field_names=fnames, skip_nans=True))
        if not pts:
            return None
        arr = np.array(pts, dtype=np.float32)
        if arr.shape[1] == 3:
            arr = np.hstack([arr, np.zeros((len(arr), 1), dtype=np.float32)])
        return (msg.header.stamp.to_sec(), arr, msg.header.frame_id)


# ──────────────────────────── 图像近似时间同步 ────────────────────────────────

class ApproxSync:
    """
    简化版 ApproximateTimeSynchronizer：维护两路消息的小缓冲，
    每收到一条消息就尝试在另一路中寻找时间差 ≤ slop_ns 的最近帧。
    找到则立即匹配并清除，避免重复处理。
    """
    def __init__(self, slop_ns=IMG_SLOP_NS, max_buf=MAX_IMG_BUF):
        self.slop_ns = slop_ns
        self.max_buf = max_buf
        self.buf_a = {}   # nsec -> msg  (RGB)
        self.buf_b = {}   # nsec -> msg  (depth)

    def add(self, msg, is_a: bool):
        """
        添加新消息（is_a=True 为 RGB，False 为 depth），
        返回匹配到的 (rgb_msg, depth_msg) 或 None。
        """
        ns = msg.header.stamp.to_nsec()
        new_buf, other_buf = (self.buf_a, self.buf_b) if is_a else (self.buf_b, self.buf_a)

        new_buf[ns] = msg

        if not other_buf:
            self._trim(new_buf, ns)
            return None

        best_ns = min(other_buf.keys(), key=lambda k: abs(k - ns))
        if abs(best_ns - ns) > self.slop_ns:
            self._trim(new_buf, ns)
            return None

        # 匹配成功
        other_msg = other_buf.pop(best_ns)
        del new_buf[ns]
        return (msg, other_msg) if is_a else (other_msg, msg)

    def _trim(self, buf, current_ns):
        if len(buf) > self.max_buf:
            del buf[min(buf.keys())]


# ──────────────────────────── Phase 1: 录制 ───────────────────────────────────

def record_bag(bag_path: str):
    """调用 rosbag record，按 Ctrl+C 停止"""
    print(f'\n[录制] 输出 bag: {bag_path}')
    print(f'[录制] 话题:')
    for t in RECORD_TOPICS:
        print(f'         {t}')
    print('[录制] 按 Ctrl+C 停止...\n')

    cmd = ['rosbag', 'record', '--output-name', bag_path] + RECORD_TOPICS
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print('\n[录制] 收到 Ctrl+C，正在停止 rosbag record...')
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait()

    if os.path.exists(bag_path):
        size_mb = os.path.getsize(bag_path) / 1024 / 1024
        print(f'[录制] 完成，bag 大小: {size_mb:.1f} MB')
    else:
        print(f'[录制] 警告: bag 文件未找到: {bag_path}')


# ──────────────────────────── Phase 2: 提取 ───────────────────────────────────

def extract_bag(bag_path: str, output_dir: str):
    """
    离线提取 bag 文件为数据集格式。
    不调用 rospy.init_node()。
    """
    import rosbag

    # 创建输出目录
    for d in ['rgb', 'depth_camera', 'lidar_pcd', 'lidar_depth']:
        os.makedirs(os.path.join(output_dir, d), exist_ok=True)

    # ── Step 1: 预加载 TF ────────────────────────────────────────────────────
    print('\n[提取] Step 1/3  预加载 TF 数据...')
    tf_buf = load_tf_buffer(bag_path)

    # ── Step 2: 检测雷达消息类型 ──────────────────────────────────────────
    print('[提取] Step 2/3  检测雷达消息类型...')
    lidar_msg_type = detect_lidar_msg_type(bag_path)
    print(f'[提取] /livox/lidar 类型: {lidar_msg_type or "未检测到"}')

    # ── Step 3: 单遍扫描，流式处理图像帧 ────────────────────────────────
    print('[提取] Step 3/3  提取图像帧（流式处理）...')

    K = None
    img_w = img_h = 0
    intrinsics_saved = False

    pose_odom = open(os.path.join(output_dir, 'poses_odom_tum.txt'), 'w')
    pose_map  = open(os.path.join(output_dir, 'poses_map_tum.txt'),  'w')
    pose_odom.write(f'# TUM: ts tx ty tz qx qy qz qw\n# odom -> {CAMERA_FRAME}\n')
    pose_map.write( f'# TUM: ts tx ty tz qx qy qz qw\n# map  -> {CAMERA_FRAME}\n')

    sync        = ApproxSync()
    lidar_buf   = []   # [(stamp_sec, ndarray, frame_id)]

    frame_cnt = 0
    odom_fail = map_fail = lidar_miss = 0

    all_topics = RECORD_TOPICS  # 一次遍历所有话题

    with rosbag.Bag(bag_path, 'r') as bag:
        total = bag.get_message_count(topic_filters=all_topics)
        print(f'[提取] bag 总消息数（目标话题）: {total}')

        for topic, msg, _ in bag.read_messages(topics=all_topics):

            # ── 跳过 TF（已在 Step 1 加载）─────────────────────────────
            if topic in ('/tf', '/tf_static'):
                continue

            # ── 雷达：维护滑动缓冲 ─────────────────────────────────────
            if topic == '/livox/lidar':
                r = parse_lidar_msg(msg, lidar_msg_type)
                if r:
                    lidar_buf.append(r)
                    if len(lidar_buf) > MAX_LIDAR_BUF:
                        lidar_buf.pop(0)
                continue

            # ── 相机内参（只保存一次）──────────────────────────────────
            if topic == '/camera/color/camera_info' and not intrinsics_saved:
                K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
                img_w, img_h = msg.width, msg.height
                info_dict = {
                    'width':             msg.width,
                    'height':            msg.height,
                    'distortion_model':  msg.distortion_model,
                    'K': list(msg.K),
                    'D': list(msg.D),
                    'R': list(msg.R),
                    'P': list(msg.P),
                }
                with open(os.path.join(output_dir, 'camera_intrinsics.yaml'), 'w') as f:
                    yaml.dump(info_dict, f, default_flow_style=False)
                intrinsics_saved = True
                print(f'[提取] 内参已保存  '
                      f'fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
                      f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}  '
                      f'{img_w}x{img_h}')
                continue

            # ── 图像：近似时间同步 ─────────────────────────────────────
            is_rgb = (topic == '/camera/color/image_raw')
            is_dep = (topic == '/camera/aligned_depth_to_color/image_raw')
            if not (is_rgb or is_dep):
                continue

            result = sync.add(msg, is_a=is_rgb)
            if result is None:
                continue

            rgb_msg, depth_msg = result
            stamp  = rgb_msg.header.stamp
            ts_str = f'{stamp.secs}_{stamp.nsecs:09d}'
            ts_sec = stamp.to_sec()

            # ── RGB 保存 ─────────────────────────────────────────────
            try:
                rgb_img = ros_image_to_numpy(rgb_msg)   # BGR, uint8
                cv2.imwrite(os.path.join(output_dir, 'rgb', f'{ts_str}.png'), rgb_img)
            except Exception as e:
                print(f'  [警告] RGB转换失败: {e}')
                continue

            # ── 相机深度保存 ──────────────────────────────────────────
            try:
                cam_d = ros_image_to_numpy(depth_msg)   # uint16, mm
                if cam_d.shape[:2] != (img_h, img_w) and img_h > 0:
                    cam_d = cv2.resize(cam_d, (img_w, img_h),
                                       interpolation=cv2.INTER_NEAREST)
                cv2.imwrite(os.path.join(output_dir, 'depth_camera', f'{ts_str}.png'), cam_d)
            except Exception as e:
                print(f'  [警告] 深度转换失败: {e}')
                continue

            # ── 雷达：找最近帧 ────────────────────────────────────────
            lf = None
            if lidar_buf:
                best = min(lidar_buf, key=lambda x: abs(x[0] - ts_sec))
                if abs(best[0] - ts_sec) <= LIDAR_SLOP:
                    lf = best

            if lf is not None:
                pts, lframe = lf[1], lf[2]
                save_pcd_binary(pts, os.path.join(output_dir, 'lidar_pcd', f'{ts_str}.pcd'))
                if K is not None:
                    ld = project_lidar_to_depth(pts, lframe, K, img_w, img_h, stamp, tf_buf)
                    if ld is not None:
                        cv2.imwrite(
                            os.path.join(output_dir, 'lidar_depth', f'{ts_str}.png'), ld)
            else:
                lidar_miss += 1
                if lidar_miss <= 5:
                    print(f'  [警告] 雷达缺失（帧 {frame_cnt}），'
                          f'请确认 /livox/lidar 正在发布')

            # ── 位姿查询 ──────────────────────────────────────────────
            op = lookup_pose(tf_buf, 'odom', CAMERA_FRAME, stamp)
            mp = lookup_pose(tf_buf, 'map',  CAMERA_FRAME, stamp)
            if op is None: odom_fail += 1
            if mp is None: map_fail  += 1
            write_pose(pose_odom, ts_sec, op)
            write_pose(pose_map,  ts_sec, mp)

            frame_cnt += 1
            if frame_cnt % 100 == 0:
                print(f'  已处理 {frame_cnt} 帧 | '
                      f'雷达缺失: {lidar_miss} | '
                      f'odom失败: {odom_fail} | map失败: {map_fail}')

    pose_odom.close()
    pose_map.close()

    print(f'\n[提取] 完成！')
    print(f'  总帧数  : {frame_cnt}')
    print(f'  雷达缺失: {lidar_miss}')
    print(f'  odom失败: {odom_fail}')
    print(f'  map失败 : {map_fail}')
    print(f'  输出目录: {output_dir}/')
    print(f'  ├── rgb/                 # {frame_cnt} 帧 RGB 图像')
    print(f'  ├── depth_camera/        # {frame_cnt} 帧相机深度图')
    print(f'  ├── lidar_pcd/           # {frame_cnt - lidar_miss} 帧雷达点云')
    print(f'  ├── lidar_depth/         # 雷达投影深度图')
    print(f'  ├── camera_intrinsics.yaml')
    print(f'  ├── poses_odom_tum.txt')
    print(f'  └── poses_map_tum.txt')


# ──────────────────────────── 入口 ────────────────────────────────────────────

def main():
    # 模式 1: extract bag_path output_dir
    if len(sys.argv) == 4 and sys.argv[1] == 'extract':
        bag_path   = os.path.abspath(sys.argv[2])
        output_dir = os.path.abspath(sys.argv[3])
        if not os.path.exists(bag_path):
            print(f'[错误] bag 文件不存在: {bag_path}')
            sys.exit(1)
        os.makedirs(output_dir, exist_ok=True)
        extract_bag(bag_path, output_dir)
        return

    # 模式 2: <output_dir>  → 录制后自动提取
    if len(sys.argv) >= 2 and sys.argv[1] != 'extract':
        output_dir = os.path.abspath(sys.argv[1])
    else:
        output_dir = os.path.abspath(os.path.expanduser('~/collected_data'))

    bag_path = output_dir + '.bag'
    os.makedirs(output_dir, exist_ok=True)

    print('=' * 60)
    print(' RealSense + Livox Mid360 数据采集 v2')
    print('=' * 60)
    print(f' bag 输出  : {bag_path}')
    print(f' 数据集目录: {output_dir}')
    print('=' * 60)

    # 阶段 1: 录制
    record_bag(bag_path)

    # 阶段 2: 提取
    if not os.path.exists(bag_path):
        print(f'[错误] bag 文件未生成: {bag_path}')
        sys.exit(1)

    print('\n[提取] 开始离线提取...')
    extract_bag(bag_path, output_dir)


if __name__ == '__main__':
    main()
