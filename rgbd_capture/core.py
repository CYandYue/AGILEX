"""RGB-D association and geometry. No ROS master or hardware required."""
import bisect
import copy
import json
import math
from collections import Counter, deque
from dataclasses import asdict, dataclass

import genpy
import numpy as np
import tf2_py
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_matrix, quaternion_slerp
from tf2_msgs.msg import TFMessage

RGB = '/camera/color/image_raw'
DEPTH = '/camera/aligned_depth_to_color/image_raw'
RGB_INFO = '/camera/color/camera_info'
DEPTH_INFO = '/camera/aligned_depth_to_color/camera_info'
ODOM = '/lio/odometry'
GROUP = ['/dataset/rgb/image_raw', '/dataset/depth/image_raw',
         '/dataset/rgb/camera_info', '/dataset/depth/camera_info',
         '/dataset/camera_pose', '/dataset/body_to_camera', '/dataset/frame']


@dataclass
class Config:
    rate: float = 6.0
    rgbd_slop: float = .01
    max_odom_gap: float = .25
    wait: float = 3.0
    camera_time_offset: float = 0.0  # t_lio = t_rgb + offset
    depth_scale: float = .001       # meters / integer depth unit
    world: str = 'lio_map'
    body: str = 'body'
    optical: str = 'camera_color_optical_frame'
    image_buffer: int = 60

    def validate(self):
        for k in ('rate', 'rgbd_slop', 'max_odom_gap', 'wait', 'depth_scale'):
            if not math.isfinite(getattr(self, k)) or getattr(self, k) <= 0:
                raise ValueError(k + ' must be finite and positive')
        if self.rate > 10 or self.rgbd_slop >= .05 or self.max_odom_gap > 1:
            raise ValueError('Require rate <= 10 Hz, RGB-D slop < 50 ms, odom gap <= 1 s')
        if not math.isfinite(self.camera_time_offset):
            raise ValueError('camera_time_offset must be finite')
        if self.image_buffer < 2 or not all((self.world, self.body, self.optical)):
            raise ValueError('Invalid buffer or frame names')


def stamp(ns):
    return genpy.Time(int(ns) // 1000000000, int(ns) % 1000000000)


def json_msg(value):
    return String(json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False))


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    xyz = np.array([p.x, p.y, p.z], dtype=float)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(xyz).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > .01:
        raise ValueError('Non-finite pose or non-unit quaternion')
    out = quaternion_matrix(quat / np.linalg.norm(quat))
    out[:3, 3] = xyz
    return out


def transform_matrix(msg):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = (msg.transform.translation.x,
                                              msg.transform.translation.y, msg.transform.translation.z)
    p.orientation = msg.transform.rotation
    return pose_matrix(p)


def interpolate(a, b, target_ns):
    ta, tb = a.header.stamp.to_nsec(), b.header.stamp.to_nsec()
    if tb <= ta or not ta <= target_ns <= tb:
        raise ValueError('Interpolation requires two distinct bracketing odometry samples')
    alpha = (target_ns-ta)/(tb-ta)
    ma, mb = pose_matrix(a.pose.pose), pose_matrix(b.pose.pose)
    out = quaternion_matrix(quaternion_slerp(quaternion_from_matrix(ma), quaternion_from_matrix(mb), alpha))
    out[:3, 3] = (1-alpha)*ma[:3, 3] + alpha*mb[:3, 3]
    return out, alpha


def image_array(msg):
    """Strict encoding, endian and padded-row support; never resize silently."""
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'rgba8': ('u1', 4),
               'bgra8': ('u1', 4), 'mono8': ('u1', 1), '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in formats:
        raise ValueError('Unsupported image encoding: ' + msg.encoding)
    fmt, channels = formats[msg.encoding]
    dt = np.dtype(('>' if msg.is_bigendian else '<') + fmt)
    row = msg.width * channels * dt.itemsize
    if msg.width <= 0 or msg.height <= 0 or msg.step < row or len(msg.data) != msg.step*msg.height:
        raise ValueError('Invalid image dimensions, row stride or payload length')
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (msg.step, dt.itemsize) if channels == 1 else (msg.step, dt.itemsize*channels, dt.itemsize)
    return np.ndarray(shape, dtype=dt, buffer=msg.data, strides=strides)


def calibration(info):
    return dict(width=info.width, height=info.height, distortion_model=info.distortion_model,
                K=list(info.K), D=list(info.D), R=list(info.R), P=list(info.P),
                binning_x=info.binning_x, binning_y=info.binning_y,
                roi=dict(x_offset=info.roi.x_offset, y_offset=info.roi.y_offset,
                         width=info.roi.width, height=info.roi.height, do_rectify=info.roi.do_rectify),
                frame_id=info.header.frame_id)


def validate_rgbd(rgb, depth, ci, di, optical):
    if rgb.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
        raise ValueError('Unsupported color encoding')
    if depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Depth must be 16UC1 or 32FC1')
    image_array(rgb)
    d = image_array(depth)
    for msg in (rgb, depth, ci, di):
        if (msg.width, msg.height) != (rgb.width, rgb.height) or msg.header.frame_id != optical:
            raise ValueError('RGB/aligned-depth/calibration dimensions or optical frames disagree')
    for info in (ci, di):
        numbers = list(info.K)+list(info.D)+list(info.R)+list(info.P)
        if not np.isfinite(numbers).all() or info.K[0] <= 0 or info.K[4] <= 0:
            raise ValueError('Invalid CameraInfo')
        if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or any(
                (info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)):
            raise ValueError('ROI/binning requires explicit calibration handling; unsupported')
        if not np.allclose(np.array(info.R).reshape(3, 3), np.eye(3), atol=1e-6):
            raise ValueError('Input must use the unrectified optical frame (R=I)')
    if calibration(ci) != calibration(di):
        raise ValueError('Aligned depth must use the color pixel model; CameraInfo differs')
    with np.errstate(invalid='ignore'):
        return float(np.mean(np.isfinite(d) & (d > 0)))


class Engine:
    """Single-threaded engine. Writer is called only by the owning event loop."""
    def __init__(self, writer, config):
        config.validate()
        self.write, self.cfg = writer, config
        self.stats = Counter()
        self.images = {RGB: deque(), DEPTH: deque()}
        self.infos = {RGB_INFO: deque(maxlen=240), DEPTH_INFO: deque(maxlen=240)}
        self.odoms = deque()
        self.pending = deque()
        self.last = {}
        self.last_seq = {}
        self.next_due = None
        self.static = {}
        self.tf = tf2_py.BufferCore(genpy.Duration(60))
        self.first_frame_ns = self.last_frame_ns = None
        self.latest_frame_wall = None
        self.max_lio_age = 0.0
        self.last_error = None

    def ingest(self, topic, msg, arrival_ns, wall):
        self.stats['received:'+topic] += 1
        if topic in (RGB, DEPTH, ODOM):
            ns = msg.header.stamp.to_nsec()
            if ns <= 0:
                raise ValueError('Zero/negative source timestamp: '+topic)
            previous = self.last.get(topic)
            if previous is not None and ns < previous:
                raise ValueError('Source clock moved backward: '+topic)
            if previous == ns:
                self.stats['duplicate:'+topic] += 1
                return
            self.last[topic] = ns
            previous_seq = self.last_seq.get(topic)
            if previous_seq is not None and msg.header.seq > previous_seq+1:
                self.stats['source_sequence_gaps:'+topic] += msg.header.seq-previous_seq-1
            self.last_seq[topic] = msg.header.seq
        if topic in self.images:
            self.images[topic].append(msg)
            if len(self.images[topic]) > self.cfg.image_buffer:
                self.images[topic].popleft()
                self.stats['unpaired:'+topic] += 1
            self._pair(wall)
        elif topic == ODOM:
            if msg.header.frame_id != self.cfg.world or msg.child_frame_id != self.cfg.body:
                raise ValueError('Odometry frame changed or does not match configuration')
            pose_matrix(msg.pose.pose)
            self.odoms.append(msg)
            # Time and count limits prevent indefinite growth under bad clocks.
            cutoff = ns-int((self.cfg.wait+2)*1e9)
            while len(self.odoms) > 2 and (self.odoms[1].header.stamp.to_nsec() < cutoff or len(self.odoms) > 2000):
                self.odoms.popleft()
            self.max_lio_age = max(self.max_lio_age, (arrival_ns-ns)/1e9)
            self.write(topic, msg, arrival_ns)
        elif topic in self.infos:
            self.infos[topic].append(copy.deepcopy(msg))
            self.write(topic, msg, arrival_ns)
        elif topic == '/tf_static':
            for t in msg.transforms:
                transform_matrix(t)
                if not t.header.frame_id or not t.child_frame_id or t.header.frame_id == t.child_frame_id:
                    raise ValueError('Invalid static transform')
                # Match TF's last-parent semantics; this driver publishes
                # equivalent color/aligned-color chains for the optical frame.
                self.static[t.child_frame_id] = copy.deepcopy(t)
                self.tf.set_transform_static(t, 'capture')
            self.write(topic, TFMessage(list(self.static.values())), arrival_ns)
        else:
            self.write(topic, msg, arrival_ns)
        if topic in (RGB, DEPTH, ODOM, RGB_INFO, DEPTH_INFO, '/tf_static'):
            self.drain(wall)

    def _pair(self, wall):
        a, b = self.images[RGB], self.images[DEPTH]
        slop = int(self.cfg.rgbd_slop*1e9)
        while a and b:
            rgb = a[0]
            t = rgb.header.stamp.to_nsec()
            # Exact match first, otherwise nearest currently buffered depth.
            index = min(range(len(b)), key=lambda i: abs(b[i].header.stamp.to_nsec()-t))
            dt = b[index].header.stamp.to_nsec()-t
            if abs(dt) <= slop:
                a.popleft()
                dep = b[index]
                del b[index]
                self.stats['paired'] += 1
                if self.next_due is not None and t < self.next_due:
                    self.stats['rate_filtered'] += 1
                    continue
                period = int(1e9/self.cfg.rate)
                if self.next_due is None:
                    self.next_due = t
                self.next_due += ((t-self.next_due)//period+1)*period
                self.pending.append((rgb, dep, wall))
            elif b[0].header.stamp.to_nsec() < t-slop:
                b.popleft()
                self.stats['unpaired:'+DEPTH] += 1
            else:
                a.popleft()
                self.stats['unpaired:'+RGB] += 1

    def _info(self, topic, t):
        candidates = [m for m in self.infos[topic] if m.header.stamp.to_nsec() <= t+int(self.cfg.rgbd_slop*1e9)]
        if not candidates:
            raise LookupError('missing_camera_info')
        return max(candidates, key=lambda m: m.header.stamp.to_nsec())

    def drain(self, wall, final=False):
        while self.pending:
            rgb, dep, queued = self.pending[0]
            try:
                group = self._make_group(rgb, dep)
            except LookupError as e:
                if not final and wall-queued < self.cfg.wait:
                    break
                self.stats['dropped:'+str(e)] += 1
                self.last_error = str(e)
                self.pending.popleft()
                continue
            except ValueError as e:
                self.stats['dropped:invalid_rgbd'] += 1
                self.last_error = str(e)
                self.pending.popleft()
                continue
            self.pending.popleft()
            ns = rgb.header.stamp.to_nsec()
            for topic, message in zip(GROUP, group):
                self.write(topic, message, ns)
            self.stats['frames'] += 1
            self.first_frame_ns = ns if self.first_frame_ns is None else self.first_frame_ns
            self.last_frame_ns = ns
            self.latest_frame_wall = wall

    def _make_group(self, rgb, dep):
        ns = rgb.header.stamp.to_nsec()
        target = ns+round(self.cfg.camera_time_offset*1e9)
        times = [m.header.stamp.to_nsec() for m in self.odoms]
        j = bisect.bisect_left(times, target)
        if j == 0 and len(times) >= 2 and target == times[0]:
            j = 1
        if j == 0 or j >= len(times):
            raise LookupError('no_bracketing_odometry')
        a, b = self.odoms[j-1], self.odoms[j]
        if (times[j]-times[j-1])/1e9 > self.cfg.max_odom_gap:
            raise LookupError('odometry_gap')
        ci = self._info(RGB_INFO, ns)
        di = self._info(DEPTH_INFO, dep.header.stamp.to_nsec())
        valid = validate_rgbd(rgb, dep, ci, di, self.cfg.optical)
        if valid == 0:
            raise ValueError('Depth has no valid pixels')
        try:
            ext = self.tf.lookup_transform_core(self.cfg.body, self.cfg.optical, genpy.Time())
        except (tf2_py.LookupException, tf2_py.ConnectivityException, tf2_py.ExtrapolationException):
            raise LookupError('missing_static_extrinsics')
        # An accepted session must retain the same body-to-camera geometry.
        matrix_ext = transform_matrix(ext)
        if hasattr(self, 'accepted_extrinsic') and not np.allclose(matrix_ext, self.accepted_extrinsic, atol=1e-8):
            raise RuntimeError('Body-to-camera extrinsics changed during recording; start a new bag')
        self.accepted_extrinsic = matrix_ext
        body, alpha = interpolate(a, b, target)
        camera = body @ matrix_ext
        index = self.stats['frames']
        pose = PoseStamped()
        pose.header.seq, pose.header.stamp, pose.header.frame_id = index, stamp(ns), self.cfg.world
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = camera[:3, 3]
        q = quaternion_from_matrix(camera)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        ext = copy.deepcopy(ext)
        ext.header.seq, ext.header.stamp = index, stamp(ns)
        copied = [copy.deepcopy(m) for m in (rgb, dep, ci, di)]
        for m in copied:
            m.header.seq = index
        copied[2].header.stamp = rgb.header.stamp
        copied[3].header.stamp = dep.header.stamp
        meta = dict(frame_index=index, rgb_stamp_ns=ns, depth_stamp_ns=dep.header.stamp.to_nsec(),
                    rgb_original_seq=rgb.header.seq, depth_original_seq=dep.header.seq,
                    rgbd_delta_ns=dep.header.stamp.to_nsec()-ns, pose_query_stamp_ns=target,
                    odom_before_ns=times[j-1], odom_after_ns=times[j], interpolation_alpha=alpha,
                    rgb_info_original_stamp_ns=ci.header.stamp.to_nsec(),
                    depth_info_original_stamp_ns=di.header.stamp.to_nsec(),
                    depth_encoding=dep.encoding, depth_scale_m=self.cfg.depth_scale if dep.encoding=='16UC1' else 1.0,
                    depth_valid_fraction=valid, camera_frame=self.cfg.optical,
                    pose_convention='T_world_camera_optical', world_frame=self.cfg.world,
                    hardware_sync_verified=False)
        return copied+[pose, ext, json_msg(meta)]

    def report(self):
        duration = (self.last_frame_ns-self.first_frame_ns)/1e9 if self.first_frame_ns is not None else 0
        return dict(counts=dict(self.stats), frames=self.stats['frames'],
                    actual_rate_hz=(self.stats['frames']-1)/duration if duration > 0 else 0,
                    max_observed_lio_arrival_age_s=self.max_lio_age,
                    first_frame_ns=self.first_frame_ns, last_frame_ns=self.last_frame_ns,
                    pending=len(self.pending), last_rejection=self.last_error, configuration=asdict(self.cfg))
