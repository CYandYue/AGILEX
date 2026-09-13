#!/usr/bin/env python3
"""Record, validate and export self-contained RGB-D/LIO ROS1 bags."""
import argparse
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict

import numpy as np
import rosbag
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from raw_recording import RawRecording, merge_bags

from core import (Config, Engine, RGB, DEPTH, RGB_INFO, DEPTH_INFO, ODOM, GROUP,
                  stamp, json_msg, image_array, calibration, interpolate,
                  pose_matrix, transform_matrix, validate_rgbd)

SESSION = '/dataset/session'
REPORT = '/dataset/report'
EXTRA_INFO = '/camera/depth/camera_info'
RAW_TOPICS = ['/livox/lidar_192_168_1_113', '/livox/lidar_192_168_1_154',
              '/livox/imu_192_168_1_113', '/livox/imu_192_168_1_154', '/imu/data_raw']
META_TOPICS = ['/camera/color/metadata', '/camera/depth/metadata',
               '/camera/extrinsics/depth_to_color']


def source_identity():
    here = Path(__file__).resolve().parent
    try:
        commit = subprocess.check_output(['git', '-C', str(here), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(['git', '-C', str(here), 'status', '--porcelain'], text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        commit, dirty = None, None
    return dict(git_commit=commit, git_dirty=dirty,
                sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in here.glob('*.py')})


class Writer:
    def __init__(self, bag):
        self.bag = bag
        self.raw_types = {}

    def __call__(self, topic, msg, ns):
        if hasattr(msg, '_buff'):  # AnyMsg: no point-by-point Python deserialization.
            h = dict(msg._connection_header)
            key = (h['type'], h['md5sum'])
            if key not in self.raw_types:
                self.raw_types[key] = type('SerializedMessage', (), {'_md5sum': h['md5sum'], '_full_text': h['message_definition']})
            cls = self.raw_types[key]
            self.bag.write(topic, (h['type'], msg._buff, h['md5sum'], cls), stamp(ns), raw=True, connection_header=h)
        elif topic in ('/tf_static', SESSION):
            h = dict(topic=topic, type=msg._type, md5sum=msg._md5sum,
                     message_definition=msg._full_text, latching='1', callerid='/rgbd_capture')
            self.bag.write(topic, msg, stamp(ns), connection_header=h)
        else:
            self.bag.write(topic, msg, stamp(ns))


def config_from_args(args):
    return Config(rate=args.rate, rgbd_slop=args.rgbd_slop, max_odom_gap=args.max_odom_gap,
                  wait=args.wait, camera_time_offset=args.camera_time_offset, depth_scale=args.depth_scale,
                  world=args.world, body=args.body, optical=args.optical)


def record(args):
    import rospy
    import rosgraph
    cfg = config_from_args(args)
    cfg.validate()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        raise ValueError('duration must be positive')
    if not math.isfinite(args.startup_timeout) or not math.isfinite(args.min_free_gb) or args.startup_timeout <= 0 or args.min_free_gb < 0:
        raise ValueError('Invalid startup timeout or disk threshold')
    dest = Path(args.output).expanduser().resolve()
    if dest.suffix != '.bag':
        raise ValueError('Output filename must end in .bag')
    active, partial, report_path = (Path(str(dest)+suffix) for suffix in ('.active', '.partial', '.report.json'))
    recovery = [Path(str(dest)+suffix) for suffix in ('.raw.bag','.raw.bag.active','.raw.log','.merge.active')]
    if any(p.exists() for p in (dest, active, partial, report_path, *recovery)):
        raise FileExistsError('Output or recovery file already exists; use a new filename')
    dest.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(dest.parent).free < args.min_free_gb*1024**3:
        raise RuntimeError('Insufficient free disk space')
    # A hung master must not make the CLI hang forever at init_node.
    initialized = threading.Event()
    def watchdog():
        if not initialized.wait(15):
            print('ROS initialization timed out. Check ROS_MASTER_URI and ROS_IP/ROS_HOSTNAME.', file=sys.stderr, flush=True)
            os._exit(2)
    threading.Thread(target=watchdog, daemon=True).start()
    try:
        rosgraph.Master('/rgbd_capture_check').getPid()
        rospy.init_node('rgbd_capture', anonymous=True, disable_signals=True)
    finally:
        initialized.set()
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Live capture requires wall-clock ROS time; use_sim_time must be false')

    events = queue.Queue(maxsize=512)
    stop, fatal = threading.Event(), threading.Event()
    state_lock = threading.Lock()
    state = dict(bytes=0, error=None, accepting_images=True, peak_bytes=0, peak_events=0)
    old_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
    def callback(msg, topic):
        if topic in (RGB, DEPTH) and not state['accepting_images']:
            return
        size = len(msg.data) if isinstance(msg, Image) else (len(msg._buff) if hasattr(msg, '_buff') else 4096)
        with state_lock:
            if state['bytes']+size > 128*1024*1024:
                state['error'] = 'Capture queue exceeded 128 MiB; disk/CPU cannot keep up'
                fatal.set()
                return
            state['bytes'] += size
            state['peak_bytes'] = max(state['peak_bytes'], state['bytes'])
        event = (topic, msg, rospy.Time.now().to_nsec(), time.monotonic(), size)
        try:
            events.put_nowait(event)
            state['peak_events'] = max(state['peak_events'], events.qsize())
        except queue.Full:
            with state_lock:
                state['bytes'] -= size
                state['error'] = 'Capture event queue overflow; recording marked incomplete'
            fatal.set()

    # Subscribe after opening the bag, to capture latched calibration/TF once.
    subscribers, bag, engine = [], None, None
    raw_recording = None
    raw_counts = {}
    error = None
    started = time.monotonic()
    tail_deadline = None
    last_status = last_flush = last_disk = started
    session = dict(schema_version=1, config=asdict(cfg), source=source_identity(),
                   created_unix_ns=time.time_ns(), raw_sensors=args.raw_sensors,
                   pose_convention='T_world_camera_optical',
                   optical_axes='x right, y down, z forward',
                   timestamps='Source headers preserved; dataset groups use RGB acquisition time as bag time',
                   camera_time_offset_convention='t_lio = t_rgb + camera_time_offset',
                   depth_units='16UC1 * configured scale in meters; 32FC1 in meters; nonpositive/nonfinite invalid',
                   distortion_policy='Raw RGB and aligned depth preserved; use CameraInfo, optional export --rectify',
                   hardware_sync_verified=False,
                   calibration_status='Vendor/nominal mounting initial values, not a calibration certificate',
                   parameters={p: rospy.get_param(p, {}) for p in ('/lio', '/lio_mounts', '/camera')})
    try:
        # Reserve the pathname exclusively before rosbag opens it.
        with active.open('xb'):
            pass
        bag = rosbag.Bag(str(active), 'w', compression=args.compression, chunk_threshold=4*1024*1024)
        writer = Writer(bag)
        writer(SESSION, json_msg(session), rospy.Time.now().to_nsec())
        engine = Engine(writer, cfg)
        if args.raw_sensors:
            raw_recording = RawRecording(dest, RAW_TOPICS, args.compression)
        topics = {RGB: Image, DEPTH: Image, RGB_INFO: CameraInfo, DEPTH_INFO: CameraInfo,
                  EXTRA_INFO: CameraInfo, ODOM: Odometry, '/tf': TFMessage, '/tf_static': TFMessage}
        for topic in META_TOPICS:
            topics[topic] = rospy.AnyMsg
        for topic, cls in topics.items():
            subscribers.append(rospy.Subscriber(topic, cls, callback, callback_args=topic,
                                                queue_size=120, buff_size=8*1024*1024, tcp_nodelay=True))
        print('Recording:', dest, flush=True)
        print('Waiting for valid RGB-D, CameraInfo, LIO brackets and static camera extrinsics. Ctrl-C stops images and drains the pose tail.', flush=True)
        while True:
            now = time.monotonic()
            if raw_recording:
                raw_recording.check_running()
            if fatal.is_set():
                raise RuntimeError(state['error'])
            if rospy.is_shutdown():
                raise RuntimeError('ROS shut down before clean capture stop')
            if args.duration is not None and now-started >= args.duration:
                stop.set()
            if stop.is_set() and tail_deadline is None:
                state['accepting_images'] = False
                tail_deadline = now+cfg.wait
                print('Finishing queued frames; waiting for LIO tail...', flush=True)
            if tail_deadline is not None and now >= tail_deadline:
                break
            try:
                topic, msg, ns, arrival, size = events.get(timeout=.05)
            except queue.Empty:
                engine.drain(now)
            else:
                with state_lock:
                    state['bytes'] -= size
                engine.ingest(topic, msg, ns, arrival)
            if now-last_disk >= 2:
                if shutil.disk_usage(dest.parent).free < args.min_free_gb*1024**3:
                    raise RuntimeError('Disk space fell below configured reserve')
                last_disk = now
            if now-last_flush >= 2:
                bag.flush()
                last_flush = now
            if engine.stats['frames'] == 0 and now-started > args.startup_timeout:
                raise RuntimeError('No valid frame before startup timeout: '+str(engine.report()))
            if engine.latest_frame_wall and now-engine.latest_frame_wall > args.startup_timeout and tail_deadline is None:
                raise RuntimeError('Valid frames stopped arriving: '+str(engine.report()))
            if now-last_status >= 5:
                r = engine.report()
                print('frames=%d rate=%.2f Hz pending=%d last_rejection=%s' % (
                    r['frames'], r['actual_rate_hz'], r['pending'], r['last_rejection']), flush=True)
                last_status = now
    except BaseException as exc:
        error = '%s: %s' % (type(exc).__name__, exc)
    finally:
        state['accepting_images'] = False
        for subscriber in subscribers:
            subscriber.unregister()
        try:
            if raw_recording:
                raw_counts = raw_recording.stop()
                imu_source = session['parameters']['/lio'].get('common',{}).get('imu_topic','/imu/data_raw')
                selected = '/livox/imu_192_168_1_113' if imu_source == '/sensors/mid360_a/imu' else imu_source
                if raw_counts.get(selected,0) == 0:
                    raise RuntimeError('Raw recording missing selected IMU '+selected)
            if engine:
                # Process callbacks already queued before unsubscribe.
                while not events.empty() and error is None:
                    topic, msg, ns, arrival, _ = events.get_nowait()
                    engine.ingest(topic, msg, ns, arrival)
                engine.drain(time.monotonic(), final=True)
                summary = engine.report()
            else:
                summary = dict(frames=0)
            if not summary['frames'] and error is None:
                error = 'No complete frames recorded'
            summary.update(status='INCOMPLETE' if error else 'RECORDED', error=error,
                           elapsed_wall_s=time.monotonic()-started, raw_topic_counts=raw_counts,
                           queue_peak_events=state['peak_events'], queue_peak_bytes=state['peak_bytes'])
            if bag:
                Writer(bag)(REPORT, json_msg(summary), rospy.Time.now().to_nsec())
        except BaseException as exc:
            error = error or str(exc)
            summary = dict(status='INCOMPLETE', error=error)
        finally:
            if bag:
                try:
                    bag.close()
                except Exception as exc:
                    error = error or str(exc)
            rospy.signal_shutdown('capture finished')
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    if error is None:
        try:
            candidate = active
            if raw_recording:
                needed = active.stat().st_size+raw_recording.path.stat().st_size+args.min_free_gb*1024**3
                if shutil.disk_usage(dest.parent).free < needed:
                    raise RuntimeError('Insufficient space for final merge; raw/core files retained')
                candidate = Path(str(dest)+'.merge.active')
                print('Merging raw sensors into the final bag...',flush=True)
                merge_bags(active,raw_recording.path,candidate,args.compression)
            summary['validation'] = check_bag(candidate)
            summary['status'] = 'PASS'
        except Exception as exc:
            error = 'Post-record validation failed: '+str(exc)
    if error:
        summary.update(status='INCOMPLETE', error=error)
        if active.exists():
            active.rename(partial)
    else:
        candidate.rename(dest)
        if raw_recording:
            active.unlink()
            raw_recording.path.unlink()
            raw_recording.log_path.unlink()
    report_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    if error:
        raise RuntimeError('Incomplete recording retained at %s: %s' % (partial, error))
    print('Validated bag:', dest, flush=True)


def frame_groups(bag):
    """Groups are contiguous at a unique RGB bag timestamp; bounded image memory."""
    group, current = {}, None
    for topic, msg, t in bag.read_messages(topics=GROUP):
        ns = t.to_nsec()
        if current is not None and ns != current:
            if set(group) != set(GROUP):
                raise ValueError('Incomplete frame group at '+str(current))
            yield current, group
            group = {}
        if topic in group:
            raise ValueError('Duplicate topic in frame group')
        current = ns
        group[topic] = msg
    if group:
        if set(group) != set(GROUP):
            raise ValueError('Incomplete last frame group')
        yield current, group


def read_session(bag):
    sessions = list(bag.read_messages(topics=[SESSION]))
    if len(sessions) != 1:
        raise ValueError('Expected exactly one session metadata message')
    value = json.loads(sessions[0].message.data)
    if value.get('schema_version') != 1:
        raise ValueError('Unsupported dataset schema')
    return value


def check_bag(path):
    with rosbag.Bag(str(path), 'r') as bag:
        session = read_session(bag)
        cfg = Config(**session['config'])
        cfg.validate()
        reports = list(bag.read_messages(topics=[REPORT]))
        if len(reports) != 1:
            raise ValueError('Missing final capture report: recording may be incomplete')
        saved_report = json.loads(reports[0].message.data)
        if saved_report.get('error') or saved_report.get('status') not in ('RECORDED', 'PASS'):
            raise ValueError('Recorder marked this bag incomplete')
        if session.get('raw_sensors'):
            counts = saved_report.get('raw_topic_counts',{})
            if not counts or any(bag.get_message_count(t) != count for t,count in counts.items()):
                raise ValueError('Merged raw message counts disagree with recorder report')
        odoms = {}
        last = -1
        for _, msg, _ in bag.read_messages(topics=[ODOM]):
            ns = msg.header.stamp.to_nsec()
            if ns <= last or msg.header.frame_id != cfg.world or msg.child_frame_id != cfg.body:
                raise ValueError('Nonmonotonic odometry or wrong frames')
            pose_matrix(msg.pose.pose)
            odoms[ns], last = msg, ns
        if bag.get_message_count('/tf') == 0 or bag.get_message_count('/tf_static') == 0:
            raise ValueError('Missing dynamic or static TF')
        import tf2_py
        import genpy
        static_buffer = tf2_py.BufferCore(genpy.Duration(60))
        for _, msg, _ in bag.read_messages(topics=['/tf_static']):
            for tr in msg.transforms:
                # rosbag may generate equivalent message classes dynamically;
                # normalize to installed geometry types before entering tf2.
                wire = io.BytesIO()
                tr.serialize(wire)
                canonical = TransformStamped().deserialize(wire.getvalue())
                static_buffer.set_transform_static(canonical, 'validation')
        expected_ext = transform_matrix(static_buffer.lookup_transform_core(cfg.body, cfg.optical, genpy.Time()))
        has_world_body = False
        for _, msg, _ in bag.read_messages(topics=['/tf']):
            for tr in msg.transforms:
                if tr.header.frame_id == cfg.world and tr.child_frame_id == cfg.body:
                    transform_matrix(tr)
                    has_world_body = True
        if not has_world_body:
            raise ValueError('Recorded TF lacks the LIO world-to-body edge')
        count, first, previous = 0, None, -1
        min_valid, max_delta, max_gap = 1., 0., 0.
        for ns, g in frame_groups(bag):
            rgb, depth, ci, di, pose, ext, annotation = [g[t] for t in GROUP]
            meta = json.loads(annotation.data)
            if ns <= previous or meta['frame_index'] != count or any(m.header.seq != count for m in (rgb, depth, ci, di, pose, ext)):
                raise ValueError('Frame IDs are not contiguous or timestamps are not increasing')
            if ns != rgb.header.stamp.to_nsec() or ns != pose.header.stamp.to_nsec() or ns != meta['rgb_stamp_ns']:
                raise ValueError('RGB/pose association timestamp mismatch')
            if ext.header.stamp.to_nsec() != ns:
                raise ValueError('Extrinsic association timestamp mismatch')
            if depth.header.stamp.to_nsec() != meta['depth_stamp_ns'] or ci.header.stamp != rgb.header.stamp or di.header.stamp != depth.header.stamp:
                raise ValueError('Depth/CameraInfo timestamp mismatch')
            valid = validate_rgbd(rgb, depth, ci, di, cfg.optical)
            target = ns+round(cfg.camera_time_offset*1e9)
            if meta['rgbd_delta_ns'] != meta['depth_stamp_ns']-ns or meta['pose_query_stamp_ns'] != target or abs(meta['depth_stamp_ns']-ns)/1e9 > cfg.rgbd_slop:
                raise ValueError('Invalid synchronization offset')
            a, b = odoms[meta['odom_before_ns']], odoms[meta['odom_after_ns']]
            gap = (b.header.stamp.to_nsec()-a.header.stamp.to_nsec())/1e9
            if gap > cfg.max_odom_gap:
                raise ValueError('Excessive odometry gap')
            body, alpha = interpolate(a, b, target)
            if pose.header.frame_id != cfg.world or ext.header.frame_id != cfg.body or ext.child_frame_id != cfg.optical:
                raise ValueError('Wrong output pose/extrinsic frames')
            matrix_ext = transform_matrix(ext)
            if not np.allclose(matrix_ext, expected_ext, atol=1e-7):
                raise ValueError('Per-frame extrinsic disagrees with recorded TF')
            if not np.allclose(pose_matrix(pose.pose), body @ matrix_ext, atol=1e-6) or abs(alpha-meta['interpolation_alpha']) > 1e-9:
                raise ValueError('Camera pose cannot be reproduced from recorded odometry/extrinsics')
            if abs(valid-meta['depth_valid_fraction']) > 1e-6 or valid == 0:
                raise ValueError('Invalid depth validity metadata')
            scale = cfg.depth_scale if depth.encoding == '16UC1' else 1.0
            if meta['depth_scale_m'] != scale or meta['depth_encoding'] != depth.encoding:
                raise ValueError('Incorrect depth units')
            count += 1
            first = ns if first is None else first
            previous = ns
            min_valid = min(min_valid, valid)
            max_delta = max(max_delta, abs(meta['depth_stamp_ns']-ns)/1e9)
            max_gap = max(max_gap, gap)
        if saved_report.get('frames') != count:
            raise ValueError('Final report count disagrees with actual frame groups')
        if count == 0:
            raise ValueError('No valid complete frames')
        return dict(status='PASS', frames=count, actual_rate_hz=(count-1)/((previous-first)/1e9) if count>1 else 0,
                    min_depth_valid_fraction=min_valid, max_rgbd_delta_s=max_delta, max_odom_gap_s=max_gap,
                    camera_pose_reproduced_from_raw_odometry=True,
                    hardware_sync_verified=False, accuracy_validated=False)


def rectify_maps(info):
    import cv2
    k = np.array(info.K).reshape(3, 3)
    d = np.array(info.D)
    if info.distortion_model in ('plumb_bob', 'rational_polynomial'):
        return cv2.initUndistortRectifyMap(k, d, np.eye(3), k, (info.width, info.height), cv2.CV_32FC1)
    if info.distortion_model == 'equidistant' and len(d) == 4:
        return cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), k, (info.width, info.height), cv2.CV_32FC1)
    raise ValueError('Unsupported distortion model for rectification: '+info.distortion_model)


def export(args):
    import cv2
    source = Path(args.bag).expanduser().resolve()
    out = Path(args.output).expanduser().resolve()
    if out.exists():
        raise FileExistsError('Export directory exists; refusing to overwrite')
    validation = check_bag(source)
    out.mkdir(parents=True)
    (out/'rgb').mkdir()
    (out/'depth').mkdir()
    maps, previous_calibration = None, None
    try:
        with rosbag.Bag(str(source)) as bag, (out/'frames.jsonl').open('w') as frames, (out/'poses_tum.txt').open('w') as poses:
            session = read_session(bag)
            poses.write('# timestamp tx ty tz qx qy qz qw; T_world_camera_optical, meters, xyzw\n')
            for ns, group in frame_groups(bag):
                rgb, dep, ci, di, pose, ext, annotation = [group[t] for t in GROUP]
                meta = json.loads(annotation.data)
                color = np.array(image_array(rgb), copy=True)
                depth = image_array(dep).astype(image_array(dep).dtype.newbyteorder('='), copy=True)
                if rgb.encoding == 'rgb8': color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
                elif rgb.encoding == 'rgba8': color = cv2.cvtColor(color, cv2.COLOR_RGBA2BGRA)
                original = calibration(ci)
                exported = copy.deepcopy(original)
                if args.rectify:
                    if original != previous_calibration:
                        maps = rectify_maps(ci)
                        previous_calibration = original
                    color = cv2.remap(color, *maps, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                    depth = cv2.remap(depth, *maps, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                    exported['D'] = [0.0]*5
                    exported['distortion_model'] = 'plumb_bob'
                    exported['R'] = list(np.eye(3).reshape(-1))
                    exported['P'] = list(np.column_stack((np.array(ci.K).reshape(3, 3), np.zeros(3))).reshape(-1))
                name = '%06d_%d' % (meta['frame_index'], ns)
                rgb_path = 'rgb/'+name+'.png'
                if not cv2.imwrite(str(out/rgb_path), color): raise IOError('PNG color write failed')
                if dep.encoding == '16UC1':
                    depth_path = 'depth/'+name+'.png'
                    if not cv2.imwrite(str(out/depth_path), depth): raise IOError('PNG depth write failed')
                else:
                    depth_path = 'depth/'+name+'.npy'
                    np.save(str(out/depth_path), depth, allow_pickle=False)
                p, q = pose.pose.position, pose.pose.orientation
                seconds = '%d.%09d' % divmod(ns, 1000000000)
                poses.write(seconds+' '+' '.join('%.12g'%v for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))+'\n')
                meta.update(rgb_path=rgb_path, depth_path=depth_path, rectified=args.rectify,
                            T_world_camera=pose_matrix(pose.pose).tolist(), T_body_camera=transform_matrix(ext).tolist(),
                            original_camera_info=original, exported_camera_info=exported)
                frames.write(json.dumps(meta, allow_nan=False)+'\n')
        (out/'dataset.json').write_text(json.dumps(dict(session=session, validation=validation, rectified=args.rectify,
                                                       source_bag=str(source), depth_resampling='nearest' if args.rectify else 'none'), indent=2)+'\n')
    except BaseException:
        (out/'INCOMPLETE').write_text('Export failed; do not use as a complete dataset.\n')
        raise
    print('Exported:', out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('record', help='Record already-running sensors and LIO; Ctrl-C to finish')
    p.add_argument('output')
    p.add_argument('--rate', type=float, default=6)
    p.add_argument('--rgbd-slop', type=float, default=.01)
    p.add_argument('--max-odom-gap', type=float, default=.25)
    p.add_argument('--wait', type=float, default=3)
    p.add_argument('--camera-time-offset', type=float, default=0)
    p.add_argument('--depth-scale', type=float, default=.001)
    p.add_argument('--world', default='lio_map')
    p.add_argument('--body', default='body')
    p.add_argument('--optical', default='camera_color_optical_frame')
    p.add_argument('--duration', type=float)
    p.add_argument('--startup-timeout', type=float, default=20)
    p.add_argument('--min-free-gb', type=float, default=2)
    p.add_argument('--compression', choices=['none', 'lz4', 'bz2'], default='lz4')
    p.add_argument('--raw-sensors', action='store_true')
    p = sub.add_parser('check', help='Offline bag validation; no ROS master needed')
    p.add_argument('bag')
    p = sub.add_parser('export', help='Export PNG/depth, per-frame calibration and TUM poses')
    p.add_argument('bag')
    p.add_argument('output')
    p.add_argument('--rectify', action='store_true')
    args = parser.parse_args()
    try:
        if args.command == 'record': record(args)
        elif args.command == 'check': print(json.dumps(check_bag(Path(args.bag).expanduser()), indent=2))
        else: export(args)
    except (Exception, KeyboardInterrupt) as e:
        print('ERROR:', e, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
