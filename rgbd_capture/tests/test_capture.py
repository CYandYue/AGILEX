import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from dataclasses import asdict

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rosbag
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage
from core import *
from capture import Writer, SESSION, REPORT, check_bag, frame_groups, export, rectify_maps


def image(t, depth=False, seq=0):
    m = Image()
    m.header.stamp, m.header.frame_id, m.header.seq = stamp(round(t*1e9)), 'camera_color_optical_frame', seq
    m.width, m.height = 12, 8
    m.encoding = '16UC1' if depth else 'rgb8'
    data = (np.arange(96, dtype=np.uint16).reshape(8, 12)+1000) if depth else np.arange(288, dtype=np.uint8).reshape(8,12,3)
    m.data = data.tobytes()
    m.step = m.width*(2 if depth else 3)
    return m


def info(t):
    m = CameraInfo()
    m.header.stamp, m.header.frame_id = stamp(round(t*1e9)), 'camera_color_optical_frame'
    m.width, m.height = 12, 8
    m.K = [8.,0,5.5,0,8.,3.5,0,0,1]
    m.D = [.1,-.01,0,0,0]
    m.R = list(np.eye(3).reshape(-1))
    m.P = [8.,0,5.5,0,0,8.,3.5,0,0,0,1,0]
    m.distortion_model = 'plumb_bob'
    return m


def odom(t):
    m = Odometry()
    m.header.stamp = stamp(round(t*1e9))
    m.header.frame_id, m.child_frame_id = 'lio_map', 'body'
    m.pose.pose.position.x = .2*(t-100)
    q = quaternion_from_euler(0, 0, .3*(t-100))
    m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
    return m


def extrinsic():
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = 'body', 'camera_color_optical_frame'
    m.transform.translation.x = .2
    m.transform.translation.z = .1
    q = quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
    return m


def dynamic(m):
    t = TransformStamped()
    t.header = copy.deepcopy(m.header)
    t.child_frame_id = m.child_frame_id
    t.transform.translation.x = m.pose.pose.position.x
    t.transform.rotation = copy.deepcopy(m.pose.pose.orientation)
    return TFMessage([t])


def make_bag(path, cfg=None, float_depth=False):
    cfg = cfg or Config(camera_time_offset=.015)
    with rosbag.Bag(str(path), 'w', compression='lz4') as bag:
        writer = Writer(bag)
        writer(SESSION, json_msg(dict(schema_version=1, config=asdict(cfg))), int(99e9))
        engine = Engine(writer, cfg)
        events = [(99.9, '/tf_static', TFMessage([extrinsic()]))]
        for i in range(61):
            t = 100+i/30
            depth = image(t+.005, True, i)
            if float_depth:
                values = image_array(depth).astype(np.float32)*.001
                values[0,0] = np.nan
                depth.encoding, depth.step, depth.data = '32FC1', depth.width*4, values.tobytes()
            # Small but nonzero depth timestamp delta; CI arrives after image.
            events.extend([(t+.03, RGB, image(t, seq=i)), (t+.032, DEPTH, depth),
                           (t+.034, RGB_INFO, info(t)), (t+.035, DEPTH_INFO, info(t+.005))])
        for i in range(24):
            t = 100+i/10
            m = odom(t)
            events.extend([(t+.22, ODOM, m), (t+.221, '/tf', dynamic(m))])
        for arrival, topic, msg in sorted(events, key=lambda e:e[0]):
            engine.ingest(topic, msg, round(arrival*1e9), arrival)
        engine.drain(106, final=True)
        report = engine.report()
        report.update(status='RECORDED', error=None)
        writer(REPORT, json_msg(report), int(106e9))
    return report


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='rgbd-capture-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_delayed_odom_and_analytic_camera_pose(self):
        bag = self.root/'good.bag'
        r = make_bag(bag)
        self.assertEqual(r['frames'], 13)
        self.assertAlmostEqual(r['actual_rate_hz'], 6)
        checked = check_bag(bag)
        self.assertEqual(checked['status'], 'PASS')
        with rosbag.Bag(str(bag)) as f:
            for ns, g in frame_groups(f):
                meta = json.loads(g[GROUP[-1]].data)
                t = ns/1e9+.015
                angle = .3*(t-100)
                p = g[GROUP[4]].pose.position
                self.assertAlmostEqual(p.x, .2*(t-100)+.2*math.cos(angle), places=7)
                self.assertAlmostEqual(p.y, .2*math.sin(angle), places=7)
                self.assertEqual(meta['depth_stamp_ns']-meta['rgb_stamp_ns'], 5000000)
                self.assertEqual(g[GROUP[0]].data, image(ns/1e9).data)

    def test_big_endian_and_padded_rows(self):
        m = image(100, True)
        m.width, m.height, m.step = 2, 2, 6
        m.is_bigendian = 1
        m.data = b'\x00\x01\x01\x00xx\x03\xe8\xff\xffyy'
        np.testing.assert_array_equal(image_array(m), [[1,256],[1000,65535]])
        m.data = m.data[:-1]
        with self.assertRaises(ValueError): image_array(m)
        m.encoding = 'made_up'
        with self.assertRaises(ValueError): image_array(m)

    def test_depth_size_and_pixel_model_must_match(self):
        a, b, ci, di = image(100), image(100, True), info(100), info(100)
        b.width = 6
        with self.assertRaises(ValueError): validate_rgbd(a,b,ci,di,Config().optical)
        b = image(100, True)
        di.D[0] += .01
        with self.assertRaises(ValueError): validate_rgbd(a,b,ci,di,Config().optical)

    def engine_ready(self, cfg=None):
        recorded = []
        e = Engine(lambda *args: recorded.append(args), cfg or Config())
        for topic, m in [('/tf_static',TFMessage([extrinsic()])),(RGB_INFO,info(100)),(DEPTH_INFO,info(100))]:
            e.ingest(topic,m,int(100e9),100)
        return e, recorded

    def test_gap_and_missing_pose_are_dropped_not_extrapolated(self):
        e, rows = self.engine_ready()
        e.ingest(ODOM,odom(100),int(100e9),100)
        e.ingest(ODOM,odom(101),int(101e9),101)
        e.ingest(RGB,image(100.5),int(101e9),101)
        e.ingest(DEPTH,image(100.5,True),int(101e9),101)
        e.drain(105,final=True)
        self.assertEqual(e.stats['frames'],0)
        self.assertEqual(e.stats['dropped:odometry_gap'],1)
        self.assertFalse(any(topic in GROUP for topic,_,_ in rows))

    def test_backward_clock_is_fatal_and_duplicate_not_reused(self):
        e, _ = self.engine_ready()
        e.ingest(ODOM,odom(100),int(100e9),100)
        e.ingest(ODOM,odom(100),int(100e9),100)
        self.assertEqual(len(e.odoms),1)
        with self.assertRaises(ValueError): e.ingest(ODOM,odom(99),int(100e9),100)

    def test_zero_depth_and_mismatched_timestamps_never_get_pose(self):
        e, _ = self.engine_ready()
        for t in (100,100.1): e.ingest(ODOM,odom(t),int(t*1e9),t)
        dep = image(100.05, True)
        dep.data = b'\x00'*len(dep.data)
        e.ingest(RGB,image(100.05),int(101e9),101)
        e.ingest(DEPTH,dep,int(101e9),101)
        self.assertEqual(e.stats['dropped:invalid_rgbd'],1)
        e.ingest(RGB,image(100.08),int(101e9),101)
        e.ingest(DEPTH,image(100.12,True),int(101e9),101)
        e.drain(105,final=True)
        self.assertEqual(e.stats['frames'],0)

    def test_export_rectifies_both_streams_and_updates_calibration(self):
        import cv2
        bag = self.root/'good.bag'
        make_bag(bag)
        out = self.root/'export'
        export(SimpleNamespace(bag=str(bag),output=str(out),rectify=True))
        rows = [json.loads(line) for line in (out/'frames.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows),13)
        self.assertEqual(rows[0]['exported_camera_info']['D'],[0]*5)
        self.assertNotEqual(rows[0]['original_camera_info']['D'],[0]*5)
        depth = cv2.imread(str(out/rows[0]['depth_path']),cv2.IMREAD_UNCHANGED)
        self.assertEqual(depth.dtype,np.uint16)
        self.assertTrue(set(depth.flat).issubset(set(range(1000,1096))|{0}))
        self.assertFalse((out/'INCOMPLETE').exists())
        with self.assertRaises(FileExistsError): export(SimpleNamespace(bag=str(bag),output=str(out),rectify=True))

    def test_validator_rejects_corrupted_pose_and_missing_group_member(self):
        good = self.root/'good.bag'
        make_bag(good)
        for mode in ('pose','missing'):
            bad = self.root/(mode+'.bag')
            with rosbag.Bag(str(good)) as src, rosbag.Bag(str(bad),'w') as dst:
                for topic,msg,t in src.read_messages():
                    if topic == GROUP[4] and msg.header.seq == 3:
                        if mode == 'missing': continue
                        msg.pose.position.x += .1
                    dst.write(topic,msg,t)
            with self.assertRaises(ValueError): check_bag(bad)

    def test_anymsg_raw_serialization_without_vendor_package(self):
        path = self.root/'raw.bag'
        m = Float32(3.5)
        buffer = io.BytesIO(); m.serialize(buffer)
        raw = SimpleNamespace(_buff=buffer.getvalue(),_connection_header={
            'type':m._type,'md5sum':m._md5sum,'message_definition':m._full_text,'callerid':'/test'})
        with rosbag.Bag(str(path),'w') as bag: Writer(bag)('/raw',raw,int(100e9))
        with rosbag.Bag(str(path)) as bag:
            self.assertAlmostEqual(next(bag.read_messages()).message.data,3.5)

    def test_raw_merge_preserves_connections_and_detects_missing_raw(self):
        from raw_recording import merge_bags
        good, core, raw, merged = (self.root/name for name in ('good.bag','core.bag','sensor.bag','merged.bag'))
        make_bag(good)
        with rosbag.Bag(str(good)) as src, rosbag.Bag(str(core),'w') as dst:
            for topic,msg,t in src.read_messages():
                if topic in (SESSION,REPORT):
                    data = json.loads(msg.data)
                    data.update({'raw_sensors':True} if topic == SESSION else {'raw_topic_counts':{'/raw':1}})
                    msg = json_msg(data)
                header = dict(latching='1',type=msg._type,md5sum=msg._md5sum,message_definition=msg._full_text) if topic == '/tf_static' else None
                dst.write(topic,msg,t,connection_header=header)
        with rosbag.Bag(str(raw),'w') as bag:
            bag.write('/raw',Float32(3.5),stamp(int(101e9)))
        with self.assertRaises(ValueError):check_bag(core)
        merge_bags(core,raw,merged)
        self.assertEqual(check_bag(merged)['status'],'PASS')
        with rosbag.Bag(str(merged)) as bag:
            messages = list(bag.read_messages(topics=['/raw','/tf_static'],return_connection_header=True))
            self.assertEqual(next(m for m in messages if m.topic == '/raw').message.data,3.5)
            self.assertTrue(all(m.connection_header['latching'] in ('1',b'1') for m in messages if m.topic == '/tf_static'))
        with self.assertRaises(FileExistsError):merge_bags(core,raw,merged)

    def test_float_depth_export_preserves_meters_and_nan(self):
        bag = self.root/'float.bag'
        make_bag(bag,float_depth=True)
        out = self.root/'float_export'
        export(SimpleNamespace(bag=str(bag),output=str(out),rectify=False))
        row = json.loads((out/'frames.jsonl').read_text().splitlines()[0])
        depth = np.load(out/row['depth_path'])
        self.assertEqual(row['depth_scale_m'],1.0)
        self.assertTrue(np.isnan(depth[0,0]))
        self.assertAlmostEqual(float(depth[0,1]),1.001,places=6)

    def test_late_static_tf_then_changed_extrinsic(self):
        e = Engine(lambda *args:None,Config())
        for topic,m in [(RGB_INFO,info(100)),(DEPTH_INFO,info(100)),(ODOM,odom(100)),(ODOM,odom(100.1)),
                        (RGB,image(100.05)),(DEPTH,image(100.05,True))]:
            e.ingest(topic,m,int(100.2e9),100.2)
        self.assertEqual(e.stats['frames'],0)
        e.ingest('/tf_static',TFMessage([extrinsic()]),int(100.3e9),100.3)
        self.assertEqual(e.stats['frames'],1)
        tr = extrinsic(); tr.transform.translation.x += .1
        e.ingest('/tf_static',TFMessage([tr]),int(100.4e9),100.4)
        e.ingest(ODOM,odom(100.2),int(100.4e9),100.4)
        e.ingest(ODOM,odom(100.3),int(100.4e9),100.4)
        e.ingest(RGB,image(100.25),int(100.4e9),100.4)
        with self.assertRaises(RuntimeError):
            e.ingest(DEPTH,image(100.25,True),int(100.4e9),100.4)

    def test_incomplete_report_cannot_pass_validation(self):
        good,bad = self.root/'good.bag',self.root/'partial.bag'
        make_bag(good)
        with rosbag.Bag(str(good)) as src, rosbag.Bag(str(bad),'w') as dst:
            for topic,msg,t in src.read_messages():
                if topic == REPORT:
                    data=json.loads(msg.data)
                    data.update(status='INCOMPLETE',error='disk failure')
                    msg=json_msg(data)
                dst.write(topic,msg,t)
        with self.assertRaises(ValueError):check_bag(bad)

    def test_fisheye_and_unknown_distortion(self):
        ci = info(100)
        ci.distortion_model, ci.D = 'equidistant',[.01,0,0,0]
        mx,my = rectify_maps(ci)
        self.assertEqual(mx.shape,(8,12))
        ci.distortion_model = 'unknown'
        with self.assertRaises(ValueError): rectify_maps(ci)


if __name__ == '__main__': unittest.main()
