#!/usr/bin/env python3
"""Private ROS master + synthetic 640x480 RGB-D; no hardware drivers or motion."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_capture import image, info, odom, dynamic, extrinsic
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture import check_bag, RGB, DEPTH, RGB_INFO, DEPTH_INFO, ODOM, RAW_TOPICS


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        port = sock.getsockname()[1]
    use_raw = '--raw-sensors' in sys.argv
    output = Path(tempfile.mkdtemp(prefix='rgbd-capture-live-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port, ROS_IP='127.0.0.1', ROS_LOG_DIR=str(output/'ros'))
    os.environ.pop('ROS_HOSTNAME',None)
    processes, handles = [], []
    def start(cmd, name):
        f = (output/name).open('w'); handles.append(f)
        p = subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p)
        return p
    try:
        start(['roscore','-p',str(port)],'master.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(150):
            try:
                if master.getPid('/capture_test')[0] == 1: break
            except OSError: pass
            time.sleep(.1)
        else: raise RuntimeError('Master did not start')
        import rospy
        import numpy as np
        from sensor_msgs.msg import Image, CameraInfo
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        rospy.init_node('capture_fixture',disable_signals=True)
        pubs = {t:rospy.Publisher(t,cls,queue_size=30,latch=t in ('/tf_static',RGB_INFO,DEPTH_INFO)) for t,cls in (
            (RGB,Image),(DEPTH,Image),(RGB_INFO,CameraInfo),(DEPTH_INFO,CameraInfo),(ODOM,Odometry),('/tf',TFMessage),('/tf_static',TFMessage))}
        raw_pubs = {topic:rospy.Publisher(topic, String if i<2 else Imu,queue_size=50) for i,topic in enumerate(RAW_TOPICS)} if use_raw else {}
        ci = info(0)
        ci.width, ci.height = 640,480
        ci.K = [450.,0,319.5,0,450.,239.5,0,0,1]
        ci.P = [450.,0,319.5,0,0,450.,239.5,0,0,0,1,0]
        # Publish once BEFORE recording: collector must recover latched TF/info.
        pubs['/tf_static'].publish(TFMessage([extrinsic()]))
        pubs[RGB_INFO].publish(ci); pubs[DEPTH_INFO].publish(ci)
        bag = output/'scene.bag'
        capture = start([str(Path(__file__).resolve().parents[1]/'run.sh'),
                         'record',str(bag),'--min-free-gb','0','--startup-timeout','15']+(['--raw-sensors'] if use_raw else []), 'capture.log')
        for _ in range(100):
            if pubs[RGB].get_num_connections() and pubs[DEPTH].get_num_connections(): break
            if capture.poll() is not None: raise RuntimeError('Capture exited before input; see '+str(output))
            time.sleep(.1)
        else: raise RuntimeError('Capture did not subscribe')
        depth_bytes = (np.arange(640*480,dtype=np.uint16).reshape(480,640)%4000+500).tobytes()
        color_bytes = np.tile(np.arange(640,dtype=np.uint8)[None,:,None],(480,1,3)).tobytes()
        raw_stop = __import__('threading').Event()
        if use_raw:
            def publish_raw_imus():
                while not raw_stop.wait(.005):
                    m = Imu(); m.header.stamp=rospy.Time.now();m.orientation.w=1;m.linear_acceleration.z=9.8
                    for topic in RAW_TOPICS[2:]:raw_pubs[topic].publish(m)
            __import__('threading').Thread(target=publish_raw_imus,daemon=True).start()
        started = time.monotonic()
        for i in range(330):
            t = rospy.Time.now().to_sec()
            a,b = image(t,seq=i),image(t,True,seq=i)
            a.width=b.width=640; a.height=b.height=480
            a.step,b.step=640*3,640*2
            a.data,b.data=color_bytes,depth_bytes
            pubs[RGB].publish(a); pubs[DEPTH].publish(b)
            if i%3 == 0:
                if use_raw:
                    for topic in RAW_TOPICS[:2]:raw_pubs[topic].publish(String('x'*400000))
                m=odom(t)
                # Keep synthetic values numerically small for epoch timestamps.
                m.pose.pose.position.x=.2*(time.monotonic()-started)
                m.pose.pose.orientation.x=m.pose.pose.orientation.y=m.pose.pose.orientation.z=0
                m.pose.pose.orientation.w=1
                pubs[ODOM].publish(m); pubs['/tf'].publish(dynamic(m))
            if i==180: os.killpg(capture.pid,signal.SIGINT)
            if capture.poll() is not None: break
            time.sleep(max(0,started+(i+1)/30-time.monotonic()))
        raw_stop.set()
        capture.wait(timeout=30)
        if capture.returncode != 0: raise RuntimeError('Capture failed; see '+str(output/'capture.log'))
        result=check_bag(bag)
        assert result['frames']>=25,result
        assert 5 < result['actual_rate_hz'] < 6.5,result
        assert not Path(str(bag)+'.active').exists()
        result['raw_sensors']=use_raw
        result['bag_bytes']=bag.stat().st_size
        result['logs']=str(output)
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
        for f in handles:f.close()
        print('Test output:',output)


if __name__=='__main__':main()
