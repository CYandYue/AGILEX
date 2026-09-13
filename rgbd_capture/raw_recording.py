"""Keep high-rate raw topics out of the Python RGB-D/pose event loop."""
import heapq
from pathlib import Path
import signal
import subprocess

import rosbag


class RawRecording:
    def __init__(self, base, topics, compression='lz4'):
        self.path = Path(str(base)+'.raw.bag')
        self.log_path = Path(str(base)+'.raw.log')
        self.topics = list(topics)
        if any(p.exists() for p in (self.path,Path(str(self.path)+'.active'),self.log_path)):
            raise FileExistsError('Raw recovery files already exist; use a new output name')
        self.log = self.log_path.open('x')
        cmd = ['rosbag','record','--buffsize','128','--chunksize','4096','--output-name',str(self.path)]
        if compression != 'none': cmd.append('--'+compression)
        cmd += self.topics
        try:
            self.process = subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:
            self.log.close()
            raise

    def check_running(self):
        if self.process.poll() is not None:
            raise RuntimeError('Raw rosbag recorder exited unexpectedly; see '+str(self.log_path))

    def stop(self):
        if self.process.poll() is None:
            import os
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                self.process.wait(timeout=5)
                raise RuntimeError('Raw recorder did not close cleanly')
        self.log.close()
        text = self.log_path.read_text(errors='replace').lower()
        if self.process.returncode not in (0, -signal.SIGINT) or 'buffer exceeded' in text or 'error writing' in text:
            raise RuntimeError('Raw recorder reported data loss/error; see '+str(self.log_path))
        if not self.path.exists() or Path(str(self.path)+'.active').exists():
            raise RuntimeError('Raw recording is not finalized')
        with rosbag.Bag(str(self.path)) as bag:
            result = {topic:bag.get_message_count(topic) for topic in self.topics}
        # Both lidars must have produced data; the selected IMU is checked by caller.
        for topic in self.topics[:2]:
            if result[topic] == 0:raise RuntimeError('Raw recording missing '+topic)
        return result


def merge_bags(core_path, raw_path, output, compression='lz4'):
    """Preserve bytes, record times and connection definitions, including latch."""
    output = Path(output)
    if output.exists():raise FileExistsError(output)
    with output.open('xb'):pass
    with rosbag.Bag(str(core_path)) as core, rosbag.Bag(str(raw_path)) as raw, rosbag.Bag(str(output),'w',compression=compression,chunk_threshold=4*1024*1024) as dest:
        streams = [bag.read_messages(raw=True,return_connection_header=True) for bag in (core,raw)]
        for item in heapq.merge(*streams,key=lambda m:m.timestamp.to_nsec()):
            dest.write(item.topic,item.message,item.timestamp,raw=True,connection_header=item.connection_header)
