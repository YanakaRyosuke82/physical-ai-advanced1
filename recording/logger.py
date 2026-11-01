"""CSV writers preserve None explicitly; rosbag keeps original ROS messages."""
import csv
import json
import signal
import subprocess
import threading
import time


class Logs:
    def __init__(self,folder):
        self.folder=folder;self.lock=threading.RLock();self.handles={};self.writers={};self.bag=None;self.bag_log=None
        self.phase='initializing';self.run_id=folder.name;self.clock=lambda:None
    def row(self,name,data):
        with self.lock:
            data={'run_id':self.run_id,'phase':self.phase,'sim_s':self.clock(),'wall_time_ns':time.time_ns(),**data}
            if name not in self.writers:
                h=(self.folder/(name+'.csv')).open('w',newline='',encoding='utf-8');self.handles[name]=h
                self.writers[name]=csv.DictWriter(h,fieldnames=list(data));self.writers[name].writeheader()
            clean={k:('None' if v is None else json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list,tuple)) else v) for k,v in data.items()}
            self.writers[name].writerow(clean);self.handles[name].flush()
    def event(self,event,data=None):self.row('events',{'event':event,'data':data})
    def start_bag(self,topics):
        self.bag_log=(self.folder/'rosbag_console.txt').open('w')
        self.bag=subprocess.Popen(['ros2','bag','record','-o',str(self.folder/'rosbag'),*topics],stdin=subprocess.DEVNULL,stdout=self.bag_log,stderr=subprocess.STDOUT,start_new_session=True)
        threading.Event().wait(.8)
        if self.bag.poll() is not None:raise RuntimeError('rosbag2 failed to start; see rosbag_console.txt')
        self.event('rosbag_started',{'topics':topics})
    def close(self):
        if self.bag and self.bag.poll() is None:
            import os
            os.killpg(self.bag.pid,signal.SIGINT)
            try:self.bag.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.bag.pid,signal.SIGTERM)
                try:self.bag.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(self.bag.pid,signal.SIGKILL);self.bag.wait()
                self.event('rosbag_forced_stop',{'metadata_may_be_incomplete':True})
        if self.bag_log:self.bag_log.close()
        for h in self.handles.values():h.close()
