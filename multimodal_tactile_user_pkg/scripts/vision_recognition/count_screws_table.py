#!/usr/bin/env python3

import rospy
from multimodal_tactile_custom_msgs.msg import object_state, screw_count
from std_msgs.msg import Int8
from statistics import mean
from decimal import Decimal, ROUND_HALF_UP
import time
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 
from pub_classes import screw_count_class

class screw_counter():
    def __init__(self, frame_id, u_id, u_name, type='raw_count'):
        if type=='raw_count':
            rospy.Subscriber('RawScrewCount', Int8, self.raw_count_callback)
        elif type == 'obj_msg_count':
            rospy.Subscriber('ObjectStates', object_state, self.obj_msg_count_callback)
        
        self.screw_pub_obj = screw_count_class(frame_id=frame_id, user_id=u_id, user_name=u_name)  
        self.seq = None
        self.screws = []
        self.screw_totals = []
        self.screw_ave = 0
        self.screw_ave_last = 0
        self.discard = False

    def obj_msg_count_callback(self, data):
        # Use this when counting screws from ros messages         
        if data.Header.seq == self.seq:
            if data.Object.Info[0] == 'screw':
                self.screws.append([data.Pose.position.x, data.Pose.position.y])
            if data.Object.Info[0] == 'hand':
                self.discard = True
        else:
            if not self.discard:
                self.count_screws()
            self.discard = False
            self.seq = data.Header.seq
            self.screws = []
            if data.Object.Info[0] == 'screw':
                self.screws.append([data.Pose.position.x, data.Pose.position.y])

    def raw_count_callback(self, msg):
        # Use this when counting screws direct
        self.screws = range(msg.data)
        self.count_screws()

    def count_screws(self):
        self.screw_totals.append([time.time(), len(self.screws)])
        self.screw_totals = [count for count in self.screw_totals if time.time()-count[0] < 3]
        self.screw_ave = int(Decimal(mean([b for b in zip(*self.screw_totals)][1])).to_integral_value(rounding=ROUND_HALF_UP))
        #print(f"Current number screws: {self.screw_ave}, last: {self.screw_ave_last}")
        self.screw_pub_obj.publish(self.screw_ave, self.screw_ave_last)

    def next_screw(self):  
        print("next screw") 
        self.screw_ave_last = self.screw_ave  
        self.screw_totals = []
        self.screw_ave = 0
        

def run():
    # ROS node setup
    frame_id = 'screw_counter'
    rospy.init_node(frame_id, anonymous=True)
    

    counter = screw_counter(frame_id, 1, 'unknown', type = 'raw_count')    

    rate = rospy.Rate(0.1) # 1hz
    while not rospy.is_shutdown():
        counter.next_screw()
        rate.sleep()

if __name__ == '__main__':
    run()
    
