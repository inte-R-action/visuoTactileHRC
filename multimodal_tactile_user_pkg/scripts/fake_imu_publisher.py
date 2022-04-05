#!/usr/bin/env python3.7

import sys, struct, serial, os
import numpy as np
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import rospy
from std_msgs.msg import Int8, Float64, String
from pub_classes import diag_class, act_class
import csv
from multimodal_tactile_custom_msgs.msg import diagnostics


os.chdir(os.path.expanduser("~/catkin_ws/src/multimodal_human_robot_collaboration/sam_nodes/scripts/"))


def sys_stat_callback(data):
    """callback for system status messages"""
    if data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')


def fakeIMUmain():
    print("-----Here we go-----")
    frame_id = 'fakeIMUpub_node'
    rospy.init_node(frame_id, anonymous=True)
    diag_obj = diag_class(frame_id=frame_id, user_id=1, user_name='j', queue=10)
    rospy.Subscriber('SystemStatus', diagnostics, sys_stat_callback)

    act_obj = act_class(frame_id=frame_id, class_count=4, user_id=1, user_name='j', queue=10)
    prediction = np.zeros(4)

    print("Starting main loop")
    
    diag_level = 1 # 0:ok, 1:warning, 2:error, 3:stale
    
    rate = rospy.Rate(2)  # Message publication rate, Hz => should be 2
    with open ('fake_imu_data.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader)
        for row in csvreader:
            if not rospy.is_shutdown():

                prediction = [float(i) for i in row[0:4]]
                prediction = np.reshape(prediction, (-1))

                try:
                    act_obj.publish(prediction.tolist())
                    print(prediction)
                    diag_msg = "fake_imu_pub all good"
                    diag_level = 0 # ok
                except Exception as e:
                    print(f"Error: {e}")
                    diag_msg = "fake_imu_pub not so good"
                    diag_level = 1 # warning

                diag_obj.publish(diag_level, diag_msg)
                #print(f"Action: {class_pred} Probs: {prediction}")
                rate.sleep()


if __name__ == "__main__":
    try:
        fakeIMUmain()
    except rospy.ROSInterruptException:
        print("Keyboard Interrupt")

    print("All done")
