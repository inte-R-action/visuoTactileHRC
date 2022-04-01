#!/usr/bin/env python3.7

import sys, struct, serial, os
import numpy as np
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import rospy
from std_msgs.msg import Int8, Float64, String
from pub_classes import diag_class, act_class, threeIMUs_class
import csv
from sam_custom_messages.msg import diagnostics, skeleton
from global_data import SKELETON_FRAMES, ALL_ACTIONS
from geometry_msgs.msg import Pose
from statistics import mean, stdev

os.chdir(os.path.expanduser("~/catkin_ws/src/multimodal_human_robot_collaboration/sam_nodes/scripts/"))
start_trial = False


def sys_stat_callback(data):
    """callback for system status messages"""
    if data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')


def sys_cmd_callback(msg):
    """callback for system command messages"""
    global start_trial
    if msg.data == 'start':
        start_trial = True


def fakeSensorsmain():
    print("-----Here we go-----")
    frame_id = 'fakeSensorspub_node'
    rospy.init_node(frame_id, anonymous=True)
    diag_obj = diag_class(frame_id=frame_id, user_id=1, user_name='j', queue=10)
    skel_diag_obj = diag_class(frame_id='skeleton_viewer', user_id=1, user_name='j', queue=10)
    rospy.Subscriber('SystemStatus', diagnostics, sys_stat_callback)
    rospy.Subscriber("ProcessCommands", String, sys_cmd_callback)

    imu_obj = threeIMUs_class(frame_id=frame_id, user_id=1, user_name='j', queue=10)
    imu_data = [0]*18
    joints_publisher = rospy.Publisher('SkeletonJoints', skeleton, queue_size = 1)

    joints_msg = skeleton()
    joints_msg.Header.stamp = rospy.get_rostime()
    joints_msg.Header.seq = 0
    joints_msg.Header.frame_id = frame_id
    joints_msg.UserId = 1
    joints_msg.UserName = 'j'

    diag_level = 1 # 0:ok, 1:warning, 2:error, 3:stale

    rate = rospy.Rate(50)  # Message publication rate, Hz => should be 50
    prev = None
    prev_t = time.time()
    with open ('imu_skeleton_data_user_j_take_3.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=';')
        next(csvreader)

        while (not start_trial) and (not rospy.is_shutdown()):
            diag_obj.publish(0, "Waiting for trial to start")
            skel_diag_obj.publish(0, "Waiting for trial to start")
            time.sleep(0.1)

        print("Fake sensor publisher starting")
        for row in csvreader:
            if not rospy.is_shutdown():

                try:
                    if (row[1] != prev) or (time.time()-prev_t > 5):
                        print(ALL_ACTIONS[int(float(row[1]))])
                        prev = row[1]
                        prev_t = time.time()
                    imu_obj.publish(row[2:20])

                    for i in range(len(SKELETON_FRAMES)):
                        pose_msg = Pose()
                        pose_msg.position.x = float(row[20+(7*i)])
                        pose_msg.position.y = float(row[21+(7*i)])
                        pose_msg.position.z = float(row[22+(7*i)])
                        pose_msg.orientation.x = float(row[23+(7*i)])
                        pose_msg.orientation.y = float(row[24+(7*i)])
                        pose_msg.orientation.z = float(row[25+(7*i)])
                        pose_msg.orientation.w = float(row[26+(7*i)])

                        setattr(joints_msg, SKELETON_FRAMES[i], pose_msg)
                    joints_msg.Header.seq += 1
                    joints_msg.Header.stamp = rospy.get_rostime()
                    joints_publisher.publish(joints_msg)

                    diag_msg = "fake_sensor_pub all good"
                    diag_level = 0 # ok

                except Exception as e:
                    print(f"Error: {e}")
                    diag_msg = "fake_sensor_pub not so good"
                    diag_level = 1 # warning

                try:
                    diag_obj.publish(diag_level, diag_msg)
                    skel_diag_obj.publish(diag_level, diag_msg)
                except rospy.exceptions.ROSException:
                    print("fake sensor pub diag pub error")

            else:
                raise rospy.ROSException

            rate.sleep()


if __name__ == "__main__":
    try:
        fakeSensorsmain()
    except rospy.ROSInterruptException:
        print("Keyboard Interrupt")
    except rospy.ROSException:
        pass
    finally:
        print("All done")
