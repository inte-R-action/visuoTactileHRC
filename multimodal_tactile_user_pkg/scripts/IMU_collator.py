#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8, Float64
import cv2
import numpy as np
import time, datetime
import argparse
import matplotlib.pyplot as plt
from postgresql.database_funcs import database

CATEGORIES = ['AllenKeyIn', 'AllenKeyOut', 'ScrewingIn', 'ScrewingOut', 'Null']
pos = np.arange(len(CATEGORIES))

# parser = argparse.ArgumentParser(
#         description='Fusion of screw/bolt state estimation from IMU and vision sources')
#
# parser.add_argument('--disp', '-V',
#                     help='Enable graph of predictions over time',
#                     default=None,
#                     action="store_true")
#
# args = parser.parse_args()
#
# if args.disp:
#     plt.ion()
#     fig, axs = plt.subplots(2, 2)
#
#

#
#
# def plot_estimation():
#     global state_est
#     global CATEGORIES
#     legend = ['Image', 'IMU', 'Final']
#     Titles = ['Total No. Screws', 'Total No. Bolts']
#     global axs
#     for i in range(0, 2):
#         ax = axs[0, i]
#         ax.cla()
#         ax.plot(state_est._im[:, i])
#         ax.plot(state_est._imu[:, i])
#         ax.plot(state_est._final[:, i])
#         ax.set_ylabel('Estimated Number')
#         ax.set_title(Titles[i])
#         ax.legend(legend)
#         ax.set_ylim(bottom=0)
#
#     ax = axs[1, 0]
#     ax.cla()
#     ax.bar(pos, imu_pred, align='center', alpha=0.5)
#     ax.set_xticks(pos)
#     ax.set_xticklabels(CATEGORIES)
#     ax.set_ylabel('Confidence')
#     ax.set_ylim([0, 1])
#     ax.set_title('Current IMU Prediction')
#
#     ax = axs[1, 1]
#     ax.cla()
#     ax.bar([1, 2], [im_screw_hist[-1, 1], im_screw_hist[-1, 2]], align='center', alpha=0.5)
#     ax.set_xticks([1, 2])
#     ax.set_xticklabels(['Screws', 'Bolts'])
#     ax.set_ylabel('Confidence')
#     ax.set_ylim([0, 4])
#     ax.set_title('Current image Prediction')
#
#     plt.pause(0.0001)
#
#
#
# state_est = StateEst()

def collate_imu_seq(imu_state_hist, imu_pred_hist):
    # 'Dilation' filter to remove single erroneous predictions
    if np.shape(imu_state_hist)[0] >= 4:
        if (imu_state_hist[-1, 0] == imu_state_hist[-3, 0]):# & (imu_state_hist[-2, 1] <= 0.00000000001): # WHAT IS THIS DOING???
            imu_state_hist[-2, 0] = imu_state_hist[-1, 0] #NEED TO CHECK CONFIDENCES HERE

        # Group predictions of same type together
        if imu_state_hist[-2, 0] == imu_state_hist[-3, 0]:
            i = np.where(imu_pred_hist[:, -1]==imu_state_hist[-3, 2])[0][0] #Get index where action starts
            imu_state_hist[-2, 1] = np.mean(imu_pred_hist[i:-1, int(imu_state_hist[-2, 0])].astype(float)) #Not convinced about this mean
            imu_state_hist[-2, 2] = imu_state_hist[-3, 2] # set start time
            imu_state_hist = np.delete(imu_state_hist, -3, 0)
        else:
            # Can publish new episode to sql
            db = database()
            date = datetime.date.today()
            start_t = imu_state_hist[-3, 2]
            end_t = imu_state_hist[-3, 3]
            dur = end_t - start_t #datetime.datetime.combine(datetime.date.min, end_t) - datetime.datetime.combine(datetime.date.min, start_t)
            #table:str(table name), columns:[str(column name),], data:[('data1', data2, ),]
            db.insert_data_list("Episodes", 
            ["date", "start_t", "end_t", "duration", "user_id", "hand", "capability", "task_id"], 
            [(date, start_t, end_t, dur, 0, "R", CATEGORIES[int(imu_state_hist[-3, 0])], 0)])

        #     imu_pred_hist = np.array(imu_pred_hist[-1, :])
        #     imu_pred_hist = np.reshape(imu_pred_hist, (1, 5))

    return imu_state_hist, imu_pred_hist