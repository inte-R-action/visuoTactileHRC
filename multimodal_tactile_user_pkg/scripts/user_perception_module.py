#!/usr/bin/env python3.7

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
import rospy
import numpy as np
import matplotlib.pyplot as plt
from pub_classes import act_class
from tensorflow.keras.models import load_model
import tensorflow_addons as tfa
import csv
from global_data import GESTURES
from std_msgs.msg import String
import tensorflow as tf

physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
   tf.config.experimental.set_memory_growth(physical_devices[0], True)

plt.ion()

Fs = 50  # Sampling frequency, Hz
WIN_TIME = 3  # Window length, s
WIN_LEN = round(WIN_TIME * Fs)  # Window length, samples


class perception_module:
    def __init__(self, name, u_id, frame_id, ACTION_CATEGORIES):
        self.name = name
        self.id = u_id
        self.frame_id = frame_id
        self.actions = ACTION_CATEGORIES
        self.new_imu_data = np.zeros((1, 18))
        self.imu_data = np.zeros((WIN_LEN, 18))
        self.imu_update_t = None
        self.imu_means = None
        self.imu_scales = None

        folder = './multimodal_tactile_user_pkg/scripts/models_parameters/'
        self.gesture_classifier = load_model(folder+'gestures_classifier_TrainOnAll_1_CCPCCPHD_inclAllNull20220330-111512.h5')
        self.screw_classifier = load_model(folder+'Screw In_classifier_TrainOnAll_allclassesincl_20220408-154326.h5')
        self.allen_classifier = load_model(folder+'Allen In_classifier_TrainOnAll_allclassesincl_20220408-154310.h5')
        self.hammer_classifier = load_model(folder+'Hammer_classifier_TrainOnAll_allclassesincl_20220408-154602.h5')
        self.hand_classifier = load_model(folder+'Hand Screw In_classifier_TrainOnAll_allclassesincl_20220408-155333.h5')
        self.screw_pred = 0
        self.allen_pred = 0
        self.hammer_pred = 0
        self.hand_pred = 0
        self.load_imu_scale_parameters(folder)

        self.plt_pred = False
        self.act_obj = act_class(frame_id=self.frame_id+'_actions', class_count=4, user_id=self.id, user_name=self.name, queue=10)
        self.ges_obj = act_class(frame_id=self.frame_id+'_gestures', class_count=len(GESTURES), user_id=self.id, user_name=self.name, queue=10)

    def update_user_details(self, name=None, Id=None, frame_id=None):
        if name:
            self.name = name
            self.act_obj.act_msg.UserName = self.name
            self.ges_obj.act_msg.UserName = self.name
        if Id:
            self.id = Id
            self.act_obj.act_msg.UserId = self.id
            self.ges_obj.act_msg.UserId = self.id
        if frame_id:
            self.frame_id = frame_id
            self.act_obj.act_msg.Header.frame_id = self.frame_id+'_actions'
            self.ges_obj.act_msg.Header.frame_id = self.frame_id+'_gestures'

    def load_imu_scale_parameters(self, folder):
        # load scaling parameters
        scale_file = folder + "imu_scale_params_winlen3_transitionsTrue_1v1_gesturesTrue_inclAllNull.csv" # file with normalisation parameters
        with open(scale_file, newline='') as f:
            reader = csv.reader(f)
            data = np.array(list(reader))
            self.means = data[1:, 1].astype(float)
            self.scales = data[1:, -1].astype(float)

    def predict_actions(self):
        predict_data = self.imu_data[np.newaxis, ...]

        self.screw_pred = self.screw_classifier.predict(predict_data)[0][0]
        self.allen_pred = self.allen_classifier.predict(predict_data)[0][0]
        self.hammer_pred = self.hammer_classifier.predict(predict_data)[0][0]
        self.hand_pred = self.hand_classifier.predict(predict_data)[0][0]

        prediction = [self.screw_pred, self.allen_pred, self.hammer_pred, self.hand_pred]
        self.act_obj.publish(prediction)

        if self.plt_pred:
            self.plot_prediction(prediction)

    def predict_gestures(self):
        predict_data = self.imu_data[np.newaxis, ...]

        self.gesture_pred = self.gesture_classifier.predict(predict_data)[0]
        self.ges_obj.publish(self.gesture_pred)

    def add_imu_data(self, data, time):
        self.imu_update_t = time
        self.new_imu_data = (data-self.means)/self.scales

    def update_data_window(self):
        self.imu_data = np.vstack((self.imu_data, self.new_imu_data))
        self.imu_data = self.imu_data[-WIN_LEN:, :]

    def plot_prediction(self, prediction):
        pos = np.arange(len(self.actions)-1)
        plt.figure("HAR prediction")
        ax = plt.gca()
        ax.cla()
        ax.bar(pos, prediction, align='center', alpha=0.5)
        plt.xticks(pos, self.actions[1:])
        plt.ylabel('Confidence')
        ax.set_ylim([0, 1])
        plt.pause(0.0001)
