#!/usr/bin/env python3.7

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
import rospy
import numpy as np
import matplotlib.pyplot as plt
from pub_classes import act_class
from process_skel_data import process_skel_data
from tensorflow.keras.models import load_model
import tensorflow_addons as tfa
from global_data import PCA_COMPS
import csv

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
        self.load_imu_scale_parameters()
        self.new_skel_data = np.zeros((1, 15*7))
        self.skel_data = np.zeros((WIN_LEN, 15*7))
        self.skel_update_t = None

        self.screw_classifier = load_model('./sam_nodes/scripts/models_parameters/Screw In_model_1_2str_i(CCPCCP)s(CCPCCP)c(HHHD)_inclAllNull.h5')
        self.allen_classifier = load_model('./sam_nodes/scripts/models_parameters/Allen In_model_1_2str_i(CCPCCP)s(CCPCCP)c(HHHD)_inclAllNull.h5')
        self.hammer_classifier = load_model('./sam_nodes/scripts/models_parameters/Hammer_model_1_2str_i(CCPCCP)s(CCPCCP)c(HHHD)_inclAllNull.h5')
        self.hand_classifier = load_model('./sam_nodes/scripts/models_parameters/Hand Screw In_model_1_2str_i(CCPCCP)s(CCPCCP)c(HHHD)_inclAllNull.h5')

        self.screw_pred = 0
        self.allen_pred = 0
        self.hammer_pred = 0
        self.hand_pred = 0

        self.plt_pred = False
        self.act_obj = act_class(frame_id=self.frame_id, class_count=4, user_id=self.id, user_name=self.name, queue=10)

    def load_imu_scale_parameters(self):
        # load scaling parameters
        scale_file = "./sam_nodes/scripts/models_parameters/imu_scale_params_winlen3_transitionsTrue_1v1_inclAllNull.csv" # file with normalisation parameters
        with open(scale_file, newline='') as f:
            reader = csv.reader(f)
            data = np.array(list(reader))
            self.means = data[1:, 1].astype(float)
            self.scales = data[1:, -1].astype(float)

    def predict_actions(self):
        new_skel_data = process_skel_data(self.skel_data)
        predict_data = [self.imu_data[np.newaxis, ...], new_skel_data[np.newaxis, ...]]

        self.screw_pred = self.screw_classifier.predict(predict_data)[0][0]
        self.allen_pred = self.allen_classifier.predict(predict_data)[0][0]
        self.hammer_pred = self.hammer_classifier.predict(predict_data)[0][0]
        self.hand_pred = self.hand_classifier.predict(predict_data)[0][0]

        prediction = [self.screw_pred, self.allen_pred, self.hammer_pred, self.hand_pred]
        self.act_obj.publish(prediction)

        if self.plt_pred:
            self.plot_prediction(prediction)

    def add_imu_data(self, data, time):
        self.imu_update_t = time
        self.new_imu_data = (data-self.means)/self.scales

    def add_skel_data(self, data, time):
        self.skel_update_t = time
        self.new_skel_data = data

    def update_data_window(self):
        self.imu_data = np.vstack((self.imu_data, self.new_imu_data))
        self.imu_data = self.imu_data[-WIN_LEN:, :]
        self.skel_data = np.vstack((self.skel_data, self.new_skel_data))
        self.skel_data = self.skel_data[-WIN_LEN:, :]

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
