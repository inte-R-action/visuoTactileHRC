#!/usr/bin/env python3.7

import numpy as np
import rospy
from std_msgs.msg import String, Int8, Float64
import time
from datetime import datetime, date
import matplotlib.pyplot as plt
from postgresql.database_funcs import database
import pandas as pd
import argparse
from global_data import ACTIONS_NULL
from user_perception_module import perception_module
from user_reasoning_module import reasoning_module


class User:
    def __init__(self, name, Id, frame_id, test):
        self.test = test
        self.status = None
        self.name = name
        self.id = Id
        self.frame_id = f"{frame_id}_{self.name}"
        self.task_data = None
        self.task = None
        self.ACTION_CATEGORIES = ACTIONS_NULL
        self.db = database()
        self.shimmer_ready = 1

        if not self.test:
            self.perception = perception_module(self.name, self.id, self.frame_id, self.ACTION_CATEGORIES)
        self.task_reasoning = reasoning_module(self.name, self.id, self.frame_id, self.ACTION_CATEGORIES)

    def update_task(self, task):
        self.task = task

        self.task_reasoning.imu_pred = np.zeros(6) #class confs, t
        self.task_reasoning.imu_pred_hist = np.empty(6) #class confs, t
        self.task_reasoning.imu_state_hist = np.array([0, 1, datetime.min, datetime.min]) #class, conf, tStart, tEnd
        self.task_reasoning.final_state_hist = np.array([0, 1, datetime.min, datetime.min], ndmin=2) #class, conf, tStart, tEnd

        if not self.test:
            self.perception.actions = self.ACTION_CATEGORIES
        self.task_reasoning.update_task(self.task)

        print(f"Updated task for user {self.name} to task {self.task}")

    # def collate_har_seq(self, task_started):
    #     # Group predictions of same type together
    #     for a, act_hist in enumerate(self._har_state_hist):
    #         prediction_prob = self._har_pred_hist[-1, a]
    #         null_prob = 1 - prediction_prob

    #         if null_prob > prediction_prob:
    #             action = 0
    #             prob = null_prob
    #         else:
    #             action = a+1
    #             prob = prediction_prob

    #         if action == act_hist[-1, 0]:
    #             self._har_state_hist[a] = np.vstack((act_hist, [action, prob, self._har_pred_hist[-1, -1]]))
    #             self._final_state_hist[a][-1, 1] = np.mean(act_hist[:, 1])  # update final confidence
    #             self._final_state_hist[a][-1, -1] = self._har_pred_hist[-1, -1]  # update finish time
    #         else:
    #             if task_started:
    #                 # Can publish new episode to sql if not null action
    #                 if int(self._final_state_hist[a][-1, 0]) != 0:
    #                     start_t = self._final_state_hist[a][-1, 2]
    #                     end_t = self._final_state_hist[a][-1, 3]
    #                     dur = end_t - start_t
    #                     action_name = self.ACTION_CATEGORIES[int(self._final_state_hist[a][-1, 0])]
    #                     self.db.insert_data_list("Episodes",
    #                     ["date", "start_t", "end_t", "duration", "user_id", "hand", "task_name", "action_name", "action_no"],
    #                     [(date.today(), start_t, end_t, dur, self.id, "R", self.task, action_name, 0)])

    #             # Update state history objects
    #             new_start_t = self._har_pred_hist[-1, -1]
    #             self._har_state_hist[a] = np.array((action, prob, new_start_t), ndmin=2)
    #             self._final_state_hist[a] = np.vstack((self._final_state_hist[a], [action, prob, new_start_t, new_start_t]))
