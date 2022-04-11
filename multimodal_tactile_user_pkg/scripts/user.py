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

        #if not self.test:
        self.perception = perception_module(self.name, self.id, self.frame_id, self.ACTION_CATEGORIES)
        self.task_reasoning = reasoning_module(self.name, self.id, self.frame_id, self.ACTION_CATEGORIES)

    def update_user_details(self, frame_id=None, name=None, Id=None, task=None):
        if name:
            self.name = name
            self.frame_id = f"{frame_id}_{self.name}"
            self.perception.update_user_details(name=name, frame_id=self.frame_id)
            self.task_reasoning.update_user_details(name=name, frame_id=self.frame_id)
        if Id:
            self.id = Id
            self.perception.update_user_details(Id=self.id)
            self.task_reasoning.update_user_details(Id=self.id)
        if task:
            self.update_task(task)

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
