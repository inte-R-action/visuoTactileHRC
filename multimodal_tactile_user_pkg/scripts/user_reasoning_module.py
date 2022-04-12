#!/usr/bin/env python3.7

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from postgresql.database_funcs import database
from datetime import datetime, date, timedelta
from global_data import ACTIONS, GESTURES
import rospy
from std_msgs.msg import String

plt.ion()


class reasoning_module:
    def __init__(self, name, Id, frame_id, ACTION_CATEGORIES):
        self.name = name
        self.id = Id
        self.frame_id = frame_id
        self.task_data = None
        self.task = None
        self.actions_id_list = []
        self.ACTION_CATEGORIES = ACTION_CATEGORIES
        self.means = None
        self.scales = None
        self.human_row_idxs = []
        self.output_override = []
        self.fut_act_pred_col_names = None

        self.imu_pred = np.zeros(6) # class confs, t
        self.imu_pred_hist = np.empty(6) # class confs, t
        self.imu_state_hist = np.array([0, 1, datetime.min, datetime.min]) # class, conf, tStart, tEnd
        self.final_state_hist = np.array([0, 1, datetime.min, datetime.min], ndmin=2) # class, conf, tStart, tEnd

        self.curr_action_no = 0
        self.curr_action_type = None
        self.curr_action_start_t = datetime.now()
        self.task_started = False
        self.user_state = "initialising"

        self.current_gesture = np.array([0, datetime.min, datetime.min]) # class, tStart, tEnd
        self.cmd_publisher = rospy.Publisher('ProcessCommands', String, queue_size=10)
        self.usr_fdbck_pub = rospy.Publisher('UserFeedback', String, queue_size=10)
        self.db = database()

    def update_user_details(self, name=None, Id=None, frame_id=None):
        if name:
            self.name = name
        if Id:
            self.id = Id
        if frame_id:
            self.frame_id = frame_id

    def predict_action_statuses(self):
        time = 0
        for i, row in self.task_data.iterrows():
            # Check if override button has been used on action
            if not self.output_override[i]:
                if i < self.curr_action_no:
                    time = 0
                else:
                    if i == self.curr_action_no:
                        t_left_action = min(max(datetime.now()-self.curr_action_start_t, timedelta()), row['default_time'])
                    else:
                        t_left_action = row['default_time']
                    
                    if i > 0:
                        time = self.task_data.loc[i-1, 'time_left'] + t_left_action.total_seconds()
                    else:
                        time = t_left_action.total_seconds()
            else:
                time = 0

            self.task_data.loc[i, 'time_left'] = time

        self.publish_to_database()

    def publish_to_database(self):
        # Update user current action in sql table
        time = datetime.utcnow()
        separator = ', '

        # Delete old rows for user
        sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = {self.id};"""
        self.db.gen_cmd(sql_cmd)

        # Insert new rows for user for each action prediciton status
        sql_cmd = f"""INSERT INTO future_action_predictions ({separator.join(self.fut_act_pred_col_names)})
        VALUES """
        for i in range(len(self.human_row_idxs)):
            if i != 0:
                sql_cmd += ", "
            sql_cmd += f"""({self.id}, '{self.name}', '{time}', '{self.task}',
            {int(self.task_data.loc[self.human_row_idxs[i], 'action_no'])},
            '{self.task_data.loc[self.human_row_idxs[i], 'started']}',
            '{self.task_data.loc[self.human_row_idxs[i], 'done']}',
            '{self.task_data.loc[self.human_row_idxs[i], 'time_left']}')"""
        sql_cmd += ";"
        self.db.gen_cmd(sql_cmd)

    def update_task(self, task):
        self.task = task

        self.imu_pred_hist = np.empty(6) #class confs, t
        self.imu_state_hist = np.array([0, 1, datetime.min, datetime.min]) #class, conf, tStart, tEnd
        self.final_state_hist = np.array([0, 1, datetime.min, datetime.min], ndmin=2) #class, conf, tStart, tEnd

        col_names, actions_list = self.db.query_table(self.task, 'all')
        self.task_data = pd.DataFrame(actions_list, columns=col_names)
        self.task_data["started"] = 0
        self.task_data["done"] = 0
        self.task_data["time_left"] = datetime.now().time()
        self.curr_action_no = 0
        self.curr_action_type = self.task_data.iloc[0]["action_name"]
        self.curr_action_start_t = datetime.now()
        self.human_row_idxs = self.task_data.index[self.task_data['user_type'] == 'human'].tolist()
        self.output_override = [0]*len(self.task_data)
        
        self.fut_act_pred_col_names, _ = self.db.query_table('future_action_predictions',rows=0)

        sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = {self.id};"""
        self.db.gen_cmd(sql_cmd)

    def next_action_override(self):
        # Get last updated robot actions
        col_names, robot_actions = self.db.query_table('robot_action_timings', 'all')
        last_robot_stats = pd.DataFrame(robot_actions, columns=col_names)

        # Get last action not completed by robot
        try:
            last_robot_action_no = last_robot_stats.loc[last_robot_stats["user_id"]==self.id]["last_completed_action_no"].values[0]
        except (KeyError, IndexError) as e:
            # user hasn't been entered into table yet
            last_robot_action_no = -1

        # get list of actions after last completed robot action
        if last_robot_action_no != -1:
            tasks_left = self.task_data.drop(self.task_data.index[:self.task_data.loc[self.task_data['action_no']==last_robot_action_no].first_valid_index()+1])
        else:
            tasks_left = self.task_data

        next_robot_action_idx = int(tasks_left[tasks_left['user_type']=='robot'].first_valid_index())
        print(next_robot_action_idx)
        i = 0
        for r in range(next_robot_action_idx):
            print(self.task_data.loc[r])
            if self.task_data.loc[r]['user_type'] == 'human':
                self.output_override[i] = 1
                self.task_data.iloc[r, self.task_data.columns.get_loc("started")] = 1
                self.task_data.iloc[r, self.task_data.columns.get_loc("done")] = 1
                i += 1

        next_action_row_i = self.task_data[self.task_data.done == False].index[0]
        self.curr_action_no = self.task_data.iloc[next_action_row_i]["action_no"]
        self.curr_action_type = self.task_data.iloc[next_action_row_i]["action_name"]
        if self.task_started:
            self.usr_fdbck_pub.publish(f"Waiting for {self.curr_action_type} action")
        print(self.output_override)

    def collate_episode(self):
        self.final_state_hist = np.vstack((self.final_state_hist, self.imu_state_hist[-3, :]))
       
        start_t = self.final_state_hist[-1, 2]
        end_t = self.final_state_hist[-1, 3]
        action_name = self.ACTION_CATEGORIES[int(self.final_state_hist[-1, 0])]
        self.pub_episode(start_t, end_t, action_name)
    
    def pub_episode(self, start_t, end_t, action_name):
        dur = end_t - start_t
         # Can publish new episode to sql
        self.db.insert_data_list("Episodes", 
        ["date", "start_t", "end_t", "duration", "user_id", "hand", "task_name", "action_name", "action_no"], 
        [(date.today(), start_t, end_t, dur, self.id, "R", self.task, action_name, int(self.task_data.loc[self.curr_action_no]['action_no']))])

    def update_robot_progress(self):
        try:
            # Get next row where action not completed
            next_action_row_i = self.task_data[self.task_data.done == False].index[0]

            if self.task_data.iloc[next_action_row_i]["user_type"] != "human":
                # Check robot actions are completed, assume other non-user actions are completed
                while self.task_data.iloc[next_action_row_i]["user_type"] != "human":
                    if self.task_data.iloc[next_action_row_i]["user_type"] == "robot":
                        col_names, actions_list = self.db.query_table('Episodes', 'all')
                        episodes = pd.DataFrame(actions_list, columns=col_names)
                        if self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("action_name")] in episodes.action_name.values:
                            self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("started")] = 1
                            self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("done")] = 1
                            next_action_row_i = self.task_data[self.task_data.done == False].index[0]
                            self.curr_action_start_t = datetime.now()
                        else:
                            return "continuing"
                    else:
                        self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("started")] = 1
                        self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("done")] = 1
                        next_action_row_i = self.task_data[self.task_data.done == False].index[0]
                        self.curr_action_start_t = datetime.now()

                self.curr_action_no = self.task_data.iloc[next_action_row_i]["action_no"]
                self.curr_action_type = self.task_data.iloc[next_action_row_i]["action_name"]
                if self.task_started:
                    self.usr_fdbck_pub.publish(f"Waiting for {self.curr_action_type} action")

        except IndexError:
            print(f"Looks like user {self.name} tasks are finished")
            self.usr_fdbck_pub.publish(f"Looks like user {self.name} tasks are finished")
            return "finished"

    def update_progress(self):
        action_name = self.ACTION_CATEGORIES[int(self.final_state_hist[-1, 0])]

        try:
            # Get next row where action not completed
            next_action_row_i = self.task_data[self.task_data.done == False].index[0]
        except IndexError:
            print(f"Looks like user {self.name} tasks are finished")
            self.usr_fdbck_pub.publish(f"Looks like user {self.name} tasks are finished")
            self.user_state = "finished"
            return

        # Check next action for user matches action completed
        next_action_expected = self.task_data.iloc[next_action_row_i]["action_name"]
        if action_name == next_action_expected:
            self.task_data.iloc[next_action_row_i, self.task_data.columns.get_loc("done")] = 1
            if self.task_data.iloc[next_action_row_i+1]["user_type"] == "human":
                self.task_data.iloc[next_action_row_i+1, self.task_data.columns.get_loc("started")] = 1
            next_action_row_i = self.task_data[self.task_data.done == False].index[0]
            self.curr_action_start_t = datetime.now()
        else:
            print(f"Updated user action ({action_name}) is not next expected ({next_action_expected})")

        self.curr_action_no = self.task_data.iloc[next_action_row_i]["action_no"]
        self.curr_action_type = self.task_data.iloc[next_action_row_i]["action_name"]
        if self.task_started:
            self.usr_fdbck_pub.publish(f"Waiting for {self.curr_action_type} action")

        self.user_state = "continuing"
        return
        
    def collate_har_seq(self):
        # 'Dilation' filter to remove single erroneous predictions
        #print('state hist: ', self.imu_state_hist)
        #print('pred hist: ', self.imu_pred_hist)
        if np.shape(self.imu_state_hist)[0] >= 4:
            self.update_robot_progress()

            if (self.imu_state_hist[-1, 0] == self.imu_state_hist[-3, 0]):
                self.imu_state_hist[-2, 0] = self.imu_state_hist[-1, 0]

            # Group predictions of same type together
            if self.imu_state_hist[-2, 0] == self.imu_state_hist[-3, 0]:
                i = np.where(self.imu_pred_hist[:, -1] == self.imu_state_hist[-3, 2])[0][0] #Get index where action starts
                self.imu_state_hist[-2, 1] = np.mean(self.imu_pred_hist[i:-1, int(self.imu_state_hist[-2, 0])].astype(float)) #Not convinced about this mean
                self.imu_state_hist[-2, 2] = self.imu_state_hist[-3, 2] # set start time
                self.imu_state_hist = np.delete(self.imu_state_hist, -3, 0)
            else:
                # New action predicted
                if self.task_started:
                    self.collate_episode()
                    self.update_progress()
            
            if self.task_started:
                if self.user_state != "finished":
                    self.predict_action_statuses()

    def gesture_handler(self, ges_idx, msg_time):
        if ges_idx == self.current_gesture[0]:
            self.current_gesture[2] = msg_time
        else:
            # Publish old gesture to episodic
            gesture = GESTURES[self.current_gesture[0]]
            if (gesture != "Null") and (gesture != "null"):
                start_t = self.current_gesture[1]
                end_t = self.current_gesture[2]
                self.pub_episode(start_t, end_t, gesture)

            # Start tracking new gesture
            self.current_gesture[0] = ges_idx
            self.current_gesture[1] = msg_time
            self.current_gesture[2] = msg_time
            gesture = GESTURES[self.current_gesture[0]]
            if (gesture != "Null") and (gesture != "null"):
                print(f"Gesture {gesture} detected")
                if self.task_started:
                    if gesture == "Left":
                        self.cmd_publisher.publish('next_action')
                        self.usr_fdbck_pub.publish(f"Sorry I'm behind, next action coming!")
                    elif gesture == "Stop":
                        self.usr_fdbck_pub.publish(f"STOP received")
                else:
                    if (gesture == "Wave") and (self.name == "unknown"):
                        self.cmd_publisher.publish(f'user_identification_{self.id}')
                    elif (gesture == "Forward") and (self.name != "unknown"):
                        print("Task starting")
                        self.cmd_publisher.publish('start')
