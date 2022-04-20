#!/usr/bin/env python3.7

from re import T
import sys, os
import rospy
import argparse
import traceback
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class, move_class
from multimodal_tactile_custom_msgs.msg import user_prediction, capability, diagnostics
from std_msgs.msg import String
from postgresql.database_funcs import database
import pandas as pd
import time
from datetime import datetime, date, timedelta
os.chdir(os.path.expanduser("~/catkin_ws/src/visuoTactileHRC/"))

database_stat = 1
user_node_stat = 1
start_trial = False
next_action = False
id_check = False
stop = False


def sys_stat_callback(data):
    """callback for system status messages"""
    global database_stat, user_node_stat
    if data.Header.frame_id == 'Database_node':
        database_stat = data.DiagnosticStatus.level
    elif data.Header.frame_id == 'users_node':
        user_node_stat = data.DiagnosticStatus.level
    elif data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')


def sys_cmd_callback(msg):
    """callback for system command messages"""
    global start_trial, next_action, id_check, stop
    if msg.data == 'start':
        start_trial = True
    elif msg.data == 'next_action':
        if not stop:
            next_action = True
        else:
            stop = False
    elif msg.data[0:19] == 'user_identification':
        id_check = True
    elif msg.data == 'Stop':
        stop = True


def send_robot_home(predictor, move_obj):
    print("Sending robot home")
    # Wait for confirmation task has been received
    while (predictor.robot_status != 'home') and (not rospy.is_shutdown()):
        move_obj.publish('home')
    predictor.robot_start_t = datetime.now().time()
    predictor.done = False
    # Wait for confirmation task has been completed
    while (not predictor.done) and (not rospy.is_shutdown()):
        time.sleep(0.01)
    # move_obj.publish('')
    return True


def id_check_position(predictor, move_obj):
    return True
    print("Sending robot to check ID")
    # Wait for confirmation task has been received
    while (predictor.robot_status != 'id_check') and (not rospy.is_shutdown()):
        move_obj.publish('id_check')
    predictor.robot_start_t = datetime.now().time()
    predictor.done = False
    # Wait for confirmation task has been completed
    while (not predictor.done) and (not rospy.is_shutdown()):
        time.sleep(0.01)
    # move_obj.publish('')
    return True


class future_predictor():
    def __init__(self):
        self.db = database()
        self.current_data = None
        self.fut_cols = ['user_id', 'user_name', 'task_name', 'last_r_action_no', 'next_r_action_no', 'est_t_remain', 'robo_task_t', 'robot_start_t', 'done']
        self.future_estimates = pd.DataFrame(columns=self.fut_cols)
        self.task_overview = None
        self.robot_status = None
        self.robot_start_t = datetime.now().time()
        self.update_predictions()
        self.task_now = None
        self.action_no_now = None
        self.done = False

    def update_predictions(self):

        # Get predictions from user reasoning
        col_names, actions_list = self.db.query_table('future_action_predictions', 'all')
        self.current_data = pd.DataFrame(actions_list, columns=col_names)

        # Get last updated robot actions
        col_names, robot_actions = self.db.query_table('robot_action_timings', 'all')
        last_robot_stats = pd.DataFrame(robot_actions, columns=col_names)

        active_users = pd.unique(self.current_data["user_id"])
        for user_id in active_users:
            # select specific user prdictions
            user_predictions = self.current_data.loc[self.current_data["user_id"]==user_id]
            user_name = user_predictions["user_name"][0]
            # Get actions list for task user is doing
            task_name = user_predictions["task_name"][0]
            task_cols, task_actions = self.db.query_table(task_name, 'all')
            task_data = pd.DataFrame(task_actions, columns=task_cols)
            self.task_overview = task_data

            # Get last action not completed by robot
            try:
                last_robot_action_no = last_robot_stats.loc[last_robot_stats["user_id"]==user_id]["last_completed_action_no"].values[0]

                i = self.future_estimates.loc[self.future_estimates['user_id'] == user_id].first_valid_index()
                if i is not None:
                    if self.future_estimates.loc[i]['done']:
                        last_robot_action_no = last_robot_stats.loc[last_robot_stats["user_id"]==user_id]["next_r_action_no"].values[0]
            except (KeyError, IndexError) as e:
                # user hasn't been entered into table yet
                last_robot_action_no = -1

            # get list of actions after last completed robot action
            if last_robot_action_no != -1:
                tasks_left = task_data.drop(task_data.index[:task_data.loc[task_data['action_no']==last_robot_action_no].first_valid_index()+1])
            else:
                tasks_left = task_data

            # find index of next task for robot
            try:
                next_robot_action_idx = int(tasks_left[tasks_left['user_type']=='robot'].first_valid_index())
                next_robot_action_no = int(tasks_left.loc[next_robot_action_idx]['action_no'])

                # if first action robot should start immediately
                if next_robot_action_no != 0:
                    # find previously dependent user action, should be redundant
                    prev_user_action_no = next_robot_action_no
                    while task_data.iloc[prev_user_action_no]["user_type"] != 'human':
                        prev_user_action_no -= 1

                    # Relevant user action predictions
                    prev_user_action_pred = user_predictions.loc[user_predictions["action_no"]==prev_user_action_no]

                    # Get time until robot should start action
                    Time2UserNeedRobot = timedelta(seconds=prev_user_action_pred["time_left"].values[0])
                    print(f"prev user action no: {prev_user_action_no}, next robot action no: {next_robot_action_no}")
                else:
                    Time2UserNeedRobot = timedelta()

                RobotActionTime = tasks_left.loc[next_robot_action_idx]['default_time']
                if tasks_left.loc[next_robot_action_idx]['prev_dependent']:
                    Time2RoboStart = Time2UserNeedRobot
                else:
                    Time2RoboStart = Time2UserNeedRobot - RobotActionTime

                print(f"\nUser {user_name} Tasks Left:")
                print(tasks_left, "\n")
            except TypeError:
                print("User task complete")
                next_robot_action_no = last_robot_action_no
                Time2UserNeedRobot = timedelta()
                RobotActionTime = timedelta()
                Time2RoboStart = timedelta()
                r_done = True

            # Update future estimates table
            i = self.future_estimates.loc[self.future_estimates['user_id'] == user_id].first_valid_index()
            if i is not None:
                # Check to make sure 'done' indicator is kept consistent
                if next_robot_action_no != last_robot_action_no:
                    if next_robot_action_no == self.future_estimates.loc[i]['next_r_action_no']:
                        r_done = self.future_estimates.loc[i]['done']
                    else:
                        r_done = False
                self.future_estimates.loc[i] = [user_id, user_name, task_name, last_robot_action_no, next_robot_action_no, Time2UserNeedRobot, RobotActionTime, Time2RoboStart, r_done]
            else:
                new_user_data = [user_id, user_name, task_name, last_robot_action_no, next_robot_action_no, Time2UserNeedRobot, RobotActionTime, Time2RoboStart, False]
                self.future_estimates = self.future_estimates.append(pd.Series(new_user_data, index=self.fut_cols), ignore_index=True)
            # except TypeError:
            #     print("User task complete")

        if not self.future_estimates.empty:
            print("\nFuture Estimates:")
            print(self.future_estimates, "\n")

        self.update_robot_completed_actions_table()

    def update_robot_completed_actions_table(self):
        # Update future prediction in sql table
        for _, row in self.future_estimates.iterrows():
            sql_cmd = f"""DELETE FROM robot_action_timings WHERE user_id = {row['user_id']};"""
            self.db.gen_cmd(sql_cmd)
            cols = ["user_id", "user_name", "task_name", "last_completed_action_no", "next_r_action_no", "robot_start_t"]
            data = (int(row["user_id"]), row["user_name"], row["task_name"], int(row["last_r_action_no"]), int(row["next_r_action_no"]), row["robot_start_t"])
            self.db.insert_data_list("robot_action_timings", cols, [data])

    def robot_stat_callback(self, msg):
        """Callback for updates to robot status"""
        print(f"robot stat callback: {msg.data} {self.robot_status} {self.task_now} {self.action_no_now}")
        if msg.data == 'Done':
            self.update_episodic()
            self.done = True

        if msg.data != "waiting_for_handover":
            self.robot_status = msg.data

    def update_episodic(self):
        """Add completed robot actions to episodic memory"""
        date_now = date.today()
        end_t = datetime.now().time()
        dur = datetime.combine(date.min, end_t) - datetime.combine(date.min, self.robot_start_t)

        if (self.robot_status != "Done") and (self.robot_status != "Waiting") and (self.robot_status is not None):
            if "move_stack" in self.robot_status:
                comp_action = "move_stack"
            else:
                comp_action = str(self.robot_status)
            # Can publish new episode to sql
            epi_cols = ["date", "start_t", "end_t", "duration", "user_id", "hand", "task_name", "action_name", "action_no"]
            epi_data = [(date_now, self.robot_start_t, end_t, dur, 0, '-', self.task_now, comp_action, self.action_no_now)]
            self.db.insert_data_list("Episodes", epi_cols, epi_data)
            self.task_now = None
            self.action_no_now = None


class robot_solo_task():
    def __init__(self):
        self.db = database()
        self.task_name = "move_stack"
        self.task_overview = None
        self.next_action_id = 0
        self.next_action = None
        self.next_action_time = None
        self.finished = False
        self.update_task_data()

    def update_task_data(self):
        task_cols, task_actions = self.db.query_table(self.task_name, 'all')
        self.task_overview = pd.DataFrame(task_actions, columns=task_cols)

        self.next_action_id = 0
        if self.task_name == "move_stack":
            self.next_action = self.task_overview.loc[
                self.task_overview['action_no'] == self.next_action_id, ['action_name']].values[0][0]+"_"+str(self.next_action_id) #bodge to make all different
        else:
            self.next_action = self.task_overview.loc[
                self.task_overview['action_no'] == self.next_action_id, ['action_name']].values[0][0]
        
        self.next_action_time = pd.Timedelta(self.task_overview.loc[
            self.task_overview['action_no'] == self.next_action_id, ['default_time']].values[0][0])

        self.update_actions_table()

    def update_progress(self):
        try:
            self.next_action_id += 1
            if self.task_name == "move_stack":
                self.next_action = self.task_overview.loc[
                    self.task_overview['action_no'] == self.next_action_id, ['action_name']].values[0][0]+"_"+str(self.next_action_id) #bodge to make all different
            else:
                self.next_action = self.task_overview.loc[
                    self.task_overview['action_no'] == self.next_action_id, ['action_name']].values[0][0]
        except IndexError:
            print("Looks like user robot task is finished")
            self.finished = True

        self.update_actions_table()

    def update_actions_table(self, t_adj=timedelta()):
        sql_cmd = """DELETE FROM robot_action_timings WHERE user_id = 0;"""
        self.db.gen_cmd(sql_cmd)
        cols = ["user_id", "user_name", "task_name", "last_completed_action_no", "next_r_action_no", "robot_start_t"]
        data = [0, "robot", self.task_name, self.next_action_id-1, self.next_action_id, self.next_action_time-t_adj]
        self.db.insert_data_list("robot_action_timings", cols, [data], verbose=False)


def robot_control_node():
    global next_action, id_check, stop

    # ROS node setup
    frame_id = 'robot_control_node'
    rospy.init_node(frame_id, anonymous=True)
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="robot", queue=1)
    diag_obj.publish(1, "Starting")

    rospy.Subscriber("SystemStatus", diagnostics, sys_stat_callback)
    rospy.Subscriber("ProcessCommands", String, sys_cmd_callback)
    # Wait for postgresql node to be ready
    while database_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for postgresql node status, currently {database_stat}")
        diag_obj.publish(1, "Waiting for postgresql node")
        time.sleep(0.5)

    # Wait for users node to be ready
    while user_node_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for users node status, currently {user_node_stat}")
        diag_obj.publish(1, "Waiting for users node")
        time.sleep(0.5)

    predictor = future_predictor()
    robot_task = robot_solo_task()
    move_obj = move_class(frame_id=frame_id, queue=10)
    

    rospy.Subscriber("RobotStatus", String, predictor.robot_stat_callback)

    rate = rospy.Rate(1)  # 1hz
    #db = database()
    home = False
    #home = send_robot_home(predictor, move_obj)
    print("Ready for trial to start")
    while (not start_trial) and (not rospy.is_shutdown()):
        diag_obj.publish(0, "Waiting for trial to start")
        # if id_check:
        #     id_check_position(predictor, move_obj)
        #     id_check = False
        time.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            predictor.update_predictions()

            # Select row with minimum time until robot required
            row = predictor.future_estimates[
                predictor.future_estimates.robot_start_t ==
                predictor.future_estimates.robot_start_t.min()]

            if (not row.empty) and (not stop):
                if ((row['robot_start_t'][0] <= pd.Timedelta(0)) or (next_action)) and (row['done'][0] is False):
                    # if time to next colab < action time start colab action
                    home = False
                    # get action and task details
                    predictor.task_now = row['task_name'][0]
                    predictor.action_no_now = row['next_r_action_no'][0]
                    action = predictor.task_overview.loc[predictor.action_no_now]['action_name']#.values[0]
                    print(f"action: {action}")
                    # Wait for confirmation task has been received
                    while (predictor.robot_status != action) and (not rospy.is_shutdown()):
                        move_obj.publish(action)
                    predictor.robot_start_t = datetime.now().time()
                    predictor.done = False
                    # Wait for confirmation task has been completed
                    while (not predictor.done) and (not rospy.is_shutdown()):
                        time.sleep(0.01)
                    predictor.future_estimates.loc[predictor.future_estimates['user_id']==row['user_id'].values[0], 'done'] = True
                    next_action = False

                elif (not robot_task.finished) and ((row['robot_start_t'][0] > robot_task.next_action_time) or (row['done'][0] is True)):
                    # if time to colab action > time to do solo action
                    home = False
                    predictor.task_now = robot_task.task_name
                    predictor.action_no_now = robot_task.next_action_id
                    print(f"Robot solo task {robot_task.next_action}")
                    # Wait for confirmation task has been received
                    while (predictor.robot_status != robot_task.next_action) and (not rospy.is_shutdown()):
                        move_obj.publish(robot_task.next_action)
                    predictor.robot_start_t = datetime.now().time()
                    predictor.done = False
                    # Wait for confirmation task has been completed
                    while (not predictor.done) and (not rospy.is_shutdown()):
                        robot_task.update_actions_table(datetime.now()-datetime.combine(datetime.now().date(), predictor.robot_start_t))
                        time.sleep(0.1)
                    robot_task.update_progress()

                elif not home:
                    # else wait for next colab action
                    predictor.task_now = None
                    predictor.action_no_now = None
                    home = send_robot_home(predictor, move_obj)
                    

            diag_obj.publish(0, "Running")
            rospy.loginfo(f"{frame_id} active")

        except rospy.exceptions.ROSException:
            pass
        except Exception as e:
            print(f"robot_control_node error: {e}")
            diag_obj.publish(2, f"Error: {e}")
            traceback.print_exc(file=sys.stdout)

        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run robot controller ROS node')
    parser.add_argument('--test', '-T',
                        help='Send test sequence of movements',
                        default=False,
                        action="store_true")

    args = parser.parse_known_args()[0]

    try:
        robot_control_node()
    except rospy.ROSInterruptException:
        print("robot_controller ROS exception")
    except Exception as e:
        print("**robot_controller Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
