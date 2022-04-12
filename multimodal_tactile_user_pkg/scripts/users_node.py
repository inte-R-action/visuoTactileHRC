#!/usr/bin/env python3.7

import rospy
import numpy as np
from std_msgs.msg import String
from user import User
from multimodal_tactile_custom_msgs.msg import current_action, diagnostics, threeIMUs
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class
import argparse
import time
from datetime import datetime
import pandas as pd
from postgresql.database_funcs import database
import os
from global_data import ACTIONS, TASKS, DEFAULT_TASK, GESTURES, USER_PARAMETERS
import threading
from vision_recognition.qrcode_recognition import read_QR

os.chdir(os.path.expanduser("~/catkin_ws/src/visuoTactileHRC/"))

# Argument parsing
parser = argparse.ArgumentParser(
    description='Node to control data flow between users and rest of system')

parser.add_argument('--user_names', '-N',
                    nargs='*',
                    help='Set name of user, default: unknown',
                    default='j',
                    type=lambda s: [str(item) for item in s.split(',')])
parser.add_argument('--task_type', '-T',
                    help='Task for users to perform, options: assemble_complex_box (default)',
                    choices=TASKS,
                    default=DEFAULT_TASK)
parser.add_argument('--test',
                    help='Test mode without sensors',
                    choices=[True, False],
                    default=True)

args = parser.parse_known_args()[0]
print(f"Users node settings: {args.task_type}")
database_stat = 1
shimmer_stat = 1
task_started = False
next_action = False
id_check = 0


def perform_id_check(users, usr_fdbck_pub, frame_id):
    usr_fdbck_pub.publish("Face camera for ID check")
    i = [idx for idx, user in enumerate(users) if int(id_check) == user.id][0]
    success, name = read_QR()
    
    if success:
        try:
            users[i].update_user_details(name=name, task=USER_PARAMETERS[name], frame_id=frame_id)
            db = database()
            sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = {users[i].id};"""
            db.gen_cmd(sql_cmd)
            sql_cmd = f"""DELETE FROM users WHERE user_id = {users[i].id};"""
            db.gen_cmd(sql_cmd)
            db.insert_data_list("users", ['user_id', 'user_name', 'last_active'], [(users[i].id, name, datetime.now())])

            usr_fdbck_pub.publish(f"Hello {name}!")
            time.sleep(1)
            usr_fdbck_pub.publish("Gesture forwards to start task")
        except Exception as e:
            success = False
    else:
        usr_fdbck_pub.publish("ID check failed :(, I'll try again")
        time.sleep(1)
    return success


def setup_user(users, frame_id, task, name=None):
    id = len(users)+1
    if name is None:
        name = "unknown"

    users.append(User(name, id, frame_id, args.test))

    db = database()
    sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = {id};"""
    db.gen_cmd(sql_cmd)
    sql_cmd = f"""DELETE FROM users WHERE user_id = {id};"""
    db.gen_cmd(sql_cmd)
    db.insert_data_list("users", ['user_id', 'user_name', 'last_active'], [(id, name, datetime.utcnow())])

    users[id-1].update_task(task)
    return users


def imu_data_callback(data, users):
    i = [idx for idx, user in enumerate(users) if data.UserId == user.id]
    if i:
        i = i[0]
        # if users[i].name != data.UserName:
        #     print(f"ERROR: users list name {users[i].name} does not match threeIMUs msg name {data.UserName}")
        # else:
        positions = ['Hand', 'Wrist', 'Arm']
        type = ['linear', 'angular']
        axes = ['x', 'y', 'z']
        msg_time = datetime.utcfromtimestamp(data.Header.stamp.secs)
        data_list = []
        for p in positions:
            for t in type:
                for a in axes:
                    data_list.append(getattr(getattr(getattr(data, p), t), a))
        assert len(data_list) == 18, "IMU data received is wrong length"
        users[i].perception.add_imu_data(data_list, msg_time)


def current_action_callback(data, users):
    i = [idx for idx, user in enumerate(users) if data.UserId == user.id]
    if i:
        i = i[0]
        # if users[i].name != data.UserName:
        #     print(f"ERROR: users list name {users[i].name} does not match current_action msg name {data.UserName}")
        # else:
        msg_time = datetime.utcfromtimestamp(data.Header.stamp.secs)#to_sec())

        # check if actions or gestures output
        if data.Header.frame_id[-8:] == '_actions':
            # Only select relevant classifier output
            try:
                action_idx = users[i].ACTION_CATEGORIES.index(users[i].task_reasoning.curr_action_type)-1
                null_probs = 1-data.ActionProbs[action_idx]

                if data.ActionProbs[action_idx] > null_probs:
                    users[i].task_reasoning.imu_state_hist = np.vstack((users[i].task_reasoning.imu_state_hist, [float(users[i].ACTION_CATEGORIES.index(users[i].task_reasoning.curr_action_type)), 0, msg_time, msg_time]))
                else:
                    users[i].task_reasoning.imu_state_hist = np.vstack((users[i].task_reasoning.imu_state_hist, [float(users[i].ACTION_CATEGORIES.index('null')), 0, msg_time, msg_time]))

            except ValueError as e:
                # When action is robot action so not found in user action list
                users[i].task_reasoning.imu_state_hist = np.vstack((users[i].task_reasoning.imu_state_hist, [float(users[i].ACTION_CATEGORIES.index('null')), 0, msg_time, msg_time]))
                null_probs = 1
            
            users[i].task_reasoning.imu_pred_hist = np.vstack((users[i].task_reasoning.imu_pred_hist, (np.hstack((null_probs, data.ActionProbs, msg_time)))))
            users[i].task_reasoning.collate_har_seq()
        
        elif data.Header.frame_id[-9:] == '_gestures':
            users[i].task_reasoning.gesture_handler(np.argmax(data.ActionProbs), msg_time)


def sys_stat_callback(data, users):
    """callback for system status messages"""
    global database_stat, shimmer_stat

    if data.Header.frame_id == 'Database_node':
        database_stat = data.DiagnosticStatus.level
    elif data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')
    elif users:
        for i in range(len(users)):
            if data.Header.frame_id == f'shimmerBase {users[i].name} {users[i].id} node':
                users[i].shimmer_ready = data.DiagnosticStatus.level
            elif data.Header.frame_id == 'fakeIMUpub_node':
                users[i].shimmer_ready = data.DiagnosticStatus.level
            elif data.Header.frame_id == 'fakeSensorspub_node':
                users[i].shimmer_ready = data.DiagnosticStatus.level

        shimmer_stat = max(users[i].shimmer_ready for i in range(len(users)))


def sys_cmd_callback(msg):
    """callback for system command messages"""
    global task_started, next_action, id_check
    print(msg.data)
    if msg.data == 'start':
        task_started = True
    elif msg.data == 'next_action':
        next_action = True
    elif msg.data[0:19] == 'user_identification':
        id_check = msg.data[-1]


def update_user_data_seq(user):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        user.perception.update_data_window()
        rate.sleep()


def users_node():
    global database_stat, shimmer_stat, next_action, id_check
    frame_id = "users_node"
    rospy.init_node(frame_id, anonymous=True)
    keyvalues = []
    users = []
    user_threads = []
    num_users = 1
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1, keyvalues=keyvalues)

    rospy.Subscriber("SystemStatus", diagnostics, sys_stat_callback, (users))
    rospy.Subscriber("ProcessCommands", String, sys_cmd_callback)
    usr_fdbck_pub = rospy.Publisher('UserFeedback', String, queue_size=10)

    # Wait for postgresql node to be ready
    while database_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for postgresql node status, currently {database_stat}")
        diag_obj.publish(1, "Waiting for postgresql node")
        usr_fdbck_pub.publish("Waiting for postgresql node")
        time.sleep(0.5)

    usr_fdbck_pub.publish("Initialising User")
    for _ in range(num_users): # args.user_names:
        name = "unknown"
        # Create user object
        users = setup_user(users, frame_id, args.task_type, name)
        # Thread to update sensor data windows
        user_threads.append(threading.Thread(target=update_user_data_seq, args=(users[-1],), daemon=True))
        user_threads[-1].start()

    # Wait for shimmer node to be ready
    while shimmer_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for shimmer node status, currently {shimmer_stat}")
        diag_obj.publish(1, "Waiting for shimmer node")
        usr_fdbck_pub.publish("Waiting for shimmer node")
        time.sleep(0.5)

    rospy.Subscriber("CurrentAction", current_action, current_action_callback, (users))
    rospy.Subscriber("IMUdata", threeIMUs, imu_data_callback, (users))

    rate = rospy.Rate(2)  # 2hz, update predictions every 0.5 s

    diag_obj.publish(0, "Running")
    time.sleep(1)
    usr_fdbck_pub.publish("Wave to start system")
    while not rospy.is_shutdown():
        # rospy.loginfo(f"{frame_id} active")
        for user in users:
            user.task_reasoning.task_started = task_started
            user.perception.predict_actions()
            user.perception.predict_gestures()                
        
        if next_action:
            for user in users:
                user.task_reasoning.next_action_override()
            next_action = False

        if (id_check != 0) and (not task_started):
            id_success = perform_id_check(users, usr_fdbck_pub, frame_id)
            if id_success:
                id_check = 0
        
        diag_obj.publish(0, "Running")
        rate.sleep()


def users_test_node():
    global database_stat, next_action, id_check
    frame_id = "users_node"
    rospy.init_node(frame_id, anonymous=True)
    keyvalues = []
    users = []
    user_threads = []
    num_users = 1
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1, keyvalues=keyvalues)

    rospy.Subscriber("SystemStatus", diagnostics, sys_stat_callback, (users))
    rospy.Subscriber("ProcessCommands", String, sys_cmd_callback)
    usr_fdbck_pub = rospy.Publisher('UserFeedback', String, queue_size=10)

    # Wait for postgresql node to be ready
    while database_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for postgresql node status, currently {database_stat}")
        diag_obj.publish(1, "Waiting for postgresql node")
        usr_fdbck_pub.publish("Waiting for postgresql node")
        time.sleep(0.5)

    usr_fdbck_pub.publish("Initialising User")
    for _ in range(num_users): # args.user_names:
        name = "unknown"
        # Create user object
        users = setup_user(users, frame_id, args.task_type, name)
        # Thread to update sensor data windows
        user_threads.append(threading.Thread(target=update_user_data_seq, args=(users[-1],), daemon=True))
        user_threads[-1].start()

    rospy.Subscriber("CurrentAction", current_action, current_action_callback, (users))
    rospy.Subscriber("IMUdata", threeIMUs, imu_data_callback, (users))

    rate = rospy.Rate(2)  # 2hz, update predictions every 0.5 s

    diag_obj.publish(0, "Running")
    time.sleep(1)
    usr_fdbck_pub.publish("Wave to start system")
    while not rospy.is_shutdown():
        # rospy.loginfo(f"{frame_id} active")
        for user in users:
            user.task_reasoning.task_started = task_started
            user.perception.predict_actions()
            user.perception.predict_gestures()                
        
        if next_action:
            for user in users:
                user.task_reasoning.next_action_override()
            next_action = False

        if (id_check != 0) and (not task_started):
            id_success = perform_id_check(users, usr_fdbck_pub, frame_id)
            if id_success:
                id_check = 0
        
        diag_obj.publish(0, "Running")
        rate.sleep()


if __name__ == '__main__':
    try:
        if not args.test:
            users_node()
        else:
            users_test_node()
    except rospy.ROSInterruptException:
        pass

    print("Users node shutdown")
