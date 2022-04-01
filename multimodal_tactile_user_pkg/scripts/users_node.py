#!/usr/bin/env python3.7

import rospy
import numpy as np
from std_msgs.msg import String
from user import User
from sam_custom_messages.msg import hand_pos, capability, current_action, diagnostics, threeIMUs, skeleton
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class, capability_class
import argparse
import datetime, time
import pandas as pd
from postgresql.database_funcs import database
from global_data import SKELETON_FRAMES
import os
from os.path import join
from statistics import mean
from global_data import ACTIONS, TASKS, DEFAULT_TASK
import threading
os.chdir(os.path.expanduser("~/catkin_ws/src/multimodal_human_robot_collaboration/"))

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
                    default=False)
# parser.add_argument('--inclAdjParam',
#                     help='include user/task adjustment parameters for lstm model',
#                     choices=[True, False],
#                     default=False)

args = parser.parse_known_args()[0]
print(f"Users node settings: {args.task_type}")
database_stat = 1
shimmer_stat = 1
kinect_stat = 1
task_started = False
next_action = False


def setup_user(users, frame_id, task, name=None):
    id = len(users)+1
    if name is None:
        name = "unknown"

    users.append(User(name, id, frame_id, args.test))

    db = database()
    time = datetime.datetime.utcnow()
    sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = {id};"""
    db.gen_cmd(sql_cmd)
    sql_cmd = f"""DELETE FROM users WHERE user_id = {id};"""
    db.gen_cmd(sql_cmd)
    db.insert_data_list("users", ['user_id', 'user_name', 'last_active'], [(id, name, time)])

    folder = './sam_nodes/scripts/models_parameters/'
    file = f"metadata_users_{name}.csv"
    try:
        df = pd.read_csv(join(folder, file))
    except FileNotFoundError:
        df = None

    for action in ACTIONS:
        if df is not None:
            adj_factor = mean(df[f"{action}"])
        else:
            adj_factor = 0
        sql = f"UPDATE users SET {action} = {adj_factor} WHERE user_name = '{name}'"
        db.gen_cmd(sql)

    users[id-1].update_task(task)
    return users


def hand_pos_callback(data, users):
    if users[data.UserId-1].name != data.UserName:
        print(f"ERROR: users list name {users[data.UserId-1].name} does not match hand_pos msg name {data.UserName}")
    else:
        if data.Hand == 0:
            self.left_h_pos = [Pose.Position.X, Pose.Position.Y, Pose.Position.Z]
            self.left_h_rot = [Pose.Orientation.X, Pose.Orientation.Y, Pose.Orientation.Z, Pose.Orientation.W]      
        elif data.Hand == 1:
            self.right_h_pos = [Pose.Position.X, Pose.Position.Y, Pose.Position.Z]
            self.right_h_rot = [Pose.Orientation.X, Pose.Orientation.Y, Pose.Orientation.Z, Pose.Orientation.W]
        else:
            print(f"ERROR: hand_pos.hand msg is {data.Hand} but should be 0 (left) or 1 (right)")
    return


def imu_data_callback(data, users):
    i = [idx for idx, user in enumerate(users) if data.UserId == user.id]
    if i:
        i = i[0]
        if users[i].name != data.UserName:
            print(f"ERROR: users list name {users[i].name} does not match threeIMUs msg name {data.UserName}")
        else:
            positions = ['Hand', 'Wrist', 'Arm']
            type = ['linear', 'angular']
            axes = ['x', 'y', 'z']
            msg_time = datetime.datetime.utcfromtimestamp(data.Header.stamp.secs)
            data_list = []
            for p in positions:
                for t in type:
                    for a in axes:
                        data_list.append(getattr(getattr(getattr(data, p), t), a))
            assert len(data_list) == 18, "IMU data received is wrong length"
            users[i].perception.add_imu_data(data_list, msg_time)


def skeleton_callback(data, users):
    skeleton_data = []
    i = [idx for idx, user in enumerate(users) if data.UserId == user.id]
    if i:
        i = i[0]
        if users[i].name != data.UserName:
            print(f"ERROR: users list name {users[i].name} does not match skeleton msg name {data.UserName}")
        else:
            for frame in SKELETON_FRAMES:
                pose = getattr(data, frame)
                skeleton_data.append(pose.position.x)
                skeleton_data.append(pose.position.y)
                skeleton_data.append(pose.position.z)
                skeleton_data.append(pose.orientation.x)
                skeleton_data.append(pose.orientation.y)
                skeleton_data.append(pose.orientation.z)
                skeleton_data.append(pose.orientation.w)
            msg_time = datetime.datetime.utcfromtimestamp(data.Header.stamp.secs)
            users[i].perception.add_skel_data(skeleton_data, msg_time)


def current_action_callback(data, users):
    i = [idx for idx, user in enumerate(users) if data.UserId == user.id]
    if i:
        i = i[0]
        if users[i].name != data.UserName:
            print(f"ERROR: users list name {users[i].name} does not match current_action msg name {data.UserName}")
        else:
            msg_time = datetime.datetime.utcfromtimestamp(data.Header.stamp.secs)#to_sec())

            users[i]._har_pred_hist = np.vstack((users[i]._har_pred_hist, (np.hstack((data.ActionProbs, msg_time)))))
            users[i].collate_har_seq(task_started)

            if task_started:
                users[i].task_reasoning.predict_action_statuses(data.ActionProbs)


def sys_stat_callback(data, users):
    """callback for system status messages"""
    global database_stat, shimmer_stat, kinect_stat

    if data.Header.frame_id == 'Database_node':
        database_stat = data.DiagnosticStatus.level
    elif data.Header.frame_id == 'skeleton_viewer':
        kinect_stat = data.DiagnosticStatus.level
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
    global task_started, next_action
    if msg.data == 'start':
        task_started = True
    elif msg.data == 'next_action':
        next_action = True


def update_user_data_seq(user):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        user.perception.update_data_window()
        rate.sleep()


def users_node():
    global database_stat, kinect_stat, shimmer_stat, next_action
    frame_id = "users_node"
    rospy.init_node(frame_id, anonymous=True)
    keyvalues = []
    users = []
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1, keyvalues=keyvalues)

    rospy.Subscriber("SystemStatus", diagnostics, sys_stat_callback, (users))
    rospy.Subscriber("ProcessCommands", String, sys_cmd_callback)

    # Wait for postgresql node to be ready
    while database_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for postgresql node status, currently {database_stat}")
        diag_obj.publish(1, "Waiting for postgresql node")
        time.sleep(0.5)

    # Wait for kinect node to be ready
    while kinect_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for kinect_stat node status, currently {kinect_stat}")
        diag_obj.publish(1, "Waiting for kinect_stat node")
        time.sleep(0.5)

    user_threads = []
    for name in args.user_names:
        # Create user object
        users = setup_user(users, frame_id, args.task_type, name)
        # Thread to update sensor data windows
        user_threads.append(threading.Thread(target=update_user_data_seq, args=(users[-1],), daemon=True))
        user_threads[-1].start()

    # Wait for shimmer node to be ready
    while shimmer_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for shimmer node status, currently {shimmer_stat}")
        diag_obj.publish(1, "Waiting for shimmer node")
        time.sleep(0.5)

    rospy.Subscriber("HandStates", hand_pos, hand_pos_callback, (users))
    rospy.Subscriber("CurrentAction", current_action, current_action_callback, (users))
    rospy.Subscriber("IMUdata", threeIMUs, imu_data_callback, (users))
    rospy.Subscriber('SkeletonJoints', skeleton, skeleton_callback, (users))

    rate = rospy.Rate(2)  # 2hz, update predictions every 0.5 s
    while not rospy.is_shutdown():
        # rospy.loginfo(f"{frame_id} active")
        for user in users:
            user.perception.predict_actions()
        
        if next_action:
            for user in users:
                user.task_reasoning.next_action_override()
            next_action = False

        diag_obj.publish(0, "Running")
        rate.sleep()


def users_test_node():
    global database_stat
    frame_id = "users_node"
    rospy.init_node(frame_id, anonymous=True)
    keyvalues = []
    users = []
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1, keyvalues=keyvalues)

    rospy.Subscriber("SystemStatus", diagnostics, sys_stat_callback, (users))

    # Wait for postgresql node to be ready
    while database_stat != 0 and not rospy.is_shutdown():
        print(f"Waiting for postgresql node status, currently {database_stat}")
        diag_obj.publish(1, "Waiting for postgresql node")
        time.sleep(0.5)

    for name in args.user_names:
        users = setup_user(users, frame_id, args.task_type, name)

    rospy.Subscriber("CurrentAction", current_action, current_action_callback, (users))

    rate = rospy.Rate(2)  # 2hz, update predictions every 0.5 s
    while not rospy.is_shutdown():
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
