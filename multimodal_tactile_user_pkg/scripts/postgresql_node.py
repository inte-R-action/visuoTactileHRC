#!/usr/bin/env python3.7
# Should be run from visuoTactileHRC folder

from fileinput import filename
import sys, os
import rospy
import argparse
import traceback
from postgresql.database_funcs import database
from pub_classes import diag_class
# from sam_nodes.scripts.robot_controller import sys_stat_callback
from multimodal_tactile_custom_msgs.msg import diagnostics
from global_data import ACTIONS, TASKS
from os import listdir
from os.path import isfile, join
import csv
import pandas as pd
from statistics import mean

os.chdir(os.path.expanduser("~/catkin_ws/src/visuoTactileHRC/"))

#Define tables: tables = [{name, [col1 cmd, col2 cmd, ...]}, ]
tables_to_make = ['tasks', 'actions', 'users', 'episodes', 'assemble_box', 'assemble_chair', 'assemble_complex_box', 'assemble_complex_box_manual', 'stack_tower', 'move_stack', 'future_action_predictions', 'robot_action_timings']
tables = [['tasks', ["task_id SERIAL PRIMARY KEY",
                    "task_name VARCHAR(255) NOT NULL UNIQUE"]+[f"{action} FLOAT" for action in ACTIONS]],
        ['actions', ["action_id SERIAL PRIMARY KEY",
                    "action_name VARCHAR(255) NOT NULL UNIQUE",
                    "std_dur_s INTERVAL",
                    "user_type VARCHAR(5)"]],
        ['users', ["user_id SERIAL PRIMARY KEY",
                    "user_name VARCHAR(255) NOT NULL UNIQUE",
                    "last_active TIMESTAMPTZ"]+[f"{action} FLOAT" for action in ACTIONS]],
        ['episodes', ["episode_id SERIAL PRIMARY KEY",
                    "date DATE",
                    "start_t TIME",
                    "end_t TIME",
                    "duration INTERVAL",
                    "user_id SMALLINT",
                    "hand CHAR(1)",
                    "task_name VARCHAR(255) REFERENCES tasks(task_name)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "action_no SMALLINT"]],
        ['assemble_box', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        ['assemble_chair', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        ['assemble_complex_box', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        ['assemble_complex_box_manual', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        ['stack_tower', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        ['move_stack', ["action_no SERIAL PRIMARY KEY",
                    "action_id INTEGER REFERENCES actions(action_id)",
                    "action_name VARCHAR(255) REFERENCES actions(action_name)",
                    "default_time INTERVAL",
                    "user_type VARCHAR(5)",
                    "prev_dependent BOOL"]],
        # ['current_actions', ["user_id INTEGER REFERENCES users(user_id) UNIQUE",
        #             "user_name VARCHAR(255) REFERENCES users(user_name)",
        #             "updated_t TIMESTAMPTZ",
        #             "task_name VARCHAR(255) REFERENCES tasks(task_name)",
        #             "current_action_no INTEGER",
        #             "start_time TIMESTAMPTZ"]],
        ['future_action_predictions', ["user_id INTEGER REFERENCES users(user_id)",
                    "user_name VARCHAR(255) REFERENCES users(user_name)",
                    "updated_t TIMESTAMPTZ",
                    "task_name VARCHAR(255) REFERENCES tasks(task_name)",
                    "action_no INTEGER",
                    "started FLOAT",
                    "done FLOAT",
                    "time_left FLOAT"]],
        # ['robot_future_estimates', ["user_id INTEGER REFERENCES users(user_id) UNIQUE",
        #             "user_name VARCHAR(255) REFERENCES users(user_name)",
        #             "task_name VARCHAR(255) REFERENCES tasks(task_name)",
        #             "current_action_no INTEGER",
        #             "est_t_remain INTERVAL",
        #             "robo_task_t INTERVAL",
        #             "robot_start_t INTERVAL",
        #             "done BOOL"]],
        ['robot_action_timings', ["user_id INTEGER REFERENCES users(user_id) UNIQUE",
                    "user_name VARCHAR(255) REFERENCES users(user_name)",
                    "task_name VARCHAR(255) REFERENCES tasks(task_name)",
                    "last_completed_action_no INTEGER",
                    "next_r_action_no INTEGER",
                    "robot_start_t INTERVAL"]]]


def sys_stat_callback(data):
    """callback for system status messages"""
    if data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')


def make_tables(db, del_tab=True):
    try:
        table_avail = [item[0] for item in tables]
        assert all(elem in table_avail for elem in tables_to_make), "Some tables to make not in tables list"
        curr_tables = db.table_list()
        print(f"Tables to create: {tables_to_make}")

        for name in tables_to_make:
            if (name in curr_tables) and not del_tab:
                print(f"Table '{name}' already exists, leaving as is")
            else:
                if name in curr_tables:#del_tab:
                    print(f"Table '{name}' alredy exists, deleting")
                    db.remove_table(name)
                _, cmd = [i for i in tables if i[0] == name][0]
                db.create_table(name, cmd)
                print(f"Successfully created table '{name}'")

    except AssertionError as e:
        print(f"Assertion Error: {e}")
        raise
    except Exception as e:
        print(f"Make Tables Error: {e}")
        raise


def update_meta_data(db):
    try:
        for task in TASKS:
            folder = './multimodal_tactile_user_pkg/scripts/models_parameters/'
            file = f"metadata_tasks_{task}.csv"
            specific_name = file.split('metadata_tasks_')[1][0:-4]
            try:
                df = pd.read_csv(join(folder, file))
            except FileNotFoundError:
                df = None

            try:
                for action in ACTIONS:
                    if df is not None:
                        adj_factor = mean(df[f"{action}"])
                    else:
                        adj_factor = 0

                    sql = f"UPDATE tasks SET {action} = {adj_factor} WHERE task_name = '{specific_name}'"
                    db.gen_cmd(sql)

            except Exception as e:
                print(f"Error with file {file}")
                print(e)

    except Exception as e:
        print(f"Error with file {file}")
        print(e)

def load_tables(db):
    base_dir = os.getcwd()+'/multimodal_tactile_user_pkg/scripts/postgresql/'
    for name in tables_to_make:
        try:
            db.csv_import(f"{base_dir}{name}.csv", tab_name=name)
            if name in ('assemble_box', 'assemble_chair', 'stack_tower', 'move_stack', 'assemble_complex_box', 'assemble_complex_box_manual'):
                # Update times and action ids from actions table
                sql = f"UPDATE {name} SET action_id = actions.action_id, default_time = actions.std_dur_s FROM actions WHERE actions.action_name = {name}.action_name"
                db.gen_cmd(sql)

            update_meta_data(db)

            print(f"Loaded data into '{name}'")

        except FileNotFoundError:
            print(f"WARNING: Load table file not found for '{name}' at {base_dir}{name}.csv")
        except Exception as e:
            print(f"Load Table Error: {e}")
            raise
    print("Load Tables Completed")


def save_tables(db, tables_to_save='all', file_path=None, verbose=True):
    if tables_to_save == 'all':
        tables_to_save = db.table_list(verbose=False)
    
    for table in tables_to_save:
        try:
            db.csv_export(table, file_path=f"{file_path}/{table}.csv", verbose=True)
        except Exception as e:
            if verbose:
                print(e)
            raise


def shutdown(db):
    #always save tables to dump on exit
    try:
        save_tables(db, tables_to_save='all', file_path=os.path.dirname(__file__)+'/postgresql/dump', verbose=False)
    except Exception as e:
        print(f"Dump tables error: {e}")
        raise

    print("Database node shutdown")


def database_run(db):
    # ROS node setup
    frame_id = 'Database_node'
    rospy.init_node(frame_id, anonymous=True)
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="N/A", queue=1)
    rospy.Subscriber('SystemStatus', diagnostics, sys_stat_callback)

    rate = rospy.Rate(1) # 1hz
    try:
        # Test connection
        db.connect(verbose=True)
        db.disconnect(verbose=True)
        # Make and load predefined tables
        make_tables(db)
        load_tables(db)
        diag_obj.publish(1, "Tables loaded")
    except Exception as e:
        print(f"Database node create database error: {e}")
        diag_obj.publish(2, f"Error: {e}")
        raise

    while not rospy.is_shutdown():
        try:
            # Test database connection to ensure running smoothly
            db.connect()
            db.disconnect()
            diag_obj.publish(0, "Running")
        except Exception as e:
            print(f"Database connection error: {e}")
            diag_obj.publish(2, f"Error: {e}")
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run database main ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=False,
                        action="store_true")

    args = parser.parse_known_args()[0]
    db = None
    try:
        db = database()
        database_run(db)
    except rospy.ROSInterruptException:
        print("database_run ROS exception")
    except Exception as e:
        print("**Database Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        if db is not None:
            shutdown(db)
        pass
