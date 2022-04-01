#!/usr/bin/env python3.7

import sys, os
import rospy
import argparse
import traceback
from diagnostic_msgs.msg import KeyValue
from pub_classes import diag_class, capability_class
from sam_custom_messages.msg import capability
from postgresql.database_funcs import database
import pandas as pd
import datetime
os.chdir(os.path.expanduser("~/catkin_ws/src/multimodal_human_robot_collaboration/"))

def test_robot_control_node():
    # ROS node setup
    rospy.init_node(f'test_robot_control_node', anonymous=True)
    frame_id = 'test_robot_control_node'
    diag_obj = diag_class(frame_id=frame_id, user_id=0, user_name="robot", queue=1)

    task_name = 'assemble_complex_box'
    try:
        db = database()
        cols, data = db.query_table(task_name, 'all')
        task_data = pd.DataFrame(data, columns=cols)

        human_row_idxs = []
        for index, row in task_data.iterrows():
            if row['user_type'] == 'human':
                human_row_idxs.append(index)

    except Exception as e:
        print(e)
        raise e

    rate = rospy.Rate(1/10) # 0.1hz
    i = 0

    col_names, act_data = db.query_table('future_action_predictions',rows=0)
    while (not rospy.is_shutdown()):# and (i < len(data)):
        try:
            print('here')
            
            time = datetime.datetime.utcnow()

            data_ins = "%s" + (", %s"*(len(col_names)-1))
            separator = ', '

            # Delete old rows for user
            sql_cmd = f"""DELETE FROM future_action_predictions WHERE user_id = 1;"""
            db.gen_cmd(sql_cmd)

            # Insert new rows for user for each action prediciton status
            sql_cmd = f"""INSERT INTO future_action_predictions ({separator.join(col_names)})
            VALUES """
            for i in range(task_data.shape[0]):
                if i != 0:
                    sql_cmd += ", "
                sql_cmd += f"""(1, 'unknown', '{time}', '{task_name}',
                {int(task_data.loc[human_row_idxs[i], 'action_no'])},
                '{task_data.loc[human_row_idxs[i], 'started']}',
                '{task_data.loc[human_row_idxs[i], 'done']}',
                '{task_data.loc[human_row_idxs[i], 'time_left']}')"""
            sql_cmd += ";"
            db.gen_cmd(sql_cmd)

            diag_obj.publish(0, "Running")
            
            print(task_data.loc[i])
            i=i+1
            if i == len(data):
                i=0

        except Exception as e:
            print(f"test_robot_control_node connection error: {e}")
            diag_obj.publish(2, f"Error: {e}")
            raise
        
        rate.sleep()
        print('here2')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run test robot controller ROS node')

    args = parser.parse_known_args()[0]

    try:
        test_robot_control_node()
    except rospy.ROSInterruptException:
        print("test_robot_controller ROS exception")
    except Exception as e:
        print("**test_robot_controller Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        pass
