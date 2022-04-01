/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 17-February-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 *
 *********************************************************************************
 */

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <unistd.h>
#include <map>
#include "sam_custom_messages/diagnostics.h"

using namespace std;

string gripper_action = "";
string done_msg = "";

// callback function to get the status from the tactile sensor
void robotCommandStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    gripper_action.clear();
    gripper_action = msg->data;

    if( gripper_action == "grasp" )
    {
        // close the gripper to the maximum value of rPR = 255
        std::cout << "CLOSE GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        printf("IN PROGRESS: grasp \n");
        sleep(3);
        printf("COMPLETED: grasp \n");
        done_msg = "grasp_completed";
    }
    else if( gripper_action == "release" )
    {
        // open the gripper to the maximum value of rPR = 0
        std::cout << "OPEN GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        printf("IN PROGRESS: release \n");
        sleep(3);
        printf("COMPLETED: release \n");
        done_msg = "release_completed";
    }
    else if ( gripper_action == "completion acknowledged" )
    {
        printf("Acknowledge from UR received \n");
        done_msg = "acknowledge received";
    }
    else
    {
        // keep current configuration
    }
}

int main(int argc, char** argv)
{
    // Set up ROS stuff
    string frame_id = "rq_gripper_2F140";
    ros::init(argc, argv, frame_id);

    ros::NodeHandle node_handle;

    ros::Publisher diag_obj = node_handle.advertise<sam_custom_messages::diagnostics>("SystemStatus", 10);
    sam_custom_messages::diagnostics diag_msg;
    diag_msg.Header.stamp = ros::Time::now();
    diag_msg.Header.seq = 0;
    diag_msg.Header.frame_id = frame_id;
    diag_msg.UserId = 0;
    diag_msg.UserName = "N/A";
    diag_msg.DiagnosticStatus.level = 1; // 0:ok, 1:warning, 2:error, 3:stale
    diag_msg.DiagnosticStatus.name = frame_id;
    diag_msg.DiagnosticStatus.message = "Starting...";
    diag_msg.DiagnosticStatus.hardware_id = "N/A";
    //diag_msg.DiagnosticStatus.values = keyvalues;
    diag_obj.publish(diag_msg);
    ros::Time diag_timeout = ros::Time::now();

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher gripperStatusPub = node_handle.advertise<std_msgs::String>("Gripper2UR", 1);
    ros::Subscriber robotStatusSub = node_handle.subscribe("UR2Gripper", 1, robotCommandStatusCallback);

    ros::spinOnce();
    loop_rate.sleep();

    std_msgs::String msg;

    while( ros::ok() )
    {
        
        if (gripper_action == "grasp" || gripper_action == "release")
        {
            while (done_msg != "acknowledge received")
            {
                msg.data = done_msg;
                gripperStatusPub.publish(msg);
            }
        }
        
        msg.data = "gripper_ready";
        gripperStatusPub.publish(msg);

        if (ros::Time::now()-diag_timeout > ros::Duration(3))
        {
            diag_msg.DiagnosticStatus.level = 0; // 0:ok, 1:warning, 2:error, 3:stale
            diag_msg.DiagnosticStatus.message = "Ok";
            diag_msg.Header.stamp = ros::Time::now();
            diag_msg.Header.seq++;
            diag_obj.publish(diag_msg);
            diag_timeout = ros::Time::now();
        }
        
        sleep(1);
    }

    ros::shutdown();

    return 0;
}