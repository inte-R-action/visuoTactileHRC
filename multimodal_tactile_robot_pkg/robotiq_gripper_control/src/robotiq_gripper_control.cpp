/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 14-December-2020
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
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"
#include <unistd.h>
#include <map>
#include "sam_custom_messages/diagnostics.h"

using namespace std;


string gripper_action = "";

// global variable to hold the status of the gripper
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;

// callback function to get the status signals from the gripper
void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripperStatus = *msg;
/*
    if( msg->gOBJ == 0 )
   		ROS_INFO("ROBOT FINGERS MOVING");
   	else if( msg->gOBJ == 1 )
   		ROS_INFO("ROBOT FINGERS STOPPED DUE TO CONTACT DETECTED WHILE OPENING");
   	else if( msg->gOBJ == 2 )
   		ROS_INFO("ROBOT FINGERS STOPPED DUE TO CONTACT DETECTED WHILE CLOSING");
   	else if( msg->gOBJ == 3 )
   		ROS_INFO("ROBOT FINGERS AT THE REQUESTED POSITION");
   	else
   		ROS_INFO("ROBOT FINGERS ERROR");
*/
}

// callback function to get the status from the tactile sensor
void robotCommandStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    gripper_action.clear();
    gripper_action = msg->data;
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

    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    ros::Subscriber Robotiq2FGripperStatusPub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1000, gripperStatusCallback);

    ros::Publisher gripperStatusPub = node_handle.advertise<std_msgs::String>("Gripper2UR", 1000);
    ros::Subscriber robotStatusSub = node_handle.subscribe("UR2Gripper", 1000, robotCommandStatusCallback);

    ros::spinOnce();
    loop_rate.sleep();


    printf("==================================================\n");
    // reset the robotic gripper (needed to activate the robot)
    outputControlValues.rACT = 0;
    outputControlValues.rGTO = 0;
    //outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 0;
    outputControlValues.rFR = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "RESET GRIPPER" << std::endl;
    // give some time the gripper to reset
    sleep(3);

    // Activate Gripper
    outputControlValues.rACT = 1;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "ACTIVATE GRIPPER" << std::endl;
    sleep(2);


    int gripperSpeed = 10;
    int gripperForce = 255;//25;
    int gripperPosition = 255;

    outputControlValues.rGTO = 1;
    outputControlValues.rSP = gripperSpeed;
    outputControlValues.rFR = gripperForce;
    outputControlValues.rPR = gripperPosition;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "CLOSE GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
    while( gripperStatus.gOBJ != 3 && gripperStatus.gOBJ != 2 && ros::ok() )
    {
        //printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
        usleep(100000);
    }
    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

    outputControlValues.rGTO = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    sleep(1);

    // open the gripper to the maximum value of rPR = 0
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 255;
    outputControlValues.rFR = gripperForce;
    outputControlValues.rPR = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "OPEN GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
    while( gripperStatus.gOBJ != 3 && ros::ok() )
    {
        //printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
        usleep(100000);
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

    std_msgs::String msg;
    //std::stringstream ss;

    while( ros::ok() )
    {

//        ss.clear();
        msg.data = "gripper_ready";
//        ss << "gripper_ready";
//        msg.data = ss.str();
        gripperStatusPub.publish(msg);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        //std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);

        if( gripper_action == "grasp" )
        {
            // close the gripper to the maximum value of rPR = 255
            // rGTO = 1 allows the robot to perform an action
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = gripperSpeed;
            outputControlValues.rFR = gripperForce;
            outputControlValues.rPR = gripperPosition;

            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "CLOSE GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            while( gripperStatus.gOBJ != 3 && gripperStatus.gOBJ != 2 )
            {
                //printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
                usleep(100000);
            }

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

//            ss << "grasp_completed";
//            msg.data = ss.str();
            while (gripper_action != "completion acknowledged")
            {
                msg.data = "grasp_completed";
                gripperStatusPub.publish(msg);
            }
            printf("Acknowledge from UR received \n");

        }
        else if( gripper_action == "release" )
        {
            // open the gripper to the maximum value of rPR = 0
            // rGTO = 1 allows the robot to perform an action
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = 255;
            outputControlValues.rFR = gripperForce;
            outputControlValues.rPR = 0;

            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "OPEN GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            while( gripperStatus.gOBJ != 3 )
            {
                //printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
                usleep(100000);
            }

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

//            ss << "release_completed";
//            msg.data = ss.str();
            while (gripper_action != "completion acknowledged")
            {
                msg.data = "release_completed";
                gripperStatusPub.publish(msg);
            }
            printf("Acknowledge from UR received \n");
        }
        //else if ( gripper_action == "completion acknowledged" )
        //{
        //   printf("Acknowledge from UR received \n");
        //}
        else
        {
            // keep current configuration
        }

        if (ros::Time::now()-diag_timeout > ros::Duration(3))
        {
            diag_msg.DiagnosticStatus.level = 0; // 0:ok, 1:warning, 2:error, 3:stale
            diag_msg.DiagnosticStatus.message = "Ok";
            diag_msg.Header.stamp = ros::Time::now();
            diag_msg.Header.seq++;
            diag_obj.publish(diag_msg);
            diag_timeout = ros::Time::now();
            std::cout << "STANDBY GRIPPER" << std::endl;
        }
    }

    ros::shutdown();

    return 0;
}
