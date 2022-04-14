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
#include "multimodal_tactile_custom_msgs/diagnostics.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

// global variable to hold the status of the tactile sensor
bool touchDetected = false, touchDetected2 = false, touchDetected3 = false;

// global variable to start the data storing
bool ready_to_write = false;

// global variable to hold the gyroscope data of three tactile sensors
float gyro1_x, gyro1_y, gyro1_z, gyro2_x, gyro2_y, gyro2_z, gyro3_x, gyro3_y, gyro3_z; 
float accel1_x, accel1_y, accel1_z, accel2_x, accel2_y, accel2_z, accel3_x, accel3_y, accel3_z; 
float pressure1, pressure2, pressure3;

bool ref_val = false;

float base_val1 = 0.0, base_val2 = 0.0, base_val3 = 0.0;
float Last_val1[2], Last_val2[2], Last_val_kalmanx1[2], Last_val3[2];

void sensor1Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{    
     accel1_x = msg->data[0]; // ROS_INFO("I heard Sensor1: [%f]", msg->x);
     accel1_y = msg->data[1];
     accel1_z = msg->data[2];
     gyro1_x = msg->data[3];
     gyro1_y = msg->data[4];
     gyro1_z = msg->data[5];
     pressure1 = msg->data[6];
     touchDetected2 = msg->data[7];
}

void sensor2Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
     accel2_x = msg->data[0]; // ROS_INFO("I heard Sensor2: [%f]", msg->x);
     accel2_y = msg->data[1];
     accel2_z = msg->data[2];
     gyro2_x = msg->data[3];
     gyro2_y = msg->data[4];
     gyro2_z = msg->data[5];
     pressure2 = msg->data[6];
     touchDetected = msg->data[7];   
}

void sensor3Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
     accel3_x = msg->data[0];
     accel3_y = msg->data[1];
     accel3_z = msg->data[2];
     gyro3_x = msg->data[3];
     gyro3_y = msg->data[4];
     gyro3_z = msg->data[5];
     pressure3 = msg->data[6];
     touchDetected3 = msg->data[7];
}

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

    ros::Publisher diag_obj = node_handle.advertise<multimodal_tactile_custom_msgs::diagnostics>("SystemStatus", 10);
    multimodal_tactile_custom_msgs::diagnostics diag_msg;
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

    ros::Subscriber sensor1_sub = node_handle.subscribe("icmData1", 100, sensor1Callback);
    ros::Subscriber sensor2_sub = node_handle.subscribe("icmData2", 100, sensor2Callback);
    ros::Subscriber sensor3_sub = node_handle.subscribe("icmData3", 100, sensor3Callback);

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
    int gripperForce = 10;//25;
    int gripperPosition = 200;

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
         else if (gripper_action == "grasp_touch")
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
            while( gripperStatus.gOBJ != 3 && gripperStatus.gOBJ != 2 && touchDetected == false)
            {
                //printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
                usleep(100000);
            }

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

            // set gripper to standby to clear the flags
            outputControlValues.rGTO = 0;
            outputControlValues.rPR = 0;
            outputControlValues.rSP = 0;
            outputControlValues.rFR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);

            while (gripper_action != "completion acknowledged")
            {
                msg.data = "grasp_completed";
                gripperStatusPub.publish(msg);
            }
            printf("Acknowledge from UR received \n");

        }   
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
