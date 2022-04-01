/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 5-February-2021
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

//#include <moveit_core/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <map>
#include <sstream>
#include "multimodal_tactile_custom_msgs/diagnostics.h"
#include "multimodal_tactile_custom_msgs/object_state.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <ros/ros.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

string objectString = "";
multimodal_tactile_custom_msgs::object_state object_state_msg;
bool robotMove = false;
string gripper_state = "";
namespace rvt = rviz_visual_tools;
int robot_execute_code;
double ft_readings [6];
bool vision_recog = false;

void ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
    double data [] = {msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz};
    std::copy(data, data + 6, ft_readings);
}

void robotMoveCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);
 
 	//objectString.clear();

    objectString = msg->data; 

    //cout << "Robot move: " << robotMove << endl; 
    //cout << "Object: " << objectString << endl;
}

void gripperStatusCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);

    if (msg->data != gripper_state)
    {
        gripper_state.clear();
        gripper_state = msg->data; 
        cout << "Gripper State: " << gripper_state << endl;
    }
}

void robotExecuteCallback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& msg)
{
    robot_execute_code = msg->result.error_code.val;
}

void objectDetectionCallback(const multimodal_tactile_custom_msgs::object_state::ConstPtr &msg)
{
    object_state_msg.Object = msg->Object;
    object_state_msg.Pose = msg->Pose;
    object_state_msg.Header = msg->Header;
}

// Map high level position to joint angles as seen on teach pendant
// Order is: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
struct jnt_angs{double angles[6];};
std::map<std::string, jnt_angs> create_joint_pos(){
    std::map<std::string, jnt_angs> joint_positions;
    joint_positions["home"] = {-11.75, -83.80, 47.90, -125.0, -90.0, 0.0};
    joint_positions["bring_side_1"] = {54.1, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["bring_side_2"] = {80.9, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["bring_side_3"] = {119.3, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["bring_side_4"] = {154.5, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["take_box"] = {14.5, -45.90, 12.2, -61.2, -91.1, 0.0};
    joint_positions["deliver_2_user"] = {14.5, -45.90, 12.2, -61.2, -90.0, 0.0};
    joint_positions["deliver_big_2_user"] = {17.4, -68.3, 35.9, -60.5, -91.0, 0.0};
    joint_positions["deliver_box"] = {130.0, -62.90, 40.2, -61.2, -91.1, 0.0};
    joint_positions["stack_red_small_block"] = {-110.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["stack_blue_small_block"] = {-130.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["stack_green_small_block"] = {-150.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["stack_yellow_small_block"] = {-90.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["look_for_objects"] = {-110.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["final_stack"] = {-20.0, -75.9, 49.1, -61.2, -91.1, 0.0};
    joint_positions["remove_stack"] = {-170.0, -75.9, 49.1, -61.2, -91.1, 0.0};
    joint_positions["bring_hand_screw_parts"] = {-65.0, -75.9, 49.1, -61.2, -90.0, 0.0};
    joint_positions["bring_seat_top"] = {54.1, -79.0, 41.0, -52.4, -90, 0.0};
    joint_positions["bring_back_frame"] = {80.9, -79.0, 41.0, -52.4, -90, 0.0};
    joint_positions["bring_back_slats"] = {119.3, -75.9, 49.1, -61.2, -90, 0.0};
    joint_positions["bring_legs"] = {-65.0, -75.9, 49.1, -61.2, -90, 0.0};
    joint_positions["take_chair"] = {17.4, -68.3, 35.9, -60.5, -90, 0.0};
    joint_positions["deliver_chair"] = {130.0, -63.8, 20.1, -46.8, -90, 0.0};
    return joint_positions;
};

void pick_up_object(moveit_robot &Robot, double down_move_dist = 0.05)
{
    // Robot moves down, grasps part and moves back to original position
    Robot.open_gripper();
    Robot.z_move(-down_move_dist, 0.8);
    Robot.close_gripper();
    Robot.z_move(down_move_dist, 1);
}

void set_down_object(moveit_robot &Robot, double down_move_dist = 0.03, double max_velocity_scale_factor = 1)
{
    // Robot moves down, grasps part and moves back to original position
    Robot.z_move(-down_move_dist, max_velocity_scale_factor);
    Robot.open_gripper();
    Robot.z_move(down_move_dist, 1.0);
}

void home(std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    string bring_cmd = "home";

    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);
}

void bring_part_to_user(string bring_cmd, std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    // Move to position above part
    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);

    // Move down, pick part up, move up
    if( bring_cmd=="bring_side_1" || bring_cmd=="bring_side_2" || \
        bring_cmd=="bring_side_3" || bring_cmd=="bring_side_4" || \
        bring_cmd=="bring_back_slats" )
    {          
        pick_up_object(Robot, 0.07);
        // Move to user delivery position
        Robot.move_robot(targetJoints, bring_cmd, string("deliver_2_user"));
        // Move down, set down part, move up
        set_down_object(Robot, 0.03, 0.5);
    }
    else if( bring_cmd=="bring_hand_screw_parts" || bring_cmd=="bring_legs" )
    {          
        pick_up_object(Robot, 0.065);
        // Move to user delivery position
        Robot.move_robot(targetJoints, bring_cmd, string("deliver_2_user"));
        // Move down, set down part, move up
        set_down_object(Robot, 0.03, 0.5);
    }
    else if( bring_cmd=="bring_seat_top" || bring_cmd=="bring_back_frame" )
    {
        pick_up_object(Robot, 0.07);
        // Move to user delivery position
        Robot.move_robot(targetJoints, bring_cmd, string("deliver_big_2_user"));
        // Move down, set down part, move up
        set_down_object(Robot, 0.04, 0.5);
    }
    else
    {
        cout << "Unrecognised bring part command: " << bring_cmd << endl;
        return;
    }

    

    // Return to home position
    //home(targetJoints, Robot);
}

void take_assembly(string take_cmd, std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    // Move robot to position above assembly
    Robot.move_robot(targetJoints, take_cmd, take_cmd);

    if( take_cmd == "take_box" )
    {
        // Move down, pick assembly up, move up
        pick_up_object(Robot, 0.07);
        // Move to position
        string deliver_cmd = "deliver_box";
        Robot.move_robot(targetJoints, deliver_cmd, deliver_cmd);
        // Move down, set down assembly, move up
        set_down_object(Robot, 0.03, 0.5);
    }
    else if ( take_cmd == "take_chair" )
    {
        // Move down, pick assembly up, move up
        pick_up_object(Robot, 0.07);
        // Move to position
        string deliver_cmd = "deliver_chair";
        Robot.move_robot(targetJoints, deliver_cmd, deliver_cmd);
        // Move down, set down assembly, move up
        set_down_object(Robot, 0.087, 0.5);
    }
    else
    {
        cout << "Unrecognised take assembly command: " << take_cmd << endl;
        return;
    }
    // Return to home
    //home(targetJoints, Robot);
}

geometry_msgs::Pose look_for_objects(string bring_cmd)
{
    // wait for message received?
    geometry_msgs::Pose object_pose;

    std::string block = bring_cmd.substr(6);
    cout << "Looking for: " << block << endl;
    while(ros::ok()){
        cout << "Looking for: " << block << " detected: " << object_state_msg.Object.Info << endl;
        if (block == object_state_msg.Object.Info){
           object_pose = object_state_msg.Pose;
           return object_pose;
        }
        else{
            ros::Duration(1).sleep();
        }
    }
    // wait for message received?
    //geometry_msgs::Pose object_pose;
    //object_pose.orientation.w = 1.0;
    //object_pose.position.x = 0.05;
    //object_pose.position.y = 0.1;
    //object_pose.position.z = -0.1;
    //return object_pose;
}

void stack_blocks(string bring_cmd, std::map<std::string, double> &targetJoints, moveit_robot &Robot, int stack_height)
{
    // Move to position above block
    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);

    bool success = false;
    if ( vision_recog ){
        // Look for block
        geometry_msgs::Pose pose_cam_obj = look_for_objects(bring_cmd);
        cout << "Found: " << pose_cam_obj << endl;
        // Transform to world frame
        geometry_msgs::Pose pose_base_obj = Robot.transform_pose(pose_cam_obj);
        // Move to new position above object
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.x = pose_base_obj.orientation.x;
        target_pose1.orientation.y = pose_base_obj.orientation.y;
        target_pose1.orientation.z = pose_base_obj.orientation.z;
        target_pose1.orientation.w = pose_base_obj.orientation.w;
        target_pose1.position.x = pose_base_obj.position.x;
        target_pose1.position.y = pose_base_obj.position.y;
        target_pose1.position.z = 0.4;
        ROS_INFO_STREAM("Target pose: \n" << target_pose1);
        success = Robot.plan_to_pose(target_pose1);
    }
    else {
        success = true;
    }

    if (success){
        //Robot.move_group.execute();
        //Robot.move_group.move();

        // Move down, pick block up, move up
        pick_up_object(Robot, 0.11);

        // Move to stack position
        Robot.move_robot(targetJoints, bring_cmd, string("final_stack"));

        // Move down, set down block, move up
        double block_heght = 0.019;
        double z_move = 0.11 - (stack_height*block_heght);
        Robot.z_move(-(z_move-block_heght), 0.05);
        Robot.z_move(-block_heght, 0.01);
        Robot.open_gripper();
        Robot.z_move(z_move, 1.0);
    }
    else {
        cout << "Failed to perform IK plan" << endl;
    }
  // Return to home position
    //home(targetJoints, Robot);
}

void remove_blocks(std::map<std::string, double> &targetJoints, moveit_robot &Robot, int stack_height)
{
    // Move robot to position above box
    string bring_cmd = "final_stack";
    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);

    // Move down, pick side up, move up
    if (stack_height < 1){
        stack_height = 1;
    }
    double z_move = 0.11 - ((stack_height-1)*0.019);
    pick_up_object(Robot, z_move);

    // Move to position
    bring_cmd = "remove_stack";
    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);

    // Move down, set down side, move up
    set_down_object(Robot, z_move, 0.05);

    // Return to home
    //home(targetJoints, Robot);
}


int main(int argc, char** argv)
{
    // Set up ROS stuff
    string frame_id = "hri_static_demo";
    ros::init(argc, argv, frame_id);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Diagnostics status publisher
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

    // High level move commands subscriber
	ros::Subscriber subRobotPosition = node_handle.subscribe("RobotMove", 1000, robotMoveCallback);

    // Object recognition subscriber
    if ( vision_recog ){
        ros::Subscriber subObjectRecog = node_handle.subscribe("ObjectStates", 1000, objectDetectionCallback);
    }

    // Robot object
    moveit_robot Robot(&node_handle);
    // Map to hold values to send to robot
    std::map<std::string, double> targetJoints;
    
    string last_obj_string = "";

    // Send robot to home position
    home(targetJoints, Robot);

    // wait position
    Robot.robot_status_msg.data = "Done";
    Robot.robot_status_pub.publish(Robot.robot_status_msg);
    cout << ">>>>-- Waiting for command --<<<<" << endl;

    int stack_height = 0;
    while( ros::ok() )
    {
            targetJoints.clear();

            // Ignore repeat requests
            if (objectString != last_obj_string)
            {
                cout << "Robot Objective: " << objectString << endl;
                last_obj_string = objectString;

                if ( objectString != "")
                {
                    //if ( objectString != "home")
                    //{
                    Robot.robot_status_msg.data = objectString;
                    Robot.robot_status_pub.publish(Robot.robot_status_msg);
                    //}

                    if( objectString.rfind("bring_", 0) == 0 )
                    {          
                        bring_part_to_user(objectString, targetJoints, Robot);
                    }
                    else if( objectString.rfind("take_", 0) == 0 )
                    {  
                        take_assembly(objectString, targetJoints, Robot);
                    }
                    else if( objectString.rfind("stack_", 0) == 0 )
                    { 
                        stack_blocks(objectString, targetJoints, Robot, stack_height);
                        stack_height++;
                    }
                    else if( objectString == "remove_stack" )
                    {  
                        remove_blocks(targetJoints, Robot, stack_height);
                        stack_height = 0;
                    }
                    else if( objectString == "home" )
                    {
                        //Robot.robot_status_msg.data = "home";
                        //Robot.robot_status_pub.publish(Robot.robot_status_msg);
                        home(targetJoints, Robot);
                    }
                    Robot.robot_status_msg.data = "Done";
                    Robot.robot_status_pub.publish(Robot.robot_status_msg);
                }
                
                // wait position
                Robot.robot_status_msg.data = "Waiting";
                Robot.robot_status_pub.publish(Robot.robot_status_msg);
                cout << ">>>>-- Waiting for command --<<<<" << endl;
            }


            if (ros::Time::now()-diag_timeout > ros::Duration(3))
            {
                diag_msg.DiagnosticStatus.level = 0; // 0:ok, 1:warning, 2:error, 3:stale
                diag_msg.DiagnosticStatus.message = "Ok";
                diag_msg.Header.stamp = ros::Time::now();
                diag_msg.Header.seq++;
                diag_obj.publish(diag_msg);
                diag_timeout = ros::Time::now();
            } 
    }
    ros::shutdown();
    return 0;
}
