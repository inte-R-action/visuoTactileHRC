#ifndef MOVEIT_ROBOT_H // include guard
#define MOVEIT_ROBOT_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
// #include <iostream>
// #include <unistd.h>
// #include <map>
// #include <sstream>
// #include "multimodal_tactile_custom_msgs/diagnostics.h"
// #include "multimodal_tactile_custom_msgs/object_state.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <ros/ros.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

class moveit_robot
{
    private:
        ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    public:
        // Setup
        // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
        // are used interchangably.
        const std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface move_group;

        // We will use the :planning_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group;

        // Now, we call the planner to compute the plan and visualize it.
        // The plan variable contains the movements that the robot will perform to move
        // from one point to another
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::core::RobotStatePtr current_state;
        std::vector<double> joint_group_positions;

        // Visualization
        // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
        // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
        moveit_visual_tools::MoveItVisualTools visual_tools;
        Eigen::Isometry3d text_pose;

        // Gripper message, cmd publisher and status subscriber
        string gripper_state;
        std_msgs::String  gripper_msg;
        ros::Publisher gripper_cmds_pub;
        ros::Subscriber gripper_feedback_sub;
        ros::Subscriber active_handover_sub;
        bool handover_active;

        int robot_execute_code;
        ros::Subscriber robot_execute_sub;

        std_msgs::String  robot_status_msg;
        ros::Publisher robot_status_pub;

        // Force sensor
        double ft_readings [6];
        ros::ServiceClient ft_client;
        ros::Subscriber ft_sub1;
        robotiq_ft_sensor::sensor_accessor ft_srv;

        // Constructor call
        moveit_robot(ros::NodeHandle* node_handle);

        // Method definitions
        bool plan_to_pose(geometry_msgs::Pose pose);
        void move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name);
        void open_gripper();
        void open_gripper_release();
        void close_gripper();
        void close_gripper_touch();
        void close_gripper_h2r_handover();
        void z_move(double dist, double max_velocity_scale_factor);
        geometry_msgs::Pose transform_pose(geometry_msgs::Pose input_pose);
        void gripperStatusCallback(const std_msgs::String::ConstPtr& msg);
        void ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg);
        void robotExecuteCallback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& msg);
        void handoverActiveCallback(const std_msgs::Bool::ConstPtr& msg);

        void sensor1Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void sensor2Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void sensor3Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        ros::Subscriber sensor1_sub;
        ros::Subscriber sensor2_sub;
        ros::Subscriber sensor3_sub;
};

#endif /*MOVEIT_ROBOT_H*/
