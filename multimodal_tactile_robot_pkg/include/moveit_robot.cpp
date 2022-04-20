#include "moveit_robot.h"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
// #include "std_msgs/String.h"
// #include <iostream>
// #include <unistd.h>
// #include <map>
// #include <sstream>
// #include "multimodal_tactile_custom_msgs/diagnostics.h"
// #include "multimodal_tactile_custom_msgs/object_state.h"
// #include "robotiq_ft_sensor/ft_sensor.h"
// #include "robotiq_ft_sensor/sensor_accessor.h"
// #include <ros/ros.h>
// #include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "joint_positions.h"

namespace rvt = rviz_visual_tools;
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

void moveit_robot::sensor1Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void moveit_robot::sensor2Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void moveit_robot::sensor3Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

moveit_robot::moveit_robot(ros::NodeHandle* node_handle) : nh_(*node_handle), PLANNING_GROUP("manipulator"), visual_tools("world"), move_group(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP)) {

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //Eigen::Isometry3d 
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "HRI Static Demo - v 0.1.0", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // 
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


    // Now, we call the planner to compute the plan and visualize it.
    // The plan variable contains the movements that the robot will perform to move
    // from one point to another
    //moveit::planning_interface::MoveGroupInterface::Plan plan;

    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.15);

    //joint_positions = create_joint_pos();
    gripper_state = "";
    gripper_cmds_pub = nh_.advertise<std_msgs::String>("UR2Gripper", 1);
    gripper_feedback_sub = nh_.subscribe("Gripper2UR", 1, &moveit_robot::gripperStatusCallback, this);
    robot_status_pub = nh_.advertise<std_msgs::String>("RobotStatus", 10);

    robot_execute_sub = nh_.subscribe("execute_trajectory/result", 1, &moveit_robot::robotExecuteCallback, this);

    ft_client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ft_sub1 = nh_.subscribe("robotiq_ft_sensor", 100, &moveit_robot::ftSensorCallback, this);

    // Add collision objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while ((planning_scene_diff_publisher.getNumSubscribers() < 1) and ros::ok())
    {
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene;
    //planning_scene.robot_state.attached_collision_objects.clear();
    //planning_scene.world.collision_objects.clear();
    //planning_scene_diff_publisher.publish(planning_scene);
    std::vector<std::string> object_ids;
    object_ids.push_back("gripper");
    object_ids.push_back("camera");
    object_ids.push_back("ground");
    planning_scene_interface.removeCollisionObjects(object_ids);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    //geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    // Add gripper object to robot
    moveit_msgs::AttachedCollisionObject gripper_object;
    gripper_object.link_name = "ee_link";
    gripper_object.object.header.frame_id = "ee_link";
    gripper_object.object.id = "gripper";
    shape_msgs::SolidPrimitive gripper_primitive;
    gripper_primitive.type = gripper_primitive.BOX;
    gripper_primitive.dimensions.resize(3);
    gripper_primitive.dimensions[0] = 0.3;
    gripper_primitive.dimensions[1] = 0.21;
    gripper_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose gripper_pose;
    gripper_pose.orientation.w = 0.0;
    gripper_pose.position.x = gripper_primitive.dimensions[0]/2;
    gripper_pose.position.y = 0.0;
    gripper_pose.position.z = 0.0;

    gripper_object.object.primitives.push_back(gripper_primitive);
    gripper_object.object.primitive_poses.push_back(gripper_pose);
    gripper_object.object.operation = gripper_object.object.ADD;

    //gripper_object.touch_links = std::vector<std::string>{ "ee_link"};
    
    //planning_scene.world.collision_objects.push_back(gripper_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);

    /* First, define the REMOVE object message*/
    //moveit_msgs::CollisionObject remove_gripper;
    //remove_gripper.id = "gripper";
    //remove_gripper.header.frame_id = "base_link";
    //remove_gripper.operation = remove_gripper.REMOVE;

    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the gripper to the robot");
    //planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(remove_gripper);
    planning_scene.robot_state.attached_collision_objects.push_back(gripper_object);
    //planning_scene_diff_publisher.publish(planning_scene);
    
    // Add camera object to robot
    moveit_msgs::AttachedCollisionObject camera_object;
    camera_object.link_name = "ee_link";
    camera_object.object.header.frame_id = "ee_link";
    camera_object.object.id = "camera";
    shape_msgs::SolidPrimitive camera_primitive;
    camera_primitive.type = camera_primitive.BOX;
    camera_primitive.dimensions.resize(3);
    camera_primitive.dimensions[0] = 0.04;
    camera_primitive.dimensions[1] = 0.09;
    camera_primitive.dimensions[2] = 0.06;

    geometry_msgs::Pose camera_pose;
    camera_pose.orientation.w = 0.0;
    camera_pose.position.x = 0.07;
    camera_pose.position.y = 0.0;
    camera_pose.position.z = (camera_primitive.dimensions[2]+gripper_primitive.dimensions[2])/2;

    camera_object.object.primitives.push_back(camera_primitive);
    camera_object.object.primitive_poses.push_back(camera_pose);
    camera_object.object.operation = camera_object.object.ADD;

    //camera_object.touch_links = std::vector<std::string>{ "ee_link", "gripper"};
    
    //planning_scene.world.collision_objects.push_back(camera_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);

    /* First, define the REMOVE object message*/
    //moveit_msgs::CollisionObject remove_camera;
    //remove_camera.id = "camera";
    //remove_camera.header.frame_id = "base_link";
    //remove_camera.operation = remove_camera.REMOVE;

    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the camera to the robot");
    //planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(remove_camera);
    planning_scene.robot_state.attached_collision_objects.push_back(camera_object);
    planning_scene_diff_publisher.publish(planning_scene);
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    //collision_objects.push_back(camera_object);

    //ROS_INFO_NAMED("tutorial", "Attach the camera to the robot");
    //move_group.attachObject(camera_object.id);

    // Add ground plane object to robot
    moveit_msgs::CollisionObject ground_object;
    ground_object.header.frame_id = move_group.getPlanningFrame();
    ground_object.id = "ground";
    shape_msgs::SolidPrimitive ground_primitive;
    ground_primitive.type = ground_primitive.BOX;
    ground_primitive.dimensions.resize(3);
    ground_primitive.dimensions[0] = 2;
    ground_primitive.dimensions[1] = 2;
    ground_primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -ground_primitive.dimensions[2];

    ground_object.primitives.push_back(ground_primitive);
    ground_object.primitive_poses.push_back(ground_pose);
    ground_object.operation = ground_object.ADD;

    collision_objects.push_back(ground_object);

    ROS_INFO_NAMED("tutorial", "Add ground into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    sensor1_sub = nh_.subscribe("icmData1", 100, &moveit_robot::sensor1Callback, this);
    sensor2_sub = nh_.subscribe("icmData2", 100, &moveit_robot::sensor2Callback, this);
    sensor3_sub = nh_.subscribe("icmData3", 100, &moveit_robot::sensor3Callback, this);

    active_handover_sub = nh_.subscribe("HandoverActive", 100, &moveit_robot::handoverActiveCallback, this);
    handover_active = false;
}

bool moveit_robot::plan_to_pose(geometry_msgs::Pose pose){
    bool success = false;
    move_group.setPoseTarget(pose);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("IK Robot", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("IK Robot", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(pose, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    if (success){
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
        move_group.move();
    }
    return success;
}

geometry_msgs::Pose moveit_robot::transform_pose(geometry_msgs::Pose input_pose){
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::Pose output_pose1;
  geometry_msgs::Pose output_pose2;
  geometry_msgs::TransformStamped transform1;
  geometry_msgs::TransformStamped transform2;

  while (ros::ok()){
    try{
      transform1 = tfBuffer.lookupTransform("world", "camera_frame",
                                 ros::Time(0));
    
      ROS_INFO("%s", transform1.child_frame_id.c_str());
      ROS_INFO("%f", transform1.transform.translation.x);
      ROS_INFO("%f", transform1.transform.translation.y);
      ROS_INFO("%f", transform1.transform.translation.z);
      ROS_INFO("%f", transform1.transform.rotation.x);
      ROS_INFO("%f", transform1.transform.rotation.y);
      ROS_INFO("%f", transform1.transform.rotation.z);
      ROS_INFO("%f", transform1.transform.rotation.w);

      tf2::doTransform(input_pose, output_pose1, transform1);

      ROS_INFO_STREAM("Input pose: \n" << input_pose);
      ROS_INFO_STREAM("Output pose1: \n" << output_pose1);

      return output_pose1;
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1).sleep();
    }
  }
}

void moveit_robot::move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name){
    
    if( joint_positions.count(jnt_pos_name) )
    {
        targetJoints.clear();
        targetJoints["shoulder_pan_joint"] = joint_positions[jnt_pos_name].angles[0]*3.1416/180;	// (deg*PI/180)
        targetJoints["shoulder_lift_joint"] = joint_positions[jnt_pos_name].angles[1]*3.1416/180;
        targetJoints["elbow_joint"] = joint_positions[jnt_pos_name].angles[2]*3.1416/180;
        targetJoints["wrist_1_joint"] = joint_positions[jnt_pos_name].angles[3]*3.1416/180;
        targetJoints["wrist_2_joint"] = joint_positions[jnt_pos_name].angles[4]*3.1416/180;
        targetJoints["wrist_3_joint"] = joint_positions[jnt_pos_name].angles[5]*3.1416/180;

        robot_status_msg.data = robot_action;
        robot_status_pub.publish(robot_status_msg);

        move_group.setStartState(*move_group.getCurrentState());

        move_group.setJointValueTarget(targetJoints);

        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing new move position plan (%.2f%% acheived)",success * 100.0);

        move_group.execute(plan);
    }
    else{
        cout << "Unrecognised joint_position key: " << jnt_pos_name << endl;
    }

}

void moveit_robot::open_gripper(){
    // Open Gripper
    gripper_msg.data = "release";
    while ((gripper_state != "release_completed") and ros::ok())
    {
        gripper_cmds_pub.publish(gripper_msg);
    }
    gripper_msg.data = "completion acknowledged";
    gripper_cmds_pub.publish(gripper_msg);
}

void moveit_robot::open_gripper_release(){ // check this part again
    // Open Gripper
   // sleep(2);
    robot_status_msg.data = "waiting_for_handover";
    robot_status_pub.publish(robot_status_msg);
    while ( (abs(gyro1_x) < 650 || abs(gyro1_y) < 650) or not handover_active ) {
    }  
    
    gripper_msg.data = "release";
    while ((gripper_state != "release_completed") and ros::ok() )
    {
           gripper_cmds_pub.publish(gripper_msg);
    }
    gripper_msg.data = "completion acknowledged";
    gripper_cmds_pub.publish(gripper_msg);
   
}

void moveit_robot::close_gripper(){
    // Close Gripper
    gripper_msg.data = "grasp";
    while ((gripper_state != "grasp_completed") and ros::ok())
    {
        gripper_cmds_pub.publish(gripper_msg);
    }
    gripper_msg.data = "completion acknowledged";
    gripper_cmds_pub.publish(gripper_msg);
}

void moveit_robot::close_gripper_h2r_handover(){ //check this part
    // Close Gripper
    robot_status_msg.data = "waiting_for_handover";
    robot_status_pub.publish(gripper_msg);
    while ((abs(gyro1_x) < 650 || abs(gyro1_y) < 650) or not handover_active ) {
    }

    gripper_msg.data = "grasp_touch";
    while ((gripper_state != "grasp_completed") and ros::ok())
    {
        gripper_cmds_pub.publish(gripper_msg);
    }

    gripper_msg.data = "completion acknowledged";
    gripper_cmds_pub.publish(gripper_msg);
}

void moveit_robot::close_gripper_touch(){
    // Close Gripper

    gripper_msg.data = "grasp_touch";
    while ((gripper_state != "grasp_completed") and ros::ok()) //(gripper_state != "grasp_completed") and )
    {
        gripper_cmds_pub.publish(gripper_msg);
    }
    gripper_msg.data = "completion acknowledged";
    gripper_cmds_pub.publish(gripper_msg);
}

void moveit_robot::z_move(double dist, double max_velocity_scale_factor){

    //--Cartesian movement planning for straight down movement--//
    // dist is -ve down, +ve up in m
    move_group.setStartState(*move_group.getCurrentState());

    geometry_msgs::Pose target_home = move_group.getCurrentPose().pose;

    geometry_msgs::Pose homeZPosition = move_group.getCurrentPose().pose;

    //move_group.setStartState(*move_group.getCurrentState());

    // Vector to store the waypoints for the planning process
    std::vector<geometry_msgs::Pose> waypoints;
    // Stores the first target pose or waypoint
    geometry_msgs::Pose target_pose3 = target_home;

    target_pose3.position.z = target_pose3.position.z + dist;
    waypoints.push_back(target_pose3);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described [here](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4).
    // Pull requests are welcome.

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
     
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    // double max_velocity_scale_factor;
    // if (dist < 0){
    //     max_velocity_scale_factor = 0.05;
    // }
    // else{
    //     max_velocity_scale_factor = 1;
    // }
    bool success = iptp.computeTimeStamps(rt, max_velocity_scale_factor);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    // Check trajectory_msg for velocities not empty
    //std::cout << trajectory << std::endl;

    plan.trajectory_ = trajectory;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
    //move_group.execute(plan);

    // You can execute a trajectory like this.
    robot_execute_code = 0;
    ft_srv.request.command_id = ft_srv.request.COMMAND_SET_ZERO;
    if(ft_client.call(ft_srv)){
        ROS_INFO("ret: %s", ft_srv.response.res.c_str());
        ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", ft_readings[0], ft_readings[1], ft_readings[2], ft_readings[3], ft_readings[4], ft_readings[5]);
    }
    std::cout << "Z move dist: " << dist << endl;
    if (dist < 0){
        move_group.asyncExecute(plan);
        double last = 0.0;
        while ((ft_readings[2] > -3) && (robot_execute_code != 1) && ros::ok())
        {
            //if (abs(ft_readings[2]) > abs(last)){
            //    last = ft_readings[2];
            //    std::cout << ft_readings[2] << endl;
            //}
        }
        move_group.stop();
        string execute_result = "unknown";
        if (robot_execute_code == 1){
            execute_result = "Complete";
        }
        else if (robot_execute_code == 0){
            execute_result = "Force Stop";
        }
        std::cout << ">> Robot code: " << robot_execute_code << " (" << execute_result << ")  Force: " << ft_readings[2] << "  Max: " << last << endl;

    }
    else{
        move_group.execute(plan);
    }

}

void moveit_robot::gripperStatusCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);

    if (msg->data != gripper_state)
    {
        gripper_state.clear();
        gripper_state = msg->data; 
        cout << "Gripper State: " << gripper_state << endl;
    }
}

void moveit_robot::ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
    double data [] = {msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz};
    std::copy(data, data + 6, ft_readings);
}

void moveit_robot::robotExecuteCallback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& msg)
{
    robot_execute_code = msg->result.error_code.val;
}

void moveit_robot::handoverActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    handover_active = msg->data;
}
