/**
 *  @file    motion.cpp
 *  @author  Johannes Offner
 *  @date    8/28/2017
 *  @version 1.0
 *
 *  @brief Roboy 2.0, calculate Inverse Kinematics, send trajectory
 *
 *  @section DESCRIPTION
 *
 * This is publisher node, that calculates IK for "robot_description" if possible
 * and visualizes the movement in Rviz. Afterwards the joint trajectory will be published.
 *
 *
 */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "std_msgs/String.h"

#include <boost/scoped_ptr.hpp>

using namespace std;

int main(int argc, char** argv)
{
	
  // *************************************************** INITIALIZE***************************************************************
	
  ros::init(argc, argv, "motion");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // *************************************************** PLANNING INIT ***********************************************************
  
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
  
  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();
  
  // DEFINITIONS
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  planning_interface::PlanningContextPtr context;
  moveit_msgs::Constraints pose_goal;
  moveit_msgs::MotionPlanResponse response;
  moveit_msgs::DisplayTrajectory display_trajectory;
  
   // *************************************************** GET POSE ***************************************************************
  std::string poses[4];
  ifstream pose_file("/home/offi/catkin_ws/src/roboy_ik/src/pose.txt");
  int i = 0;
  if(!pose_file) 
  {
    cout<<"Error opening output file"<<endl;
    system("pause");
    return -1;
  }
  ROS_INFO("Goal Position (x, y, z):");
  while(!pose_file.eof())
  {
    getline(pose_file, poses[i], '\n');
    ROS_INFO("%s", poses[i].c_str());
  }
  
  // *************************************************** SET POSE ****************************************************************
  // Sample Pose: 0.03394;  -0.17347;   -0.37346;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.position.x = atof(poses[0].c_str());
  pose.pose.position.y = atof(poses[1].c_str());
  pose.pose.position.z = atof(poses[2].c_str());
  pose.pose.orientation.w = 1.0;;


  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.1);
  
  pose_goal = kinematic_constraints::constructGoalConstraints("pabi_legs__link_0_0", pose, tolerance_pose, tolerance_angle);
  req.group_name = "leg";
  req.goal_constraints.push_back(pose_goal);
  
  // ********************************************** CALCULATING IKFAST ***********************************************************
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  
  // ************************************************** VISUALIZE ****************************************************************
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the IK trajectory");
  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  
  sleep_time.sleep();
  ROS_INFO("Done");

  return 0;
}
