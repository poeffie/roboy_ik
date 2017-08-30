cmake_minimum_required(VERSION 2.8.3)
project(roboy_ik)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   cmake_modules
   moveit_core
   moveit_ros_planning
   moveit_ros_planning_interface
   pluginlib
   geometric_shapes
   moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(TinyXML REQUIRED)

## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package(  
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
    TinyXML
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${TinyXML_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Build
add_executable(motion src/motion.cpp)
target_link_libraries(motion ${catkin_LIBRARIES})

add_executable(traj_sub src/traj_sub.cpp)
target_link_libraries(traj_sub ${catkin_LIBRARIES})
