/**
 *  @file    roboy_ik_client.cpp
 *  @author  Johannes Offner
 *  @date    8/28/2017
 *  @version 1.0
 *
 *  @brief Roboy 2.0, calculate Inverse Kinematics, client
 *
 *  @section DESCRIPTION
 *
 * This is client node, that makes the position request.
 *
 *
 */
#include "ros/ros.h"
#include "roboy_ik/InverseKinematics.h"
#include <cstdlib>
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboy_ik_client");
  
  if (argc != 4)
  {
    ROS_INFO("usage: send position X Y Z");
    return 1;
  }
  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<roboy_ik::InverseKinematics>("roboy_ik");
  roboy_ik::InverseKinematics srv;
  srv.request.a = atof(argv[1]);
  srv.request.b = atof(argv[2]);
  srv.request.c = atof(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Inverse Kinematics Success: %s", (int)srv.response.sum ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to call inverse kinematics");
    return 1;
  }

  return 0;
}
