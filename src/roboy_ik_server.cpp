/**
 *  @file    roboy_ik_server.cpp
 *  @author  Johannes Offner
 *  @date    8/28/2017
 *  @version 1.0
 *
 *  @brief Roboy 2.0, calculate Inverse Kinematics, server
 *
 *  @section DESCRIPTION
 *
 * This is the server node, the initializing part for inverse kinematics calulations. 
 * The server works together with a so called client, which build together a ROS service.
 *
 *
 */
#include "ros/ros.h"
#include "roboy_ik/InverseKinematics.h"
#include "std_msgs/String.h"

#include <iostream>
#include <fstream>
using namespace std;
#include <iomanip>
#include <sstream>

bool call_ik(roboy_ik::InverseKinematics::Request  &req,
         roboy_ik::InverseKinematics::Response &res)
{
  if((req.a + req.b + req.c) < 10000.0) {
  	res.sum = true;
  	ofstream myfile;
    myfile.open ("/home/offi/catkin_ws/src/roboy_ik/src/pose.txt");
    myfile << std::fixed << std::setprecision(5) << req.a <<endl;
    myfile << std::fixed << std::setprecision(5) << req.b <<endl;
    myfile << std::fixed << std::setprecision(5) << req.c;
    myfile.close();
  }
  else {
	res.sum = false;
  }
  ROS_INFO("request: x=%lf, y=%lf, z=%lf", (double)req.a, (double)req.b, (double)req.c);
  ROS_INFO("sending back response: [%d]", (bool)res.sum);
  if(res.sum == true) {
    system("/home/offi/catkin_ws/src/roboy_ik/src/run.sh");
  }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboy_ik_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("roboy_ik", call_ik);
  ROS_INFO("Ready to send position request.");
  ros::spin();

  return 0;
}
