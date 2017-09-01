/**
 *  @file    traj_sub.cpp
 *  @author  Johannes Offner
 *  @date    8/28/2017
 *  @version 1.0
 *
 *  @brief Roboy 2.0, Inverse Kinematics, receive trajectory from ROS MoeveIt! and writes XML file
 *
 *  @section DESCRIPTION
 *
 * This is subscriber node, that listens to the specific ROS MoveIt topic, that contains
 * the trajectory data. This data contains a joint trajectory over several timesteps. The
 * callback function reads the values out of the topic structure and saves it into an 2D array.
 * This data is afterwards converted into an trajectory XML file, that is usable by CASPROS.
 * I use the library tinyXML2 for this to write XML files fast and memory efficient.
 *
 * NOTE: This code may be not running yet! I still have some include errors for the tinyXML2.h
 * But the generation of XML trajectoris and storing joint data into an array already works,
 * when executed by their own.
 *
 */

#include "ros/ros.h"
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iomanip>
#include <pluginlib/class_loader.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <fstream>
//#include <moveit/tiny_xml/tinyxml2.h>
#include <tinyxml.h>
#include <iomanip> // setprecision
#include <sstream> // stringstream

#define IS_TRAJECTORY 1

using namespace std;

#define NUM_JOINTS 6
#define NUM_WAYPOINTS 5

/**
*   @brief  Writes joint values into a new XML file, that is readable by CASPR / CASPROS.
*
*   @param  p_array is an pointer to the joint_traj_array
*   @param  combined_file represents the final XML file, that consists of two XML parts
*   @return void
*/

string get_joint_string(double q_array[NUM_WAYPOINTS][NUM_JOINTS], int wp){
    stringstream stream;
    for (int i = 0; i < NUM_JOINTS; i++){
		stream << q_array[wp][i];
		if (i != NUM_JOINTS-1) {
			stream << ", ";
		}
    }
    return stream.str();
}

void write_joint_trajectory_xml (double q_array[NUM_WAYPOINTS][NUM_JOINTS]) {
        int i = 0;

		ROS_INFO("IN CALLBACk222!!!");
        TiXmlDocument doc;

        TiXmlElement * root = new TiXmlElement( "trajectories" );
        doc.LinkEndChild( root );

        TiXmlElement * cxn = new TiXmlElement( "quintic_spline_trajectory" );
        root->LinkEndChild( cxn );
        cxn->SetAttribute("id", "traj_test2");
        cxn->SetAttribute("time_definition", "absolute");
        cxn->SetAttribute("time_step", "0.00667");

        TiXmlElement * pts = new TiXmlElement( "points" );
        cxn->LinkEndChild( pts );

        TiXmlElement * point = new TiXmlElement( "point" );
        pts->LinkEndChild( point );
        /**
         * In this state of development I have not yet implemented the conversion from array
         * data to joint strings. This will be the next step. So far, I just set all the joint
         * values to 0.
         */
        TiXmlElement * q = new TiXmlElement( "q" );
        point->LinkEndChild( q );
        TiXmlText * q_values = new TiXmlText( get_joint_string(q_array, i) );
        q->LinkEndChild( q_values );

        TiXmlElement * q_dot = new TiXmlElement( "q_dot" );
        point->LinkEndChild( q_dot );
        TiXmlText * q_dot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
        q_dot->LinkEndChild( q_dot_values );

        TiXmlElement * q_ddot = new TiXmlElement( "q_ddot" );
        point->LinkEndChild( q_ddot );
        TiXmlText * q_ddot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
        q_ddot->LinkEndChild( q_ddot_values );
        /**
         * If the trajectory consits of more than one point, this loop will be called.
         * The first point of a trajectory and the following ones differ by the point-attribute
         * time, that is set here.
         */
        if (IS_TRAJECTORY == 1)
        {
                for (double i = 1; i < NUM_WAYPOINTS; i++) {
                        TiXmlElement * point = new TiXmlElement( "point" );
                        pts->LinkEndChild( point );

                        TiXmlElement * q = new TiXmlElement( "q" );
                        point->LinkEndChild( q );
                        point->SetAttribute("time", to_string(i/10));
                        TiXmlText * q_values = new TiXmlText( get_joint_string(q_array, i)  );
                        q->LinkEndChild( q_values );

                        TiXmlElement * q_dot = new TiXmlElement( "q_dot" );
                        point->LinkEndChild( q_dot );
                        // TODO: replace "0.0 0.0 0.0 0.0 0.0 0.0" by get_joint_string(q_array, i)
                        // if you want to have full functionalty. This is just for experimental
                        // purposes, as the PaBiLegs CASPR model has 6 joints, instead of 2.
                        TiXmlText * q_dot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
                        q_dot->LinkEndChild( q_dot_values );

                        TiXmlElement * q_ddot = new TiXmlElement( "q_ddot" );
                        point->LinkEndChild( q_ddot );
                        // TODO: replace "0.0 0.0 0.0 0.0 0.0 0.0" by get_joint_string(q_array, i)
                        // if you want to have full functionalty. This is just for experimental
                        // purposes, as the PaBiLegs CASPR model has 6 joints, instead of 2.
                        TiXmlText * q_ddot_values = new TiXmlText( "0.0 0.0 0.0 0.0 0.0 0.0"  );
                        q_ddot->LinkEndChild( q_ddot_values );
                }
        }
        /**
         * tinyXML does not parse DOCTYPE elements. CASPR/OS needs a DOCTYPE declaration.
         * The doctype declaration never changes for any trajectory, so I created an XML file, called
         * head, containing the top of a trajectory XML file, that never changes.
         * trajo.xml is the part of the trajectory that contains the individual joint angles
         * we read from ROS MoveIt. Therefore it has to be generated seperately and combined
         * afterwards with the head.xml
         * trajo_combined.xml is the final usable trajectory file for CASPR / CASPROS
         */

        // TODO: change the file path for trajo.xml, head.xml, trajo_combined.xml as you wish.
        doc.SaveFile( "/home/offi/catkin_ws/src/roboy_ik/src/trajo.xml" );
        ifstream file1( "/home/offi/catkin_ws/src/roboy_ik/src/head.xml" ) ;
        ifstream file2( "/home/offi/catkin_ws/src/roboy_ik/src/trajo.xml" ) ;
        ofstream combined_file( "/home/offi/catkin_ws/src/roboy_ik/src/trajo_combined.xml" );
        combined_file << file1.rdbuf() << file2.rdbuf();
        
        ifstream  src("/home/offi/catkin_ws/src/roboy_ik/src/trajo_combined.xml", std::ios::binary);
        // TODO: change the file path to the CASPR model accordingly
        ofstream  dst("/home/offi/CASPR-master/data/model_config/models/PaBiLegs/PaBiLegs_trajectories.xml",   std::ios::binary);

        dst << src.rdbuf();
        ROS_INFO("Trajectory successfully created!");
}


void traj_sub_callback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    ROS_INFO("IN CALLBACk!!!");
    const trajectory_msgs::JointTrajectory traj = msg->trajectory[0].joint_trajectory;
    ROS_INFO("I heard: [%d]", traj.header.stamp.sec);
    int arrlen = sizeof(traj.points);
    ROS_INFO("SIZE: [%d]", arrlen);

    double joint_traj_array[NUM_WAYPOINTS][NUM_JOINTS];
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        for(int j = 0; j < NUM_JOINTS; j++) {
            // Put some values in
            if (j >= 2) {
				joint_traj_array[i][j] = 0.0;
			}
			else {
				joint_traj_array[i][j] = traj.points[i].positions[j];
			}
            ROS_INFO("Array value: [%f]", joint_traj_array[i][j]);
        }
        ROS_INFO(" ");
    }
    //double (p_array)[NUM_WAYPOINTS][NUM_JOINTS] = &joint_traj_array;
    write_joint_trajectory_xml(joint_traj_array);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscribe_trajectory");
  ros::NodeHandle n;
  ros::Subscriber traj_sub = n.subscribe("/move_group/display_planned_path", 1000, traj_sub_callback);
  ros::spin();
  return 0;
}
