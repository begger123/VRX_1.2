#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "custom_messages_biggie/waypoint_array.h"
#include <tf/tf.h>


//This class will take in an array from the high level planner that contains an array of waypoints.
//Each element in the array will be a geometry_msgs::Point
//The x and y elements will be considered as the east and north desired position
//The z element will be considered as the velocity
//The class will take in the array of waypoints and publish them to the controller one at a time.
//The waypoints will be iterated based on a maximum error radius
namespace waypoints_to_waypoint
{
	class ws2w
	{
	public:
		ws2w(ros::NodeHandle &nh);
		~ws2w();
		//Needed things
		//subscribe to the_planner to get array of waypoints
		void waypoint_array_callback(const custom_messages_biggie::waypoint_array::ConstPtr& msg);
		//subscribe to state
		void state_callback(const nav_msgs::Odometry::ConstPtr& msg);
		//some error checking about length of array, current position in array, structure of array
		int error_checking();
		//check if vehicle is at waypoint
		bool within_error_bound();
		//publish current waypoint as heading to controller
		void pub();
		//do the things
		void run();
	private:
		//Message based data
		ros::NodeHandle *ws2w_nh;
		ros::Subscriber waypoint_array_sub;
		ros::Subscriber state_sub;
		ros::Publisher control_target_pub;
		ros::Publisher mission_complete_pub;

		//Local storage of message data
		nav_msgs::Odometry state_data;
		geometry_msgs::Pose2D current_waypoint;
		custom_messages_biggie::waypoint_array the_array;

		bool newState=false;
		bool newWaypoint=false;
		int index=0;

		//Params
		ros::Rate loop_rate;
		double the_rate;
		float error_bound=4.0;
		float error_magnitude;
		std_msgs::Bool missionComplete;
		
	};

}

#endif
