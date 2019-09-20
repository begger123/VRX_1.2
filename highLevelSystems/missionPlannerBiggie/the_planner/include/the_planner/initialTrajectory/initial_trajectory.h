#ifndef INITIAL_TRAJ_H
#define INITIAL_TRAJ_H

#include <iostream>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <ros/console.h>
#include <the_planner/initTraj.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <the_planner/line.h>
#include <the_planner/selectedCSP.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace high_level
{
	class gen_trajectory
	{
	public:
		gen_trajectory(ros::NodeHandle &nh);
		~gen_trajectory();
		void get_params();
		void state_callback(const nav_msgs::Odometry::ConstPtr& msg);
		bool generate(the_planner::initTraj::Request  &req, the_planner::initTraj::Response &res);
		void define_lines();
		the_planner::line headingDist(geometry_msgs::Point pointA, geometry_msgs::Point pointB, double wpSpace);
		void set_csp();
		void generate_trajectory();
		void gen_segment(geometry_msgs::Pose2D anchorPoint, geometry_msgs::Pose2D endPoint, double wpSpace);
	private:
		//Message based data
		ros::NodeHandle *init_traj_nh;
		ros::Subscriber state_sub;

		//Params
		ros::Rate loop_rate;
		geometry_msgs::Point pointA;
		geometry_msgs::Point pointB;
		geometry_msgs::Point pointC;
		geometry_msgs::Point pointD;
		double RoD;
		double wpSpace;

		nav_msgs::Odometry state_data;

		//Computational Variables
		the_planner::line lineA;
		the_planner::line lineB;
		the_planner::line lineC;
		the_planner::line lineD;
		geometry_msgs::Pose2D CCSP[4];
		the_planner::selectedCSP CSP;
		geometry_msgs::Pose2D vehicle;
		std::vector<geometry_msgs::Pose2D> waypoints;
		double theMin;
		int lineNum;
		geometry_msgs::Pose2D anchorPoint;
		geometry_msgs::Pose2D endPoint;
		std::vector<geometry_msgs::Pose2D> segment;
		std::vector<geometry_msgs::Pose2D> fullSegment;
    	bool eastWestFlag=false;
    	bool westEastFlag=false;


		bool newState = false;

		int loop_counter = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds
		
	};

}

#endif
