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
#include <geometry_msgs/Pose2D.h>
#include "custom_messages_biggie/control_target.h"
#include "custom_messages_biggie/control_effort.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h> //this gives axis to Matrix3x3

namespace pid_controller
{
	class pid
	{
	public:
		pid(ros::NodeHandle &nh);
		~pid();
		void target_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
		void state_callback(const nav_msgs::Odometry::ConstPtr& msg);
		void set_error_los();
		void set_error_ebs();
		double wrap_heading(double heading);
		void set_timestep();
		void integrate_error();
		void anti_windup();
		void anti_windup_ros_pid();
		void differentiate_error();
		void design_filter();
		void filter_error();
		void accumulate_control_effort();
		void pub();
		void get_params();
		void check_pid_gains();
		void run();
	private:
		//Message based data
		ros::NodeHandle *pid_nh;
		ros::Subscriber target_sub;
		ros::Subscriber state_sub;
		ros::Publisher control_effort_pub;
		nav_msgs::Odometry state_data;
		geometry_msgs::Point previous_waypoint;
		Eigen::Vector3f setpoint;
		float velocity_command;

		//Params
		ros::Rate loop_rate;
		double the_rate;
		double Kp_h, Ki_h, Kd_h;
		double Kp_v, Ki_v, Kd_v;
		double anti_windup_limit_h;
		double anti_windup_limit_v;
		// Cutoff frequency for the derivative calculation in Hz.
		// Negative -> Has not been set by the user yet, so use a default.
		//This value should revolve around the update rate of the sensor
		double cutoff_frequency_h=-1;
		double cutoff_frequency_v=-1;
		double heading_scalar;
		double velocity_scalar;
		std::vector<double> max_thrust=std::vector<double>(2); //Value in position 0 is forward thrust, value in position 1 is reverse_pub;

		//Not sure what to classify these as yet
		double yaw_angle;
		Eigen::Vector3f nedLocation;
		ros::Time prev_time;
		ros::Duration delta_t;
		// Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
		// 1/4 of the sample rate.
		double c_v=1.;
		double c_h=1.;
		// Used to check for tan(0)==>NaN in the filter calculation
		double tan_filt_h = 1.;
		double tan_filt_v = 1.;
		bool newCommand=false;
		bool newState=false;
		bool is_sim;

		//Heading calculation holders
		std::vector<double> heading_error=std::vector<double>(3);
		std::vector<double> filtered_heading_error=std::vector<double>(3);
		std::vector<double> heading_error_deriv=std::vector<double>(3);
		std::vector<double> filtered_heading_error_deriv=std::vector<double>(3);
		double integral_heading_error;
		double proportional_heading;
		double integral_heading;
		double derivative_heading;
		double control_effort_heading;
		bool resetOrigWaypoint=true;
		Eigen::Vector3f originalPoint;

		//Velocity calculation holders
		std::vector<double> velocity_error=std::vector<double>(3);
		std::vector<double> filtered_velocity_error=std::vector<double>(3);
		std::vector<double> velocity_error_deriv=std::vector<double>(3);
		std::vector<double> filtered_velocity_error_deriv=std::vector<double>(3);
		double integral_velocity_error;
		double proportional_velocity;
		double integral_velocity;
		double derivative_velocity;
		double control_effort_velocity;

		int loop_counter = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds

		float delta_x;
		float delta_y;
		float d;
		float g;
		float R;

		float a;
		float b;
		float c;

		float y_los, x_los;
		
	};

}

#endif
