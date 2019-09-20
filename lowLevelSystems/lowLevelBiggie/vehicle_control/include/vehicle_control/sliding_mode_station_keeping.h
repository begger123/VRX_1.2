#ifndef SL_MO_SK_CO_H
#define SL_MO_SK_CO_H

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
#include <tf/tf.h>
#include <tf/transform_datatypes.h> //this gives axis to Matrix3x3
#include "custom_messages_biggie/control_target.h"
#include "custom_messages_biggie/control_effort.h"

namespace sm_controller
{
	class sl_mode_st_keep
	{
	public:
		sl_mode_st_keep(ros::NodeHandle &nh);
		~sl_mode_st_keep();
		void get_params();
		void target_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
		void state_callback(const nav_msgs::Odometry::ConstPtr& msg);
		double wrap_heading(double heading);
		void set_timestep();
		void calc_eta_t();
		void integrate_eta_t();
		Eigen::Vector3f sat_function(Eigen::Vector3f errorBoundTimesSurface);
		void calc_variales();
		void calc_tau();
		void pub();
		void run();
	private:
		//Message based data
		ros::NodeHandle *sm_sk_nh_;
		ros::Subscriber target_sub_;
		ros::Subscriber state_sub_;
		ros::Publisher control_effort_pub_;

		//controller variables
		//pose
		Eigen::Vector3f eta_;
		//desired pose
		Eigen::Vector3f eta_d_;
		//desired velocity
		Eigen::Vector3f eta_d_dot_;
		//pose error
		Eigen::Vector3f eta_t_;
		//velocity error
		Eigen::Vector3f eta_t_dot_;
		//pose error
		Eigen::Vector3f integral_eta_t_;

		//virutal reference trajectory
		Eigen::Vector3f eta_r_dot_;
		Eigen::Vector3f eta_r_dot_dot_;
		//surface
		Eigen::Vector3f s_;
		//control matrix
		Eigen::Matrix3f lamda_m3x3_;
		//uncertainty matrix
		Eigen::Matrix3f R_m3x3_;
		//error bounds
		Eigen::Matrix3f e_m3x3_;
		//control output
		Eigen::Vector3f tau_;

		//Hydrodynamic Parameters
		//Rigid body inertial terms
		double m_, Iz_;
		//Added mass inertial terms
		double Xu_dot_, Yv_dot_, Nr_dot_;
		//Drag terms
		double Xu_, Yv_, Nr_;
		//Control matrix
		double lamda_;
		//M C(v) and D
		Eigen::Matrix3f M_m3x3_, C_m3x3_, D_m3x3_;
		Eigen::Matrix3f J_, J_dot_;
		Eigen::Matrix3f J_trans_, J_trans_dot_;

		double r_kx_, r_ky_, r_kpsi_;
		double e_kx_, e_ky_, e_kpsi_;

		//Not sure what to classify these as yet
		ros::Rate loop_rate;
		nav_msgs::Odometry state_data_;
		double yaw_angle;
		ros::Time prev_time;
		ros::Duration delta_t;
		bool newCommand=false;
		bool newState=false;
		bool is_sim;
	};

}

#endif
