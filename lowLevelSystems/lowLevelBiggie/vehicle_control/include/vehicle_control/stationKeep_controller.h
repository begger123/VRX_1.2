#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "custom_messages_biggie/control_effort.h"

// The following is to perform dynamic reconfigure of the PID gains
#include <dynamic_reconfigure/server.h>
#include <vehicle_control/pidSKGainsConfig.h>

namespace pid_controller
{
	class sk
	{
	public:
		sk(ros::NodeHandle &nh);
		~sk();
        void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
        void velo_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void get_theSKGains(vehicle_control::pidSKGainsConfig &config, uint32_t level);
        void compute_tau();
        void pub_control();
        void run();
        double piwrap(double angle);
    private:
        // ROS stuff...
        ros::NodeHandle *sk_nh;
        ros::Subscriber pose_sub;
        ros::Subscriber state_sub;
        ros::Publisher control_pub;
        ros::Time prev_time, prev_time2;
        ros::Time curr_time, curr_time2;
		ros::Duration delta_t, delta_t2;
		ros::Rate loop_rate;
        // dynamic_reconfigure stuff
        dynamic_reconfigure::Server<vehicle_control::pidSKGainsConfig> server;
        dynamic_reconfigure::Server<vehicle_control::pidSKGainsConfig>::CallbackType f;
        // Flags and variables
        bool initialized, newPose, newVelo, newControl;
        double Kp, Ki, Kd;
        double x, y, yaw_angle;
        double init_heading;

        // Linear algebra...
        Eigen::Vector3d eta_des;
        Eigen::Vector3d eta;
        Eigen::Vector3d eta_t;
        Eigen::Vector3d eta_td;
        Eigen::Vector3d tau;
        Eigen::Matrix3d Jt_prev;

    };
}
#endif            
