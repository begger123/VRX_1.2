#ifndef SCALED_UNDERACTUATED_ALLOCATION_H
#define SCALED_UNDERACTUATED_ALLOCATION_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <cmath>

#include "vehicle_allocation/global.h"
#include "vehicle_allocation/ctrl_types.h"
#include "custom_messages_biggie/control_effort.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

namespace alloc {

	class scaledUnderactuatedAllocationSim
	{
	public:
		scaledUnderactuatedAllocationSim(ros::NodeHandle &nh);
		~scaledUnderactuatedAllocationSim();
		Eigen::Vector4f allocate(const Eigen::Vector3f& ctrlForces);
		int run(void);
	private:
		ros::NodeHandle *alloc_nh;
		ros::Rate loop_rate;
		ros::Subscriber control_effort_sub;
		ros::Publisher actuation_pub;
		ros::Publisher sim_port_pub;
		ros::Publisher sim_stbd_pub;
		custom_messages_biggie::control_effort tau_msg;

		// allocation specific parameters
		tau_t tau_max_;		// maximum force produced by actuators on vehicle
		tau_t tau_min_;		// minimum force produced by actuators on vehicle
		double act_max_T_;	// maximum thrust from actuators
		double act_min_T_;	// minimum thrust from actuators
		double act_max_alpha_;	// maximum actuator angle from actuators
		double act_min_alpha_;	// minimum actuator angle from actuators
		double lx_;				// longitudinal separation of actuators from cg
		double ly_;				// lateral separation of actuators from cg
		Eigen::Matrix2f T_inv_;

		std_msgs::Bool isSim;

		void get_ros_params_(void);
		void publish_actuator_inputs_(const Eigen::Vector4f& u);
		void tau_callback_(const custom_messages_biggie::control_effort::ConstPtr& msg);
	};
}
#endif
