#ifndef ADD_DECAY_H
#define ADD_DECAY_H

#include <iostream>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

namespace vision
{
	class add_decay
	{
	public:
		add_decay(ros::NodeHandle &nh);
		~add_decay();
		void get_params();
		void fill_buffer();
		void pop_and_push();
		void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void point_cloud_pub();
		bool newPC2=false;
	private:
		//Message based data
		ros::NodeHandle *decay_nh;
		ros::Subscriber pc2_sub;
		ros::Publisher decayed_pc2_pub;

		//Params
		ros::Rate loop_rate;
		double decay_time;
		sensor_msgs::PointCloud2 super_pc2;
		std::deque<std::uint8_t> super_array;
		std::deque<std::uint32_t> width_array;

		int loop_counter = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds
		
	};

}

#endif
