#ifndef NAVIGATION_CHANNEL_H
#define NAVIGATION_CHANNEL_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class navigation_channel : public Mission
{
public:
	navigation_channel(ros::NodeHandle &nh);
	~navigation_channel();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void check_order(geometry_msgs::Point32 startSpeedBuoys[2]);
	void navigation_channel_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void calculate_midpoint_and_heading(geometry_msgs::Point32 startSpeedBuoys[2]);
	void add_new_way_point();
	bool gateSetFound(geometry_msgs::Point32 startNavBuoys[2]);
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, GET_INITIAL_POINT, SEARCHING_FOR_NEXT, FINAL_POINT, FINISHED};

private:
	ros::Publisher speed_gate_mission_pub;
	ros::Subscriber navigation_channel_sub;
	the_planner::waypoint_list waypoint_list_t;
	sensor_msgs::PointCloud point_cloud_t;
	float midpointSlope;
	float midpointX;
	float midpointY;
	float midpointHeading;
	bool nextSetFound=false;
	bool maxDistanceTraveled=false;
	float maxDistance;
	float nextPointX;
	float nextPointY;
	float angleDelta=M_PI/2; //40 degrees
	geometry_msgs::Point32 startNavBuoys[2];
	geometry_msgs::Point32 endNavBuoys[2];
};

#endif
