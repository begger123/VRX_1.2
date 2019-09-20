#ifndef SPEED_GATE_MISSION_H
#define SPEED_GATE_MISSION_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class speed_gate_mission : public Mission
{
public:
	speed_gate_mission(ros::NodeHandle &nh);
	~speed_gate_mission();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void check_order();
	bool gateSetFound();
	void speed_gate_mission_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void calculate_midpoint_and_heading();
	void add_new_way_point();
	usv16_msgs::Usv16Mission calculate_four_waypoints();
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, GET_INITIAL_POINT, SEARCHING_FOR_BLUE, BLUE_FOUND, HOMEWARD_BOUND, FINISHED};

private:
	ros::Publisher speed_gate_mission_pub;
	ros::Subscriber speed_gate_mission_sub;
	the_planner::waypoint_list waypoint_list_t;
	usv16_msgs::Usv16Mission traj_mission_t;
	sensor_msgs::PointCloud point_cloud_t;
	geometry_msgs::Point32 blueBuoy;
	usv_ahc_py::buoy redBuoy;
	usv_ahc_py::buoy greenBuoy;
	float midpointSlope;
	float midpointX;
	float midpointY;
	float midpointHeading;
	float angleDelta=1.57; //20 degrees
	bool blueBuoyFound=false;
	bool maxDistanceTraveled=false;
	float maxDistance;
	float nextPointX;
	float nextPointY;
	geometry_msgs::Point32 startSpeedBuoys[2];
};

#endif
