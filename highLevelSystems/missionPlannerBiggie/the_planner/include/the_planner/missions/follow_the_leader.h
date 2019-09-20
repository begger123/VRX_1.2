#ifndef FOLLOW_THE_LEADER_H
#define FOLLOW_THE_LEADER_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class follow_the_leader : public Mission
{
public:
	follow_the_leader(ros::NodeHandle &nh);
	~follow_the_leader();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void follow_the_leader_callback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, FIND_AND_FOLLOW_LEADER, SECOND_SPIN, THIRD_SPIN, FINISHED};

private:
	ros::Publisher follow_the_leader_pub;
	ros::Subscriber follow_the_leader_sub;
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
	float angleDelta=0.34; //20 degrees
	bool blueBuoyFound=false;
	bool maxDistanceTraveled=false;
	float maxDistance;
	float nextPointX;
	float nextPointY;
	geometry_msgs::Point32 startSpeedBuoys[2];
	geometry_msgs::Point32 theLeader;
};

#endif
