#ifndef TESTER_H
#define TESTER_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

typedef struct{
double north;
double east;
} NED_struct;

class tester : public Mission
{
public:
	tester(ros::NodeHandle &nh);
	~tester();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, STATIONKEEP, WAYPOINTS, FINISHED};
	ros::ServiceClient traj_client;

private:
	ros::Publisher sk_publisher;
	ros::Publisher tester_publisher;
	ros::Subscriber task_subscriber;
	ros::Subscriber station_keeping_subscriber;
	void task_callback(const vrx_gazebo::Task::ConstPtr& msg);
	void sk_goal_callback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg);
    vrx_gazebo::Task theTaskMsg;
    geographic_msgs::GeoPoseStamped skGoalMsg;
    bool newTask;
    bool skGoal;
    std::vector<float> datum;
    NED_struct nedPoint;
    NED_struct Geo2NED(double lat, double lon, double latref, double lonref);
    geometry_msgs::Pose2D skPoint;
};

#endif
