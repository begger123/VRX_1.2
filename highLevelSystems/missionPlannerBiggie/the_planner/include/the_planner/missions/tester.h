#ifndef TESTER_H
#define TESTER_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class tester : public Mission
{
public:
	tester(ros::NodeHandle &nh);
	~tester();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, MOVETOSK, MOVETO, GETCURRSETHEAD, MOVEBY, MOVEBYHEADING, FINISHED};
	ros::ServiceClient traj_client;

private:
	ros::Publisher tester_publisher;
	ros::Subscriber tester_subscriber;
	void tester_callback(const std_msgs::Bool::ConstPtr& msg);
};

#endif
