#ifndef BUOY_FIELD_H
#define BUOY_FIELD_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class buoy_field : public Mission
{
public:
	buoy_field(ros::NodeHandle &nh);
	~buoy_field();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void buoy_field_callback();
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, THROUGH_THE_FIELD, FINISHED};

private:
	ros::Publisher buoy_field_pub;
	ros::Subscriber buoy_field_sub;
};

#endif
