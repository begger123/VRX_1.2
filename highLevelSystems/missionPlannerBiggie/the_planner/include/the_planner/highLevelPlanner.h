#ifndef HIGH_LEVEL_LIB_H
#define HIGH_LEVEL_LIB_H

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

//ROS messages
#include <the_planner/mission_out.h>

//Main Mission Class
#include <the_planner/Missions.h>

//Mission Classes
#include <the_planner/missions/tester.h>
#include <the_planner/missions/navigation_channel.h>
#include <the_planner/missions/acoustics_channel.h>
#include <the_planner/missions/buoy_field.h>
#include <the_planner/missions/totems.h>
//#include <the_planner/missions/listen_to_gui.h>
//#include <the_planner/missions/slalom_mission.h>
//#include <the_planner/missions/speed_gate_mission.h>
//#include <the_planner/missions/follow_the_leader.h>

//Container class for missions
class highLevelPlanner
{
public:
    highLevelPlanner(ros::NodeHandle &nh);
    ~highLevelPlanner();
	void hlp_callback(const std_msgs::String::ConstPtr& msg);
	void set_mission_list();
	void publish_mission();
	void next_mission();
	bool is_finished();

    int current_mission;
	int number_of_missions;
	Mission **mission_list;

private:
    ros::NodeHandle *hlp_nh_;
	ros::Subscriber hlp_subscriber;
	ros::Publisher hlp_publisher;

	std::vector<std::string> mission_string;
	
	double timeout;
	double begin_time;
	double time_remaining;

	bool finished;

};
#endif
