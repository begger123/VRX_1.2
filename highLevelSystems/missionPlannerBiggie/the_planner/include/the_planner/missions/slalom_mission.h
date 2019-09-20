#ifndef SLALOM_MISSION_H
#define SLALOM_MISSION_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>

class slalom_mission : public Mission
{
public:
	slalom_mission(ros::NodeHandle &nh);
	~slalom_mission();
	void mission_callback(const std_msgs::String::ConstPtr& msg);
	void loop();
	//this needs to be updated to address the multitude of tasks
	enum Task{START, THERE, BACK, FINISHED};

private:
	ros::Publisher slalom_mission_pub;
	ros::Subscriber slalom_mission_sub;
	void slalom_mission_callback(const the_planner::waypoint_list::ConstPtr& msg);
	the_planner::waypoint_list waypoint_list_t;
	usv16_msgs::Usv16Mission traj_mission_t;
};

#endif
