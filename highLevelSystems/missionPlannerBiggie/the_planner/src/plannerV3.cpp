#include <the_planner/highLevelPlanner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_planner");
	ros::NodeHandle nh;
	highLevelPlanner thePlanner(nh);

	ROS_INFO("Let's read and output all the mission data");
	thePlanner.set_mission_list();
	ROS_INFO("Now we take a brief pause before...");
	ros::Rate loopRate(.5);
	sleep(1);
	ROS_INFO("...starting our mission loop\n");
	while(ros::ok()&&!thePlanner.is_finished())
	{
		ROS_INFO("iteration");
		thePlanner.publish_mission();

		while(ros::ok()&&!thePlanner.mission_list[thePlanner.current_mission]->is_finished())
		{
			//ROS_INFO("internal iteration");
			thePlanner.mission_list[thePlanner.current_mission]->loop();
		}
	
		thePlanner.next_mission();
		loopRate.sleep();
	}
	return 0;
}
