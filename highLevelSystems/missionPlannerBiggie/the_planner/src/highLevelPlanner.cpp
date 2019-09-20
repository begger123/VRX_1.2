#include <the_planner/highLevelPlanner.h>
///////////////////////////////////high_level_planner///////////////////////////////////
highLevelPlanner::highLevelPlanner(ros::NodeHandle &nh) : hlp_nh_(&nh)
{
	hlp_subscriber = hlp_nh_->subscribe("/mission_out", 1, &highLevelPlanner::hlp_callback, this);
	hlp_publisher = hlp_nh_->advertise<std_msgs::String>("hlp_out", 10);
	current_mission=0;
}

highLevelPlanner::~highLevelPlanner()
{
	delete[] mission_list;
}

void highLevelPlanner::hlp_callback(const std_msgs::String::ConstPtr& msg)
{

}

void highLevelPlanner::set_mission_list()
{
	hlp_nh_->getParam("planner/mission_string", mission_string);
	number_of_missions = mission_string.size();

	//this function will get the pertinent data from missions.yaml to create a class based
	//on the order of missions set in mission string
	mission_list = new Mission*[number_of_missions];
	for(int i=0; i<number_of_missions; i++)
	{
		ROS_INFO("Mission number: %i is: %s", i, mission_string[i].c_str());
		if(mission_string[i]=="tester")
		{
			mission_list[i] = new tester(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/tester/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/tester/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/tester/north", mission_list[i]->north);
			hlp_nh_->getParam("planner/missions/tester/east", mission_list[i]->east);
			hlp_nh_->getParam("planner/missions/tester/heading", mission_list[i]->heading);
		}
		else if(mission_string[i]=="navigation_channel")
		{
			mission_list[i] = new navigation_channel(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/navigation_channel/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/navigation_channel/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/navigation_channel/north", mission_list[i]->north);
			hlp_nh_->getParam("planner/missions/navigation_channel/east", mission_list[i]->east);
			hlp_nh_->getParam("planner/missions/navigation_channel/heading", mission_list[i]->heading);
		}
		else if(mission_string[i]=="acoustics_channel")
		{
			mission_list[i] = new acoustics_channel(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/acoustics_channel/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/acoustics_channel/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/acoustics_channel/north", mission_list[i]->north);
			hlp_nh_->getParam("planner/missions/acoustics_channel/east", mission_list[i]->east);
			hlp_nh_->getParam("planner/missions/acoustics_channel/heading", mission_list[i]->heading);
		}
        else if(mission_string[i]=="buoy_field")
		{
			mission_list[i] = new buoy_field(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/buoy_field/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/buoy_field/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/buoy_field/north", mission_list[i]->north);
			hlp_nh_->getParam("planner/missions/buoy_field/east", mission_list[i]->east);
			hlp_nh_->getParam("planner/missions/buoy_field/heading", mission_list[i]->heading);
		}
		else if (mission_string[i] == "totems") 
        {
            mission_list[i] = new totems(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/totems/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/totems/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/totems/north", mission_list[i]->north);
			hlp_nh_->getParam("planner/missions/totems/east", mission_list[i]->east);
			hlp_nh_->getParam("planner/missions/totems/heading", mission_list[i]->heading);
        }
		/*else if(mission_string[i]=="listen_to_gui")
		{
			mission_list[i] = new listen_to_gui(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/listen_to_gui/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/listen_to_gui/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/listen_to_gui/lat", mission_list[i]->lat);
			hlp_nh_->getParam("planner/missions/listen_to_gui/lon", mission_list[i]->lon);
			hlp_nh_->getParam("planner/missions/listen_to_gui/heading", mission_list[i]->heading);
		}
		else if(mission_string[i]=="slalom_mission")
		{
			mission_list[i] = new slalom_mission(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/slalom_mission/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/slalom_mission/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/slalom_mission/lat", mission_list[i]->lat);
			hlp_nh_->getParam("planner/missions/slalom_mission/lon", mission_list[i]->lon);
			hlp_nh_->getParam("planner/missions/slalom_mission/heading", mission_list[i]->heading);
		}
		else if(mission_string[i]=="speed_gate_mission")
		{
			mission_list[i] = new speed_gate_mission(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/speed_gate_mission/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/speed_gate_mission/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/speed_gate_mission/lat", mission_list[i]->lat);
			hlp_nh_->getParam("planner/missions/speed_gate_mission/lon", mission_list[i]->lon);
			hlp_nh_->getParam("planner/missions/speed_gate_mission/heading", mission_list[i]->heading);
		}
		else if(mission_string[i]=="follow_the_leader")
		{
			mission_list[i] = new follow_the_leader(*hlp_nh_);
			hlp_nh_->getParam("planner/missions/follow_the_leader/name", mission_list[i]->mission_name);
			hlp_nh_->getParam("planner/missions/follow_the_leader/timeout", mission_list[i]->timeout);
			hlp_nh_->getParam("planner/missions/follow_the_leader/lat", mission_list[i]->lat);
			hlp_nh_->getParam("planner/missions/follow_the_leader/lon", mission_list[i]->lon);
			hlp_nh_->getParam("planner/missions/follow_the_leader/heading", mission_list[i]->heading);
		}
		*/
	}
}

void highLevelPlanner::publish_mission()
{
	std_msgs::String the_mission_name;
	the_mission_name.data=mission_list[current_mission]->mission_name;
	hlp_publisher.publish(the_mission_name);
}

void highLevelPlanner::next_mission()
{
	current_mission++;
}

bool highLevelPlanner::is_finished()
{
	return current_mission == number_of_missions;
}
