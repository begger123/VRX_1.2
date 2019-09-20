#include<the_planner/missions/slalom_mission.h>

slalom_mission::slalom_mission(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	this->traj_mission_t.type=100;
	this->slalom_mission_sub=mission_nh_->subscribe("slalom_waypoints", 10, &slalom_mission::slalom_mission_callback, this);
}

slalom_mission::~slalom_mission()
{

}

void slalom_mission::slalom_mission_callback(const the_planner::waypoint_list::ConstPtr& msg)
{
	this->waypoint_list_t.waypoint_list=msg->waypoint_list;
	this->traj_mission_t.type=0;
	for(int i=0; i<this->waypoint_list_t.waypoint_list.size(); i++){
		this->traj_mission_t.data.push_back(this->waypoint_list_t.waypoint_list[i].y);
		this->traj_mission_t.data.push_back(this->waypoint_list_t.waypoint_list[i].x);
		this->traj_mission_t.data.push_back(1.5);
	}
}

void slalom_mission::loop()
{
	switch(this->task)
	{		
		case START:
		{
			ROS_INFO("The slalom_mission has begun");
			//go to initial waypoint
			this->task=THERE;
			break;
		}
		case THERE:
		{
			ROS_INFO("Getting info from slalom_generator");
			//start getting waypoints from slalom_generator
			ROS_INFO("Testing publish_to_controller_NONBLOCK with 10 point trajectory");
			while(this->traj_mission_t.type==100&&ros::ok()){
				//ROS_INFO("Waiting for waypoints");
				ros::spinOnce();
			}
			publish_to_controller_NONBLOCK(traj_mission_t);
			this->task=BACK;
			break;
		}
		case BACK:
		{
			ROS_INFO("White buoy circled, lets head back");

			this->task=FINISHED;
			break;
		}
		case FINISHED:
		{
			ROS_INFO("Slalom mission complete");
			finished=true;
			break;
		}
	}
}