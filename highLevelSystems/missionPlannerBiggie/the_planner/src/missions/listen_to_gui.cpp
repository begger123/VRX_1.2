#include<the_planner/missions/listen_to_gui.h>

/*listen_to_gui::listen_to_gui(ros::NodeHandle &nh) : Mission(nh)
{
	task=START;
	listen_to_gui_subscriber = mission_nh_->subscribe("trajectory_from_gui", 10, &listen_to_gui::listen_to_gui_callback, this);
}

listen_to_gui::~listen_to_gui()
{

}

void listen_to_gui::listen_to_gui_callback(const usv16::PathPlannerResult::ConstPtr& msg)
{
	theTrajectory.wpt_x = msg->wpt_x;
	theTrajectory.wpt_y = msg->wpt_y;
	theTrajectory.wpt_psi = msg->wpt_psi;
}

void listen_to_gui::loop()
{
	switch(this->task)
	{
		case START:
		{
			ROS_INFO("The vessel will now listen to the gui");
			while(theTrajectory.wpt_x.size()<1&&ros::ok())
			{
				ros::spinOnce();
			}
			this->task=LAST;
			break;
		}
		case LAST:
		{
			ROS_INFO("The vehicle will now begin moving");
			for(int i=0; i<theTrajectory.wpt_x.size(); i++)
			{
				missionTarget.data = {
							theTrajectory.wpt_x[i], theTrajectory.wpt_y[i], theTrajectory.wpt_psi[i],
				};
				missionTarget.type = 0;
				this->publish_to_controller(missionTarget);
				ros::spinOnce();
			}
			ROS_INFO("The mission is complete");
			theTrajectory.wpt_x.clear();
			theTrajectory.wpt_y.clear();
			theTrajectory.wpt_psi.clear();
			this->task=START;
			break;
		}
		case FINISHED:
		{
			finished=true;
			break;
		}
	}
}*/

