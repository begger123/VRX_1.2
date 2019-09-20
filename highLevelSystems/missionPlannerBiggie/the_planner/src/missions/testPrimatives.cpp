#include<the_planner/missions/tester.h>

tester::tester(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	//these two are calls to villanova
	tester_subscriber = mission_nh_->subscribe("areYouDone", 10, &tester::tester_callback, this);
	tester_publisher = mission_nh_->advertise<std_msgs::Bool>("shouldIStart", 10);
}

tester::~tester()
{

}

void tester::tester_callback(const std_msgs::Bool::ConstPtr& msg)
{
	std_msgs::Bool areYouDone;
	areYouDone.data = msg->data;
	ROS_INFO("areYouDone is holding %d", areYouDone.data);
}

void tester::loop()
{
	switch(this->task)
	{		
		case START:
		{
			ROS_INFO("the tester case statements have started");
			this->task=MOVETOSK;
			break;
		}
		case MOVETOSK:
		{
			ROS_INFO("Testing move_to_sk");
			
			float x=10.0; float y=10.0; float phi=-M_PI/4; double timeval=30.0;

			move_to_sk(x, y, phi, timeval);

			this->task=MOVETO;
			break;
		}
		case MOVETO:
		{
			ROS_INFO("Testing move_to");

			float x=20; float y=10; float vel=1.5;

			move_to(x, y, vel);

			this->task=GETCURRSETHEAD;
			break;
		}
		case GETCURRSETHEAD:
		{
			ROS_INFO("Testing get_curr and set_head");

			ROS_INFO("the current X location is: %f", get_curr(X));
			ROS_INFO("the current Y location is: %f", get_curr(Y));
			ROS_INFO("the current PSI is: %f", get_curr(PSI));
			
			double timeval=30.;

			set_head(get_curr(PSI)+M_PI/2, timeval);

			this->task=MOVEBY;
			break;
		}
		case MOVEBY:
		{
			ROS_INFO("Testing move_by");

			float delx=5.0; float dely=5.0; float vel=1.5; float delphi=M_PI/8; double timeval=30.;

			move_by(delx, dely, vel, delphi, timeval);

			this->task=MOVEBYHEADING;
			break;
		}
		case MOVEBYHEADING:
		{
			ROS_INFO("Testing move_by_heading");

			float distance = 10;

			move_by_heading(distance);

			this->task=FINISHED;
			break;
		}
		case FINISHED:
		{
			finished=true;
			break;
		}
	}
}
