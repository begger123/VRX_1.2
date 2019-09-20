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
			this->task=MOVETO;
			break;
		}
		case MOVETO:
		{
			ROS_INFO("Testing publish_to_controller_NONBLOCK with 10 point trajectory");

			float x=0.0; float y=-20.0; float phi=-0; double timeval=30.0;

			move_to_sk(x, y, phi, timeval);

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