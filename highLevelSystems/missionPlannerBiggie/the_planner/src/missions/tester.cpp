#include<the_planner/missions/tester.h>

tester::tester(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	ROS_DEBUG("tester has started");
	tester_subscriber = mission_nh_->subscribe("areYouDone", 10, &tester::tester_callback, this);
	tester_publisher = mission_nh_->advertise<std_msgs::Bool>("shouldIStart", 10);
	traj_client = mission_nh_->serviceClient<the_planner::initTraj>("gen_init_traj");
}

tester::~tester()
{

}

void tester::tester_callback(const std_msgs::Bool::ConstPtr& msg)
{
	std_msgs::Bool areYouDone;
	areYouDone.data = msg->data;
	ROS_DEBUG("areYouDone is holding %d", areYouDone.data);
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
			ROS_INFO("first diddy");

			//rosservice call to /gen_init_traj
			//pipe this into array.waypoint_array
			the_planner::initTraj theTraj;
			geometry_msgs::Point garbageCanPoint;
			theTraj.request.pointA = garbageCanPoint;
			theTraj.request.pointB = garbageCanPoint;
			theTraj.request.pointC = garbageCanPoint;
			theTraj.request.pointD = garbageCanPoint;
			if (traj_client.call(theTraj))
			{
				ROS_INFO("Initial Trajectory Obtained");
			}
			else
			{
				ROS_ERROR("Failed to call service gen_init_traj");
			}

    		//directly to high_to_low_node
			custom_messages_biggie::waypoint_array array;	
			//for(int i=0; i<theTraj.response.initial_trajectory.size(); i++)
			//{
			//	array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].x);
			//	array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].y);
			//	array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].theta);
			//}
			
			//running through Armando's obstacle avoidance node    
		    //geometry_msgs::PoseArray theArray; 
			//for(int i=0; i<theTraj.response.initial_trajectory.size(); i++)
			//{
		    //    geometry_msgs::Pose elem;
		    //    elem.position.x = theTraj.response.initial_trajectory[i].y;
		    //    elem.position.y = theTraj.response.initial_trajectory[i].x;
		    //    elem.orientation.z = theTraj.response.initial_trajectory[i].theta;
		    //    elem.orientation.w = 1.0;
		    //    theArray.poses.push_back(elem);
			//}			
	        //pose_array_pub.publish(theArray);
	        //theArray.poses.clear();

			//POINTS IN NED
			array.waypoint_array={
									40, 40, 1.5,
									40, -40, 1.5,
									-40, -40, 1.5,
									-40, 40, 1.5,
									40, 40, 1.5,
								  				};//in ned

			custom_waypoint_array_publisher.publish(array);
			
			while(ros::ok())
			{
				ros::Rate loop_rate(4);
				loop_rate.sleep();
			}
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