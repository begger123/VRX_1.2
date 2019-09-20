//This will be a cascaded heading and velocity controller
#include <high_to_low/waypoints_to_waypoint.h>

waypoints_to_waypoint::ws2w::ws2w(ros::NodeHandle &nh) : ws2w_nh(&nh), loop_rate(4) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("waypoints_to_waypoint started");

	waypoint_array_sub = ws2w_nh->subscribe("waypoint_array", 10, &waypoints_to_waypoint::ws2w::waypoint_array_callback, this);
	state_sub = ws2w_nh->subscribe("/p3d_wamv_ned", 10, &waypoints_to_waypoint::ws2w::state_callback, this);
	control_target_pub = ws2w_nh->advertise<geometry_msgs::Pose2D>("/control_target", 10); //published TAU = {X,Y,Eta}
	mission_complete_pub = ws2w_nh->advertise<std_msgs::Bool>("/missionComplete", 1);
	missionComplete.data=false;

}

waypoints_to_waypoint::ws2w::~ws2w()
{

}

void waypoints_to_waypoint::ws2w::waypoint_array_callback(const custom_messages_biggie::waypoint_array::ConstPtr& msg)
{
	the_array.waypoint_array=msg->waypoint_array;
	index=0;
	newWaypoint=true;
}

void waypoints_to_waypoint::ws2w::state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//ROS_DEBUG("waypoints_to_waypoint state_callback");
	state_data.header=msg->header;
	state_data.child_frame_id=msg->child_frame_id;
	state_data.pose=msg->pose;
	state_data.twist=msg->twist;

	newState=true;
}

int waypoints_to_waypoint::ws2w::error_checking()
{
	return 1;
}

bool waypoints_to_waypoint::ws2w::within_error_bound()
{
	//ROS_DEBUG("waypoints_to_waypoint within_error_bound");
	current_waypoint.x=the_array.waypoint_array[index];
	current_waypoint.y=the_array.waypoint_array[index+1];
	current_waypoint.theta=the_array.waypoint_array[index+2];
	//ROS_DEBUG("The waypoint message is: %f, %f, %f", current_waypoint.x, current_waypoint.y, current_waypoint.z);
	//ROS_DEBUG("The state message is: %f, %f, %f", state_data.pose.pose.position.x, state_data.pose.pose.position.y, state_data.pose.pose.position.z);
	error_magnitude=sqrt(pow(current_waypoint.x-state_data.pose.pose.position.x,2)+pow(current_waypoint.y-state_data.pose.pose.position.y,2));
	ROS_DEBUG("The error magnitude is %f", error_magnitude);
	//this functiom is the one that will hangle the current index of the waypoint array to be published to controller
	if(error_magnitude<error_bound)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void waypoints_to_waypoint::ws2w::pub()
{
	control_target_pub.publish(current_waypoint);
}


void waypoints_to_waypoint::ws2w::run()
{
	while(ros::ok())
	{
		//if we have a new state message and a new waypoint message
		//ROS_WARN("newState and newWaypoint %d, %d", newState, newWaypoint);
		if(newState&&newWaypoint)
		{
			//if the error is within bounds, and error_checking is clear, iterate array to publish the next waypoint
			if(!within_error_bound())
			{
				//do nothing
			}
			else
			{
				//need to check for end of message
				//ROS_WARN("Waypoint size is:%f",the_array.waypoint_array.size());
				if(index<(the_array.waypoint_array.size()-3))
				{
					//we increment by 3 here since a waypoint is comprised of an x, y, and velocity
					index=index+3;	
				}
				else
				{
					newWaypoint=false;
					missionComplete.data=true;
					mission_complete_pub.publish(missionComplete);
				}
			}
			this->pub();
			//else either publish nothing, or throw an error, or something not yet sure about	
			newState=false;
		}
		missionComplete.data=false;
		mission_complete_pub.publish(missionComplete);
		ros::spinOnce();
		loop_rate.sleep();
	}
}