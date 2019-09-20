#include<the_planner/missions/buoy_field.h>

buoy_field::buoy_field(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
}

buoy_field::~buoy_field()
{

}

void buoy_field::buoy_field_callback()
{
}

void buoy_field::loop()
{
	switch(this->task)
	{	
		//go to buoy field
		//detect white can buoy
		//detect which corner we are in
		//go to other corner
		case START:
		{
			ROS_INFO("The buoy_field has begun, heading to 1st waypoint");
			//go to initial waypoint
			
			// Define message and fill it up with the coordinates of the totems entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = this->north;
            theGoal.position.y = this->east;

            ROS_WARN("north and east in buoy field are %f %f", this->north, this->east);

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = theOdom.pose.pose.position.x;
            goto_srv.request.vehicle_pos.y = theOdom.pose.pose.position.y;
            goto_srv.request.dist_stop     = 0.0;

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            else {
                ROS_INFO("Service call failed");
                finished = true;
            }

            previous_status = 1;
            while (ros::ok() && goal_reached == false) {
                ros::spinOnce();
                ROS_INFO("The vehicle hasn't reached the goal yet");
                ros::Rate rate(10);
                rate.sleep();
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = THROUGH_THE_FIELD;
            }
		}
		case THROUGH_THE_FIELD:
		{
			ROS_INFO("Into the fray once more.");
			geometry_msgs::Pose theGoal;
            theGoal.position.x = 32;
            theGoal.position.y = 113;

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = theOdom.pose.pose.position.x;
            goto_srv.request.vehicle_pos.y = theOdom.pose.pose.position.y;
            goto_srv.request.dist_stop     = 0.0;

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            else {
                ROS_INFO("Service call failed");
                finished = true;
            }

            previous_status = 1;
            while (ros::ok() && goal_reached == false) {
                ros::spinOnce();
                ROS_DEBUG("The vehicle hasn't reached the goal yet");
                ros::Rate rate(10);
                rate.sleep();
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
