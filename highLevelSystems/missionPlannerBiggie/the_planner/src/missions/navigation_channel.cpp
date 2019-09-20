#include<the_planner/missions/navigation_channel.h>

navigation_channel::navigation_channel(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	this->navigation_channel_sub=mission_nh_->subscribe("centroids", 10, &navigation_channel::navigation_channel_callback, this);
}

navigation_channel::~navigation_channel()
{

}

void navigation_channel::check_order(geometry_msgs::Point32 startNavBuoys[2])
{
	int rightBuoy=2;
	float theta1=atan2((startNavBuoys[0].x-theOdom.pose.pose.position.x),(startNavBuoys[0].y-theOdom.pose.pose.position.y));
	float theta2=atan2((startNavBuoys[1].x-theOdom.pose.pose.position.x),(startNavBuoys[1].y-theOdom.pose.pose.position.y));
	float delta=theta1-theta2;
	if(delta<-M_PI){
		delta-=2*M_PI;
	}
	else if(delta>M_PI){
		delta+=2*M_PI;
	}
	if(delta>0)
	{
		rightBuoy=1;
	}

	geometry_msgs::Point32 tempSpeedBuoys[2];
	if(rightBuoy==1){
		tempSpeedBuoys[0]=startNavBuoys[0];
		tempSpeedBuoys[1]=startNavBuoys[1];
	}
	else{
		tempSpeedBuoys[1]=startNavBuoys[1];
		tempSpeedBuoys[0]=startNavBuoys[0];
	}

	startNavBuoys[0]=tempSpeedBuoys[0];
	startNavBuoys[1]=tempSpeedBuoys[1];
	//ROS_INFO("%f,%f",startNavBuoys[0].x, startNavBuoys[0].y);
	//ROS_INFO("%f,%f",startNavBuoys[1].x, startNavBuoys[1].y);
}

void navigation_channel::calculate_midpoint_and_heading(geometry_msgs::Point32 startNavBuoys[2])
{

	//the order here is the red buoy is position 1, green buoy is position 0

	ROS_INFO("Closer Point %f, %f", startNavBuoys[0].x, startNavBuoys[0].y);
	ROS_INFO("Further Point %f, %f", startNavBuoys[1].x, startNavBuoys[1].y);
	//calculate midpoint
	this->midpointX=(startNavBuoys[0].x+startNavBuoys[1].x)/2;
	this->midpointY=(startNavBuoys[0].y+startNavBuoys[1].y)/2;
	ROS_INFO("midpointX %f", midpointX);
	ROS_INFO("midpointY %f", midpointY);

	//calculate slope between buoys
	//remember NED convention here - x means north, y means east
	//we want atan2(red buoy east - green buoy east / red buoy north - green buoy north)
	//then, if this is less than zero, add pi/2, if greater than zero, subtract pi/2 to get perpendicular

	midpointSlope=atan2((startNavBuoys[1].y-startNavBuoys[0].y),(startNavBuoys[1].x-startNavBuoys[0].x));

	ROS_INFO("midpointSlope %f", midpointSlope);
	if(midpointSlope<0){
		midpointSlope+=M_PI/2;
	}
	else{
		midpointSlope-=M_PI/2;
	}

	if(midpointHeading>M_PI){
		midpointHeading-=2*M_PI;
	}
	else if (midpointHeading<-M_PI){
		midpointHeading+=2*M_PI;
	}
	else{
		midpointHeading=midpointSlope;
	}

	ROS_INFO("perpSlope %f", midpointSlope);
	ROS_INFO("midpointHeading %f", midpointHeading);
}

void navigation_channel::add_new_way_point()
{
	float dist=8;
	this->nextPointX=this->theOdom.pose.pose.position.x+dist*cos(this->midpointHeading);
	this->nextPointY=this->theOdom.pose.pose.position.y+dist*sin(this->midpointHeading);
	//if(this->midpointHeading>0){
	//	this->nextPointX=this->theOdom.pose.pose.position.x+dist*cos(this->midpointHeading);
	//	this->nextPointY=this->theOdom.pose.pose.position.y+dist*sin(this->midpointHeading);
	//}
	//else{
	//	this->nextPointX=this->theOdom.pose.pose.position.x-dist*cos(this->midpointHeading);
	//	this->nextPointY=this->theOdom.pose.pose.position.y-dist*sin(this->midpointHeading);
	//}
}

void navigation_channel::navigation_channel_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	sensor_msgs::PointCloud tempCloud;
	tempCloud.header=msg->header;
	tempCloud.points=msg->points;
	try{
	    lidar_nwu_to_ned_origin_tf.transformPointCloud("ned_origin", tempCloud, point_cloud_t);
  	}
  	catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"lidar_nwu\" to \"ned_origin\": %s", ex.what());
  	}
}

bool navigation_channel::gateSetFound(geometry_msgs::Point32 startNavBuoys[2])
{
	bool distanceCheck0=false;
	bool distanceCheck1=false;
	bool angleCheck0=false;
	bool angleCheck1=false;
	//check distance of buoy 0
	//check distance of buoy 1
	//check angle of buoy 0
	//check angle of buoy1
	if(sqrt(pow((startNavBuoys[0].x-this->theOdom.pose.pose.position.x),2)+pow((startNavBuoys[0].y-this->theOdom.pose.pose.position.y),2))<20.0){
		distanceCheck0=true;
	}
	if(sqrt(pow((startNavBuoys[1].x-this->theOdom.pose.pose.position.x),2)+pow((startNavBuoys[1].y-this->theOdom.pose.pose.position.y),2))<20.0){
		distanceCheck1=true;
	}
	if(sqrt(pow(atan2((startNavBuoys[0].x-this->theOdom.pose.pose.position.x),(startNavBuoys[0].y-this->theOdom.pose.pose.position.y)),2))<angleDelta){
		angleCheck0=true;
	}
	if(sqrt(pow(atan2((startNavBuoys[1].x-this->theOdom.pose.pose.position.x),(startNavBuoys[1].y-this->theOdom.pose.pose.position.y)),2))<angleDelta){
		angleCheck1=true;
	}

	//ROS_INFO("distanceCheck0 %f",sqrt(pow((startNavBuoys[0].x-this->theOdom.pose.pose.position.x),2)+pow((startNavBuoys[0].y-this->theOdom.pose.pose.position.y),2)));
	//ROS_INFO("distanceCheck1 %f",sqrt(pow((startNavBuoys[1].x-this->theOdom.pose.pose.position.x),2)+pow((startNavBuoys[1].y-this->theOdom.pose.pose.position.y),2)));
	//ROS_INFO("angleCheck0 %f",sqrt(pow(atan2((startNavBuoys[0].x-this->theOdom.pose.pose.position.x),(startNavBuoys[0].y-this->theOdom.pose.pose.position.y)),2)));
	//ROS_INFO("angleCheck1 %f",sqrt(pow(atan2((startNavBuoys[1].x-this->theOdom.pose.pose.position.x),(startNavBuoys[1].y-this->theOdom.pose.pose.position.y)),2)));
	if(distanceCheck0&&distanceCheck1&&angleCheck0&&angleCheck1){
		return true;
	}
}

void navigation_channel::loop()
{
	switch(this->task)
	{		
		case START:
		{
			ROS_INFO("The navigation_channel mission has begun, heading to 1st waypoint");
			//go to initial waypoint
			
			// Define message and fill it up with the coordinates of the totems entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = this->north;
            theGoal.position.y = this->east;

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
            while (ros::ok()&& goal_reached == false) {
                ros::spinOnce();
                ROS_DEBUG("The vehicle hasn't reached the goal yet");
                ros::Rate rate(10);
                rate.sleep();
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = GET_INITIAL_POINT;
            }
		}
		case GET_INITIAL_POINT:
		{
			//This will provide labels to the persistance table
			//Vision system is running, 
			//Look through marker message for 2 closest can_buoys
			//Right can buoy is green, designated index 0
			//Left can buoy is red, designated index 1
			//this will need to be organized to ensure the order
			ROS_INFO("Just looking to get the midpoint");

			//the point cloud that is being published in order of ascending order, therefore the buoys in
			//locations 0 and 1 are the two closest.
			//however, what we really need to check is for the two closest objects that fit inside of a certain cone
			//lets say plus minus 20 deg

			//find the two closest points that are within the cone
			bool speedGatesFound=false;
			geometry_msgs::Point32 startNavBuoys[2];
			while(!speedGatesFound && ros::ok()) {
				for(int i=0; i<point_cloud_t.points.size()-1; i++){
					for(int j=1; j<point_cloud_t.points.size(); j++){
						startNavBuoys[0]=point_cloud_t.points[i];
						startNavBuoys[1]=point_cloud_t.points[j];
						if(gateSetFound(startNavBuoys)){
							speedGatesFound=true;
							i=point_cloud_t.points.size()-1;
							j=point_cloud_t.points.size();
						}
					}
				}
				ros::Rate rate(10);
				rate.sleep();
				ros::spinOnce();
				ROS_INFO("Spinning while waiting for 2 markers");
			}
			
			//make sure the right buoy is index 0
			//stuff
			check_order(startNavBuoys);

			ROS_INFO("Sufficient detection, calculating next waypoint");

			calculate_midpoint_and_heading(startNavBuoys);
			ROS_DEBUG("%f",midpointX);
			ROS_DEBUG("%f",midpointY);

			//publish this command
			custom_messages_biggie::waypoint_array array;	
			array.waypoint_array={this->midpointX, this->midpointY, 1.5,};//in ned
			ROS_INFO("Traveling to midpoint");
			while(!missionComplete.data&&ros::ok()){
				custom_waypoint_array_publisher.publish(array);
				ros::spinOnce();
			}
			missionComplete.data=false;
			ROS_INFO("Midpoint reached");
			this->task=SEARCHING_FOR_NEXT;
			break;
		}
		case SEARCHING_FOR_NEXT:
		{
			this->add_new_way_point();
			custom_messages_biggie::waypoint_array array;	
			array.waypoint_array={this->nextPointX, this->nextPointY, 1.5,};//in ned
			ROS_INFO("Traveling to next point");
			while(!missionComplete.data&&ros::ok()){
				custom_waypoint_array_publisher.publish(array);
				ros::spinOnce();
			}
			missionComplete.data=false;
			ROS_INFO("Let's get these next gates");
			//if the next gates are not found
			while(!nextSetFound&&ros::ok())
			{
				ROS_DEBUG("Looking for next gates");
				//if the next two clostest objects are a certain distance and fall within a cone
				if(point_cloud_t.points.size()>=2 && ros::ok()) {
					geometry_msgs::Point32 startNavBuoys[2];

					//the point cloud that is being published in order of ascending order, therefore the buoys in
					//locations 0 and 1 are the two closest.
					startNavBuoys[0]=point_cloud_t.points[0];
					startNavBuoys[1]=point_cloud_t.points[1];
						
					if(gateSetFound(startNavBuoys)) {
						
						ROS_INFO("Found the exit gates");
						this->nextSetFound=true;
						check_order(startNavBuoys);

						calculate_midpoint_and_heading(startNavBuoys);
						ROS_INFO("%f",midpointX);
						ROS_INFO("%f",midpointY);

						//publish this command
						array.waypoint_array={this->midpointX, this->midpointY, 1.5,};//in ned
						ROS_INFO("Traveling to final midpoint");
						while(!missionComplete.data&&ros::ok()){
							custom_waypoint_array_publisher.publish(array);
							ros::spinOnce();
						}
						missionComplete.data=false;
						this->task=FINAL_POINT;
						break;
					}

					if(!this->maxDistanceTraveled){
						this->add_new_way_point();
					}
					else
					{
						this->task=FINAL_POINT;
						break;
					}
					array.waypoint_array={this->nextPointX, this->nextPointY, 1.5,};//in ned
					ROS_INFO("Traveling to next point");
					while(!missionComplete.data&&ros::ok()){
						custom_waypoint_array_publisher.publish(array);
						ros::spinOnce();
					}
					missionComplete.data=false;
				}
				ros::spinOnce();
			}
			this->task=FINAL_POINT;
			break;
		}
		case FINAL_POINT:
		{
			ROS_INFO("Now that the gates are clear, on to the rest of the course");
			custom_messages_biggie::waypoint_array array;	
			array.waypoint_array={80, 20, 1.5,};//in ned
			ROS_INFO("Traveling to safe point");
			while(!missionComplete.data&&ros::ok()){
				custom_waypoint_array_publisher.publish(array);
				ros::spinOnce();
			}
			missionComplete.data=false;
			this->task=FINISHED;
			break;
		}
		case FINISHED:
		{
			ROS_INFO("Navigation channel complete");
			finished=true;
			break;
		}
	}
}

