#include<the_planner/missions/speed_gate_mission.h>

speed_gate_mission::speed_gate_mission(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	this->traj_mission_t.type=100;
	this->speed_gate_mission_sub=mission_nh_->subscribe("centroids", 10, &speed_gate_mission::speed_gate_mission_callback, this);
	this->missionComplete.data=false;
}

speed_gate_mission::~speed_gate_mission()
{

}

void speed_gate_mission::check_order()
{
	int rightBuoy=2;
	float theta1=atan2((startSpeedBuoys[0].x-theState.pose.x),(startSpeedBuoys[0].y-theState.pose.y));
	float theta2=atan2((startSpeedBuoys[1].x-theState.pose.x),(startSpeedBuoys[1].y-theState.pose.y));
	float delta=theta1-theta2;
	if(delta<-M_PI){
		delta+=2*M_PI;
	}
	else if(delta>M_PI){
		delta-=2*M_PI;
	}
	if(delta>0)
	{
		rightBuoy=1;
	}

	geometry_msgs::Point32 tempSpeedBuoys[2];
	if(rightBuoy==1){
		tempSpeedBuoys[0]=startSpeedBuoys[0];
		tempSpeedBuoys[1]=startSpeedBuoys[1];
	}
	else{
		tempSpeedBuoys[1]=startSpeedBuoys[0];
		tempSpeedBuoys[0]=startSpeedBuoys[1];
	}

	startSpeedBuoys[0]=tempSpeedBuoys[0];
	startSpeedBuoys[1]=tempSpeedBuoys[1];
}

void speed_gate_mission::calculate_midpoint_and_heading()
{
	
	//the order here is the red buoy is position 1, green buoy is position 0

	//ROS_INFO("Closer Point %f, %f", startSpeedBuoys[0].x, startSpeedBuoys[0].y);
	//ROS_INFO("Further Point %f, %f", startSpeedBuoys[1].x, startSpeedBuoys[1].y);
	//calculate midpoint
	this->midpointX=(startSpeedBuoys[0].x+startSpeedBuoys[1].x)/2;
	this->midpointY=(startSpeedBuoys[0].y+startSpeedBuoys[1].y)/2;
	//ROS_INFO("midpointX %f", midpointX);
	//ROS_INFO("midpointY %f", midpointY);

	//calculate slope between buoys
	//remember NED convention here - x means north, y means east
	//we want atan2(red buoy east - green buoy east / red buoy north - green buoy north)
	//then, if this is less than zero, add pi/2, if greater than zero, subtract pi/2 to get perpendicular

	midpointSlope=atan2((startSpeedBuoys[1].y-startSpeedBuoys[0].y),(startSpeedBuoys[1].x-startSpeedBuoys[0].x));

	//ROS_INFO("midpointSlope %f", midpointSlope);
	if(midpointSlope<0){
		midpointSlope+=M_PI/2;
	}
	else{
		midpointSlope-=M_PI/2;
	}
	

	if(midpointSlope>M_PI){
		midpointHeading-=2*M_PI;
	}
	else if (midpointSlope<-M_PI){
		midpointHeading+=2*M_PI;
	}
	else{
		midpointHeading=midpointSlope;
	}

	ROS_INFO("perpSlope %f", midpointSlope);
	ROS_INFO("midpointHeading %f", midpointHeading);
}

void speed_gate_mission::add_new_way_point()
{
	float dist=5;
	if(this->midpointHeading<0){
		this->nextPointX=this->theState.pose.x+dist*cos(this->midpointHeading);
		this->nextPointY=this->theState.pose.y+dist*sin(this->midpointHeading);
	}
	else{
		this->nextPointX=this->theState.pose.x-dist*cos(this->midpointHeading);
		this->nextPointY=this->theState.pose.y-dist*sin(this->midpointHeading);
	}
}

usv16_msgs::Usv16Mission speed_gate_mission::calculate_four_waypoints()
{
	float desired_radius=2;

	//calculate slope between buoy and object
	//remember NED convention here - x means north, y means east
	//the order here is the red buoy is position 1, green buoy is position 0
	//we want atan2(red buoy east - green buoy east / red buoy north - green buoy north)

	//we want atan2(blue buoy east - boat east / blue buoy north - boat north)
	//then, if this is less than zero, add pi/2, if greater than zero, subtract pi/2 to get perpendicular

	float newSlope=atan2((blueBuoy.y-theState.pose.y),(blueBuoy.x-theState.pose.x));
	float perpindicularSlope;
	if(perpindicularSlope<0){
		perpindicularSlope=perpindicularSlope+M_PI/2;
	}
	else{
		perpindicularSlope=perpindicularSlope-M_PI/2;
	}

	float point1x, point1y, point2x, point2y, point3x, point3y, point4x, point4y;

	if(blueBuoy.x-this->theState.pose.x==0){
		//buoy may need to be changed to vehicle
		point1x=blueBuoy.x-desired_radius;
		point1y=blueBuoy.y;
		point3x=blueBuoy.x+desired_radius;
		point3y=blueBuoy.y;
	}
	else if(blueBuoy.y-this->theState.pose.y==0){
		//buoy may need to be changed to vehicle
		point1x=blueBuoy.x;
		point1y=blueBuoy.y-desired_radius;
		point3x=blueBuoy.x;
		point3y=blueBuoy.y+desired_radius;
	}
	else{  
		point1x=blueBuoy.x-desired_radius;
		point1y=perpindicularSlope*(point1x-blueBuoy.x)+blueBuoy.y;
		point3x=blueBuoy.x+desired_radius;
		point3y=perpindicularSlope*(point3x-blueBuoy.x)+blueBuoy.y;
	}

	if(point3x-point1x==0){
		//buoy may need to be changed to vehicle
		point2x=blueBuoy.x+desired_radius;
		point2y=blueBuoy.y;
	}
	else if(point3y-point1y==0){
		//buoy may need to be changed to vehicle
		point2x=blueBuoy.x;
		point2y=blueBuoy.y+desired_radius;
	}
	else{  
		point2x=blueBuoy.x+desired_radius;
		point2y=perpindicularSlope*(point1x-blueBuoy.x)+blueBuoy.y;
	}

	usv16_msgs::Usv16Mission temp;
	temp.type=0;
	temp.data={point1x, point1y, 1.5, point2x, point2y, 1.5, point3x, point3y, 1.5, this->midpointX, this->midpointY, 1.5,};
	return temp;
}

void speed_gate_mission::speed_gate_mission_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	this->point_cloud_t.points=msg->points;
	for(int i=0; i<point_cloud_t.points.size(); i++){
		point_cloud_t.points[i].y*=-1;
	}
}

bool speed_gate_mission::gateSetFound()
{
	bool distanceCheck0=false;
	bool distanceCheck1=false;
	bool angleCheck0=false;
	bool angleCheck1=false;
	//check distance of buoy 0
	//check distance of buoy 1
	//check angle of buoy 0
	//check angle of buoy1
	if(sqrt(pow((startSpeedBuoys[0].x-this->theState.pose.x),2)+pow((startSpeedBuoys[0].y-this->theState.pose.y),2))<8.0){
		distanceCheck0=true;
		//ROS_INFO("distanceCheck0 %f",sqrt(pow((startSpeedBuoys[0].x-this->theState.pose.x),2)+pow((startSpeedBuoys[0].y-this->theState.pose.y),2)));
	}
	if(sqrt(pow((startSpeedBuoys[1].x-this->theState.pose.x),2)+pow((startSpeedBuoys[1].y-this->theState.pose.y),2))<8.0){
		distanceCheck1=true;
		//ROS_INFO("distanceCheck1 %f",sqrt(pow((startSpeedBuoys[1].x-this->theState.pose.x),2)+pow((startSpeedBuoys[1].y-this->theState.pose.y),2)));
	}
	if(sqrt(pow(atan2((startSpeedBuoys[0].x-this->theState.pose.x),(startSpeedBuoys[0].y-this->theState.pose.y)),2))<angleDelta){
		angleCheck0=true;
		//ROS_INFO("angleCheck0 %f",sqrt(pow(atan2((startSpeedBuoys[0].x-this->theState.pose.x),(startSpeedBuoys[0].y-this->theState.pose.y)),2)));
	}
	if(sqrt(pow(atan2((startSpeedBuoys[1].y-this->theState.pose.y),(startSpeedBuoys[1].x-this->theState.pose.x)),2))<angleDelta){
		angleCheck1=true;
		//ROS_INFO("angleCheck1 %f",sqrt(pow(atan2((startSpeedBuoys[1].y-this->theState.pose.y),(startSpeedBuoys[1].x-this->theState.pose.x)),2)));
	}
	if(distanceCheck0&&distanceCheck1&&angleCheck0&&angleCheck1){
		return true;
	}
}

void speed_gate_mission::loop()
{
	switch(this->task)
	{		
		case START:
		{
			ROS_INFO("The speed_gate_mission has begun, heading to 1st waypoint");
			//go to initial waypoint
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			trajMission.data=	{this->lat, this->lon, 1.5,};
			missionComplete.data=false;
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;
			this->set_head(this->heading, 10);
			this->task=GET_INITIAL_POINT;
			break;
		}
		case GET_INITIAL_POINT:
		{
			//This will provide labels to the persistance table
			//Vision system is running, 
			//Look through marker message for 2 closest can_buoys
			//Right can buoy is green, designated index 0
			//Left can buoy is red, designated index 1
			//this will need to be organized to ensure the order
			//find the two closest points that are within the cone
			bool speedGatesFound=false;
			while(!speedGatesFound && ros::ok()) {
				for(int i=0; i<point_cloud_t.points.size()-1; i++){
					for(int j=1; j<point_cloud_t.points.size(); j++){
						startSpeedBuoys[0]=point_cloud_t.points[i];
						startSpeedBuoys[1]=point_cloud_t.points[j];
						if(gateSetFound()){
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
			check_order();

			ROS_INFO("Sufficient detection, calculating next waypoint");

			calculate_midpoint_and_heading();
			ROS_INFO("%f",midpointX);
			ROS_INFO("%f",midpointY);

			//publish this command
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			trajMission.data={this->midpointX, this->midpointY, 1.5,};
			ROS_INFO("Traveling to midpoint");

			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;
			ROS_INFO("Midpoint reached");
			this->task=SEARCHING_FOR_BLUE;
			break;
		}
		case SEARCHING_FOR_BLUE:
		{
			ROS_INFO("Blue, you're my boy");
			//blue is classified as the next found object within a tight angular
			this->add_new_way_point();
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			trajMission.data=	{this->nextPointX, this->nextPointY, 1.5,};
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;
			int loopCounter=0;
			//if blue buoy not detected, add another waypoint 3 meters away on line
			while(!blueBuoyFound&&ros::ok())
			{
				ROS_INFO("Looking for blue");
				//look for objects in centroids, and see if any are within a certain distance and angle
				if(sqrt(pow((point_cloud_t.points[0].x-this->theState.pose.x),2)+pow((point_cloud_t.points[0].y-this->theState.pose.y),2))<8&&
				   sqrt(pow(atan2((point_cloud_t.points[0].y-this->theState.pose.y),(point_cloud_t.points[0].x-this->theState.pose.x)),2))<angleDelta){
					this->blueBuoy.x=point_cloud_t.points[0].x;
					this->blueBuoy.y=point_cloud_t.points[0].y;
					this->blueBuoyFound=true;
				}
				loopCounter++;
				if(loopCounter<4){
					this->add_new_way_point();
				}
				else
				{
					ROS_INFO("Max distance reached, returning home");
					this->task=HOMEWARD_BOUND;
					return;
				}
				usv16_msgs::Usv16Mission trajMission;
				trajMission.type=0;
				trajMission.data=	{this->nextPointX, this->nextPointY, 1.5,};
				while(!missionComplete.data&&ros::ok()){
					publish_to_controller_NONBLOCK(trajMission);
					ros::spinOnce();
				}
				missionComplete.data=false;
				ros::spinOnce();
			}

			this->task=BLUE_FOUND;
			break;
		}
		case BLUE_FOUND:
		{
			//since we have the blue buoy and the starting part
			//calculate 3 points around the outside of the buoy
			//and add the starting point to end to return back
			//through the gates

			//possible addition - search for gates a second time
			//and update the home position upon finding them
			ROS_INFO("My baby boy blue has been found, circling, and heading home");
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(calculate_four_waypoints());
				ros::spinOnce();
			}
			missionComplete.data=false;
			this->task=HOMEWARD_BOUND;
			break;
		}
		case HOMEWARD_BOUND:
		{							
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			trajMission.data=	{-107.64, -9.68, 1.5,};
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;

			trajMission.type=0;
			trajMission.data=	{-104.18, -4.56, 1.5,};
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;
			this->task=FINISHED;
			break;
		}
		case FINISHED:
		{
			ROS_INFO("Speed Gate Complete");
			finished=true;
			break;
		}
	}
}