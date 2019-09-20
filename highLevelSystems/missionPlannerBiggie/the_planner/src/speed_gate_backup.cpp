#include<the_planner/missions/speed_gate_mission.h>

speed_gate_mission::speed_gate_mission(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	this->traj_mission_t.type=100;
	this->speed_gate_mission_sub=mission_nh_->subscribe("markers", 10, &speed_gate_mission::speed_gate_mission_callback, this);
}

speed_gate_mission::~speed_gate_mission()
{

}

void speed_gate_mission::calculate_midpoint_and_heading(std::vector<usv_ahc_py::buoy> startSpeedBuoys)
{
	
	//the order here is the red buoy is position 1, green buoy is position 0

	//calculate midpoint
	ROS_INFO("length %i",startSpeedBuoys.size());
	this->midpointX=(startSpeedBuoys[0].centroid.x+(startSpeedBuoys[1].centroid.x-startSpeedBuoys[0].centroid.x)/2);
	this->midpointY=-(startSpeedBuoys[0].centroid.y+(startSpeedBuoys[1].centroid.y-startSpeedBuoys[0].centroid.y)/2);
	ROS_INFO("midpointX %f", midpointX);
	ROS_INFO("midpointY %f", midpointY);

	//calculate slope between buoys
	//remember NED convention here - x means north, y means east
	//we want atan2(red buoy east - green buoy east / red buoy north - green buoy north)
	//then, if this is less than zero, add pi/2, if greater than zero, subtract pi/2 to get perpendicular

	midpointSlope=atan2((startSpeedBuoys[0].centroid.y-startSpeedBuoys[1].centroid.y),(startSpeedBuoys[0].centroid.x-startSpeedBuoys[1].centroid.x));

	if(midpointSlope<0){
		midpointHeading=midpointSlope+M_PI/2;
	}
	else{
		midpointHeading=midpointSlope-M_PI/2;
	}

	if(midpointHeading>M_PI){
		midpointHeading-=2*M_PI;
	}
	else if (midpointHeading<-M_PI){
		midpointHeading+=2*M_PI;
	}

	ROS_INFO("midpointSlope %f", midpointSlope);
	ROS_INFO("midpointSlope %f", midpointHeading);
}

void speed_gate_mission::add_new_way_point()
{
	float dist=5;
	if(this->midpointHeading>0){
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
	float desired_radius=4;

	//calculate slope between buoy and object
	//remember NED convention here - x means north, y means east
	//the order here is the red buoy is position 1, green buoy is position 0
	//we want atan2(red buoy east - green buoy east / red buoy north - green buoy north)

	//we want atan2(blue buoy east - boat east / blue buoy north - boat north)
	//then, if this is less than zero, add pi/2, if greater than zero, subtract pi/2 to get perpendicular

	float newSlope=atan2((blueBuoy.centroid.y-theState.pose.y),(blueBuoy.centroid.x-theState.pose.x));
	float perpindicularSlope;
	if(perpindicularSlope<0){
		perpindicularSlope=perpindicularSlope+M_PI/2;
	}
	else{
		perpindicularSlope=perpindicularSlope-M_PI/2;
	}

	float point1x, point1y, point2x, point2y, point3x, point3y, point4x, point4y;

	if(blueBuoy.centroid.x-this->theState.pose.x==0){
		//buoy may need to be changed to vehicle
		point1x=blueBuoy.centroid.x-desired_radius;
		point1y=blueBuoy.centroid.y;
		point3x=blueBuoy.centroid.x+desired_radius;
		point3y=blueBuoy.centroid.y;
	}
	else if(blueBuoy.centroid.y-this->theState.pose.y==0){
		//buoy may need to be changed to vehicle
		point1x=blueBuoy.centroid.x;
		point1y=blueBuoy.centroid.y-desired_radius;
		point3x=blueBuoy.centroid.x;
		point3y=blueBuoy.centroid.y+desired_radius;
	}
	else{  
		point1x=blueBuoy.centroid.x-desired_radius;
		point1y=perpindicularSlope*(point1x-blueBuoy.centroid.x)+blueBuoy.centroid.y;
		point3x=blueBuoy.centroid.x+desired_radius;
		point3y=perpindicularSlope*(point3x-blueBuoy.centroid.x)+blueBuoy.centroid.y;
	}

	if(point3x-point1x==0){
		//buoy may need to be changed to vehicle
		point2x=blueBuoy.centroid.x+desired_radius;
		point2y=blueBuoy.centroid.y;
	}
	else if(point3y-point1y==0){
		//buoy may need to be changed to vehicle
		point2x=blueBuoy.centroid.x;
		point2y=blueBuoy.centroid.y+desired_radius;
	}
	else{  
		point2x=blueBuoy.centroid.x+desired_radius;
		point2y=perpindicularSlope*(point1x-blueBuoy.centroid.x)+blueBuoy.centroid.y;
	}

	usv16_msgs::Usv16Mission temp;
	temp.type=0;
	temp.data={point1x, point1y, 1.5, point2x, point2y, 1.5, point3x, point3y, 1.5, this->midpointX, this->midpointY, 1.5,};
	return temp;
}

void speed_gate_mission::speed_gate_mission_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	this->marker_array_t.markers=msg->markers;
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
			ROS_INFO("Just looking to get the midpoint");

			while(marker_array_t.markers.size()<2 && ros::ok()) {
				ros::spinOnce();
				ROS_INFO("Spinning while waiting for 2 markers");
			}

			std::vector<usv_ahc_py::buoy> startSpeedBuoys;
			usv_ahc_py::buoy tempBuoy;


			for(int i=0; i < marker_array_t.markers.size(); i++){ 
				ROS_INFO("In the for loop %i",i);
				std::string label = "large_sphere_buoy";
				if(!label.compare(marker_array_t.markers[i].ns.c_str())){
					ROS_INFO("buoy x %f", marker_array_t.markers[i].pose.position.x);
					ROS_INFO("buoy y %f", marker_array_t.markers[i].pose.position.y);
					tempBuoy.centroid.x=this->marker_array_t.markers[i].pose.position.x;
					tempBuoy.centroid.y=this->marker_array_t.markers[i].pose.position.y;
					tempBuoy.centroid.z=this->marker_array_t.markers[i].pose.position.z;
					startSpeedBuoys.push_back(tempBuoy);
				}
			}
			if(startSpeedBuoys.size()>=2)
			{
				//get the 2 closest
				//twoBuoyFlag=true;
			}
			//else if(startSpeedBuoys.size()<2)
			//{
			//	ROS_INFO("NOT ENOUGH BUOYS FOUND, EXITING MISSION");
			//	break;
			//}
			
			//make sure the right buoy is index 0
			//stuff

			ROS_INFO("Sufficient detection, calculating next waypoint");

			calculate_midpoint_and_heading(startSpeedBuoys);
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

			//if blue buoy not detected, add another waypoint 3 meters away on line
			while(!blueBuoyFound&&ros::ok())
			{
				for(int i=0; i<this->marker_array_t.markers.size(); i++){
					if(sqrt(pow(atan2((this->marker_array_t.markers[i].pose.position.y-this->theState.pose.y),(this->marker_array_t.markers[i].pose.position.x-this->theState.pose.x)),2))<angleDelta){
						this->blueBuoy.centroid.x=this->marker_array_t.markers[i].pose.position.x;
						this->blueBuoy.centroid.y=this->marker_array_t.markers[i].pose.position.y;
						this->blueBuoy.centroid.z=this->marker_array_t.markers[i].pose.position.z;
						this->blueBuoyFound=true;
					}
				}
				if(!this->maxDistanceTraveled){
					this->add_new_way_point();
				}
				else
				{
					this->task=HOMEWARD_BOUND;
					break;
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