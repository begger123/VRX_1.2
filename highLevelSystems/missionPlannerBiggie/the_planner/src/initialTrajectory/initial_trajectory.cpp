#include <the_planner/initialTrajectory/initial_trajectory.h>

//COMPUTES IN THE NED FRAME
high_level::gen_trajectory::gen_trajectory(ros::NodeHandle &nh) : init_traj_nh(&nh), loop_rate(4) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	state_sub = init_traj_nh->subscribe("/p3d_wamv_ned", 10, &high_level::gen_trajectory::state_callback, this);//use this for simulation
	ROS_DEBUG("get state");
	while(ros::ok()&&!newState)
	{
		ros::spinOnce();
	}
	this->get_params();
}

high_level::gen_trajectory::~gen_trajectory()
{

}

void high_level::gen_trajectory::get_params()
{
	ROS_DEBUG("getting params");
	//This style of obtaining params is used because it resolves the param relative to the namespace of the node
  	ros::param::get("high_level/pointA/x", pointA.x);
  	ros::param::get("high_level/pointA/y", pointA.y);
  	ros::param::get("high_level/pointB/x", pointB.x);
  	ros::param::get("high_level/pointB/y", pointB.y);
  	ros::param::get("high_level/pointC/x", pointC.x);
  	ros::param::get("high_level/pointC/y", pointC.y);
  	ros::param::get("high_level/pointD/x", pointD.x);
  	ros::param::get("high_level/pointD/y", pointD.y);
  	ros::param::get("high_level/RoD", RoD);
  	ros::param::get("high_level/wpSpace", wpSpace);

  	this->define_lines();
  	this->set_csp();
  	this->generate_trajectory();
}

void high_level::gen_trajectory::state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	state_data.header=msg->header;
	state_data.child_frame_id=msg->child_frame_id;
	state_data.pose=msg->pose;
	state_data.twist=msg->twist;

	Eigen::Vector3f nedLocation;
	nedLocation << state_data.pose.pose.position.x, state_data.pose.pose.position.y, state_data.pose.pose.position.z;
	
	vehicle.x=nedLocation[0];
	vehicle.y=nedLocation[1];
	vehicle.theta=tf::getYaw(state_data.pose.pose.orientation); //use tf::transform_datatypes function getYaw to convert Odom's quaternion to yaw

	newState=true;
}

void high_level::gen_trajectory::define_lines()
{	
	ROS_DEBUG("defining lines");
	//Use points to get bounding lines
	//point 1 to 2
	lineA=headingDist(pointA,pointB,wpSpace);
	//waypointsA=setWaypoints(lineA);
	//point 2 to 3
	lineB=headingDist(pointB,pointC,wpSpace);
	//waypointsB=setWaypoints(lineB);
	//point 3 to 4
	lineC=headingDist(pointC,pointD,wpSpace);
	//waypointsC=setWaypoints(lineC);
	//point 4 to 1
	lineD=headingDist(pointD,pointA,wpSpace);
	//waypointsD=setWaypoints(lineD);
}

the_planner::line high_level::gen_trajectory::headingDist(geometry_msgs::Point firstPoint, geometry_msgs::Point secondPoint, double wpSpace)
{
    the_planner::line tempLine;
    tempLine.heading.data=atan2((pointB.x-pointA.x),(pointB.y-pointA.y));//to follow NED convention
    tempLine.distance.data=sqrt(pow((pointB.x-pointA.x),2)+pow((pointB.y-pointA.y),2));
    tempLine.start=firstPoint;
    tempLine.finish=secondPoint;
    tempLine.numPoints.data=floor((tempLine.distance.data)/wpSpace);
    tempLine.wpSpace.data=wpSpace;
    return tempLine;
}

void high_level::gen_trajectory::set_csp()
{
	ROS_DEBUG("setting csp");
	//Get Candidate Commence Search Points
	CCSP[0].x=pointA.x; CCSP[0].y=pointA.y+RoD; CCSP[0].theta=lineA.heading.data;
	CCSP[1].x=pointB.x; CCSP[1].y=pointB.y+RoD; CCSP[1].theta=lineA.heading.data+M_PI;//needs wrapping
	CCSP[2].x=pointC.x; CCSP[2].y=pointC.y-RoD; CCSP[2].theta=lineC.heading.data;
	CCSP[3].x=pointD.x; CCSP[3].y=pointD.y-RoD; CCSP[3].theta=lineC.heading.data+M_PI;//needs wrapping
	//Select CSP
    theMin=std::min({sqrt(pow((CCSP[0].x-vehicle.x),2)+pow((CCSP[0].y-vehicle.y),2)), 
                sqrt(pow((CCSP[1].x-vehicle.x),2)+pow((CCSP[1].y-vehicle.y),2)), 
                sqrt(pow((CCSP[2].x-vehicle.x),2)+pow((CCSP[2].y-vehicle.y),2)),  
                sqrt(pow((CCSP[3].x-vehicle.x),2)+pow((CCSP[3].y-vehicle.y),2))});
    if(sqrt(pow((CCSP[0].x-vehicle.x),2)+pow((CCSP[0].y-vehicle.y),2))==theMin)
    {
        CSP.CSP.x=CCSP[0].x;
        CSP.CSP.y=CCSP[0].y;
        CSP.CSP.theta=CCSP[0].theta;
        CSP.choice.data=1;
    }
    else if(sqrt(pow((CCSP[1].x-vehicle.x),2)+pow((CCSP[1].y-vehicle.y),2))==theMin)
    {
        CSP.CSP.x=CCSP[1].x;
        CSP.CSP.y=CCSP[1].y;
        CSP.CSP.theta=CCSP[1].theta;
        CSP.choice.data=2;
    }
    else if(sqrt(pow((CCSP[2].x-vehicle.x),2)+pow((CCSP[2].y-vehicle.y),2))==theMin)
    {
        CSP.CSP.x=CCSP[2].x;
        CSP.CSP.y=CCSP[2].y;
        CSP.CSP.theta=CCSP[2].theta;
        CSP.choice.data=3;
    }
    else if(sqrt(pow((CCSP[3].x-vehicle.x),2)+pow((CCSP[3].y-vehicle.y),2))==theMin)
    {
        CSP.CSP.x=CCSP[3].x;
        CSP.CSP.y=CCSP[3].y;
        CSP.CSP.theta=CCSP[3].theta;
        CSP.choice.data=4;
    }
    else
    {
        ROS_ERROR("Bad assignment in CSP");
        CSP.CSP.x=-1;
        CSP.CSP.y=-1;
        CSP.CSP.theta=-1;
        CSP.choice.data=-1;
    }

	//The CSP choice also governs which enumerated leg of the trajetory we start with
	//The unofficial enumeration is
	//1 - west to east
	//2 - south to north
	//3 - east to west
	//4 - north to south
	if(CSP.choice.data==1)
	{
	    lineNum=1;
	}
	else if(CSP.choice.data==2)
	{
	    lineNum=3;
	}
	else if(CSP.choice.data==3)
	{
	    lineNum=3;
	}
	else if(CSP.choice.data==4)
	{
	    lineNum=1;
	}
	else{
	    ROS_ERROR("error in line number selection");
	}
	ROS_DEBUG("CSP set to %f, %f",CSP.CSP.x,CSP.CSP.y);
	ROS_DEBUG("CSP choice %i",CSP.choice.data);
}

void high_level::gen_trajectory::generate_trajectory()
{
	ROS_DEBUG("generating trajectory");
	//South to North Case
	anchorPoint.x=CSP.CSP.x;
	anchorPoint.y=CSP.CSP.y;
	if(CSP.choice.data==1||CSP.choice.data==2)
	{
    	int lastFlag=0; //0 is unset, 1 is westEast, 2 is eastWest
	    while(ros::ok()&&anchorPoint.x<std::max({pointC.x, pointD.x}))
	    //for (int i=0; i<4; i++) //used for debugging in case while conditions are never met
	    {
	    	//ROS_DEBUG("lineNum is %i",lineNum);
	    	//ROS_DEBUG("anchorPoint x and y are %f, %f",anchorPoint.x, anchorPoint.y);
			//ROS_DEBUG("anchorPoint y is %f",anchorPoint.y);
	        //we
			if(lineNum==1)
	        {
	            endPoint.x=anchorPoint.x;
	            endPoint.y=lineB.start.y-RoD;
	            westEastFlag=true;
	        }
	        //sn
	        else if(lineNum==2)
	        {
	            endPoint.x=anchorPoint.x+RoD;
	            endPoint.y=anchorPoint.y;
	        }
	        //ew
	        else if(lineNum==3)
	        {
	            endPoint.x=anchorPoint.x;
	            endPoint.y=lineD.start.y+RoD;
	            eastWestFlag=true;
	        }
	        //sn
	        else if(lineNum==4)
	        {
	            endPoint.x=anchorPoint.x+RoD;
	            endPoint.y=anchorPoint.y;
	        }
	        else
	        {
	            ROS_ERROR("error in south to north case");
	        }

	        gen_segment(anchorPoint, endPoint, wpSpace);

	        if(westEastFlag)
	        {	        	
	            ROS_ERROR("setting west east");
		        anchorPoint.x=segment[segment.size()-1].x;
		        anchorPoint.y=segment[segment.size()-1].y+5;
		        westEastFlag=false;
	        }
	        else if(eastWestFlag)
	        {
	            ROS_ERROR("setting east west");
		        anchorPoint.x=segment[segment.size()-1].x;
		        anchorPoint.y=segment[segment.size()-1].y-5;
		        eastWestFlag=false;
	        }
	        else
	        {
	            ROS_ERROR("setting south north");
		        anchorPoint.x=segment[segment.size()-1].x+5;
		        anchorPoint.y=segment[segment.size()-1].y;	        	
	        }

	        lineNum=lineNum+1;
	        if(lineNum==5)
	        {
	            lineNum=1;
	        }
	        //should be a decend performance gain
	        fullSegment.reserve(segment.size());
	        for(int i=0; i<segment.size(); i++)
	        {
	        	fullSegment.push_back(segment[i]);
	        }
	    }
	}
	//end S to N case
	//begin N to S case
	else if(CSP.choice.data==3||CSP.choice.data==4)
	{
	    while(ros::ok()&&anchorPoint.y>std::min({pointA.x, pointB.x}))
	    {
	        //we
	        if(lineNum==1)
	        {
	            endPoint.x=anchorPoint.x;
	            endPoint.y=lineB.start.y-RoD-5;
	            eastWestFlag=true;
	        }
	        //ns
	        else if(lineNum==2)
	        {
	            endPoint.x=anchorPoint.x-RoD;
	            endPoint.y=anchorPoint.y;
	        }
	        //ew
	        else if(lineNum==3)
	        {
	            endPoint.x=anchorPoint.x;
	            endPoint.y=lineD.start.y+RoD+5;
	            westEastFlag=true;
	        }
	        //ns
	        else if(lineNum==4)
	        {
	            endPoint.x=anchorPoint.x-RoD;
	            endPoint.y=anchorPoint.y;
	        }
	        else
	        {
	            ROS_ERROR("error in north to south case");
	        }
	        gen_segment(anchorPoint, endPoint, wpSpace);
	        if(eastWestFlag)
	        {
		        anchorPoint.y=segment[segment.size()-1].y;
		        eastWestFlag=false;
	        }
	        else if(westEastFlag)
	        {	        	
		        anchorPoint.y=segment[segment.size()-1].y;
		        westEastFlag=false;
	        }
	        else
	        {
		        anchorPoint.x=segment[segment.size()-1].x;
		        anchorPoint.y=segment[segment.size()-1].y;	        	
	        }
	        lineNum=lineNum+1;
	        if(lineNum==5)
	        {
	            lineNum=1;
	        }
	        //should be a decent performance gain
	        fullSegment.reserve(segment.size());
	        for(int i=0; i<segment.size()-1; i++)
	        {
	        	fullSegment.push_back(segment[i]);
	        }
	    }
	}//end N to S case
	//end traj gen loops
	ROS_DEBUG("trajectory generated");
}

void high_level::gen_trajectory::gen_segment(geometry_msgs::Pose2D anchorPoint, geometry_msgs::Pose2D endPoint, double wpSpace)
{
	ROS_DEBUG("Generating Segment");
	segment.clear();
	//get num points for leg
    double distance=sqrt(pow((endPoint.x-anchorPoint.x),2)+pow((endPoint.y-anchorPoint.y),2));
	ROS_DEBUG("endPoint x and y %f, %f", endPoint.x, endPoint.y);
	ROS_DEBUG("anchorPoint x and y %f, %f", anchorPoint.x, anchorPoint.y);
    double numPoints=floor(distance/wpSpace);
	ROS_DEBUG("distance %f", distance);
	ROS_DEBUG("numPoints %f", numPoints);

    //check orientation
    segment.reserve(numPoints);
    geometry_msgs::Pose2D temp;

    //WE
    if(endPoint.y>anchorPoint.y)
    {
		ROS_DEBUG("Generating Segment WE");
        for(int i=1; i<numPoints-1; i++)
        {
            temp.x=anchorPoint.x;
            temp.y=anchorPoint.y+i*wpSpace;
        	temp.theta=1.5;
            //temp.theta=M_PI/2;
            segment.push_back(temp);
        }
        temp.x=endPoint.x;
        temp.y=endPoint.y-5;
        temp.theta=1.5;
        //temp.theta=M_PI/2;
        segment.push_back(temp);
    }
    //SN
    if(endPoint.x>anchorPoint.x)
    {
    	//temp.x=anchorPoint.x+5;//rounds the trajectory
        //temp.y=anchorPoint.y;
    	//temp.theta=1.5;	
        //segment.push_back(temp);
    	ROS_DEBUG("Generating Segment SN");
        for(int i=1; i<numPoints; i++)
        {
            temp.x=anchorPoint.x+i*wpSpace;
            temp.y=anchorPoint.y;
        	temp.theta=1.5;
            //temp.theta=0;  
            segment.push_back(temp);         
        }
        temp.x=endPoint.x-5;
        temp.y=endPoint.y;
        temp.theta=1.5;
        //temp.theta=0;
        segment.push_back(temp);
    }
    //EW
    if(endPoint.y<anchorPoint.y)
    {
		ROS_DEBUG("Generating Segment EW");
        for(int i=1; i<numPoints; i++)
        {
            temp.x=anchorPoint.x;
            temp.y=anchorPoint.y-i*wpSpace;
        	temp.theta=1.5;
            //temp.theta=-M_PI/2;   
            segment.push_back(temp);     
        }
        temp.x=endPoint.x;
        temp.y=endPoint.y+5;
        temp.theta=1.5;
        //temp.theta=-M_PI/2;
        segment.push_back(temp);
    }
    //NS
    if(endPoint.x<anchorPoint.x)
    {
    	anchorPoint.y=anchorPoint.y-wpSpace;//rounds the trajectory
		ROS_DEBUG("Generating Segment NS");
        for(int i=0; i<numPoints; i++)
        {
            temp.x=anchorPoint.x-i*wpSpace;
            temp.y=anchorPoint.y;
        	temp.theta=1.5;
            //temp.theta=M_PI;  
            segment.push_back(temp);          
        }
        temp.x=endPoint.x;
        temp.y=endPoint.y;
        temp.theta=1.5;
        //temp.theta=M_PI;
        segment.push_back(temp);
    }
    for(int i=0; i<segment.size();i++)
    {
			ROS_DEBUG("last bit of segment x and y %f, %f", segment[i].x, segment[i].y);
    }
}

bool high_level::gen_trajectory::generate(the_planner::initTraj::Request  &req, the_planner::initTraj::Response &res)
{  
	//res.initial_trajectory[0].x = req.pointA.x;
 	//res.initial_trajectory[0].y = req.pointA.y;

	ROS_DEBUG("fullSegment length is %lu", fullSegment.size());
	for(int i=0; i<fullSegment.size(); i++)
	{
		res.initial_trajectory.push_back(fullSegment[i]);	
	}
  	
 	return true;
}
