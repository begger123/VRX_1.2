#include<the_planner/missions/tester.h>

tester::tester(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	ROS_DEBUG("tester has started");
	task_subscriber = mission_nh_->subscribe("/vrx/task/info", 10, &tester::task_callback, this);
	station_keeping_subscriber = mission_nh_->subscribe("/vrx/station_keeping/goal", 10, &tester::sk_goal_callback, this);
	waypoint_subscriber = mission_nh_->subscribe("/vrx/wayfinding/waypoints", 10, &tester::wp_goal_callback, this);
	sk_publisher = mission_nh_->advertise<geometry_msgs::Pose2D>("/control_target_sk", 10);
	tester_publisher = mission_nh_->advertise<std_msgs::Bool>("shouldIStart", 10);
	traj_client = mission_nh_->serviceClient<the_planner::initTraj>("gen_init_traj");
    newTask=false;
    
}

tester::~tester()
{

}

void tester::task_callback(const vrx_gazebo::Task::ConstPtr& msg)
{
    ROS_DEBUG("New Task Received High Level");
    theTaskMsg.name=msg->name;	
    theTaskMsg.state=msg->state;	
    theTaskMsg.ready_time=msg->ready_time;	
    theTaskMsg.running_time=msg->running_time;	
    theTaskMsg.elapsed_time=msg->elapsed_time;	
    theTaskMsg.remaining_time=msg->remaining_time;	
    theTaskMsg.timed_out=msg->timed_out;	
    theTaskMsg.score=msg->score;	

    if (theTaskMsg.name=="stationkeeping" && theTaskMsg.state=="ready")
    {
        this->task=STATIONKEEP;
        newTask=true;
        finished=false;
    }
    
    if (theTaskMsg.name=="wayfinding" && theTaskMsg.state=="ready")
    {
        this->task=WAYPOINTS;
        newTask=true;
        finished=false;
    }
    if (theTaskMsg.state=="finished")
    {
        finished=true;
    }
}

void tester::sk_goal_callback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
    mission_nh_->getParam("/wamv/robot_localization/navsat_transform_node/datum", datum);
    skGoalMsg.header=msg->header;
    skGoalMsg.pose=msg->pose;

    //convert GPS goal to NED goal
    nedPoint=this->Geo2NED(skGoalMsg.pose.position.latitude, skGoalMsg.pose.position.longitude, datum[0], datum[1]);

    skPoint.x=nedPoint.north;
    skPoint.y=nedPoint.east;

    skPoint.theta=tf::getYaw(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));

    //wraps heading between 0 and 2PI    
    //skPoint.theta=fmod(skPoint.theta, 2*M_PI); 

    // Convert yawAngle from ENU to NED convention
    //if (skPoint.theta < M_PI/2)
    //{
    //    skPoint.theta = M_PI/2 - skPoint.theta;
    //}
    //else 
    //{
    //    skPoint.theta = 5*M_PI/2 - skPoint.theta;
    //}

    ROS_INFO("The target in NED is %f %f %f", skPoint.x, skPoint.y, skPoint.theta);    
    skGoal=true;
}

void tester::wp_goal_callback(const geographic_msgs::GeoPath::ConstPtr& msg)
{
    mission_nh_->getParam("/wamv/robot_localization/navsat_transform_node/datum", datum);
    
    wpGoalMsg.header=msg->header;
    wpGoalMsg.poses=msg->poses;

    ROS_WARN("the number of waypoints is %lu", wpGoalMsg.poses.size()); 
    
    for (int i=0; i<wpGoalMsg.poses.size(); i++)
    {
       NED_struct tempPoint; 
       tempPoint=this->Geo2NED(wpGoalMsg.poses[i].pose.position.latitude, wpGoalMsg.poses[i].pose.position.longitude, datum[0], datum[1]);
       
       theArray.waypoint_array.push_back(tempPoint.north);// tempPoint.east, 1.5};
       theArray.waypoint_array.push_back(tempPoint.east);// tempPoint.east, 1.5};
       theArray.waypoint_array.push_back(2.0);// tempPoint.east, 1.5};
    }
    wpGoal=true;
}

NED_struct tester::Geo2NED(double lat, double lon, double latref, double lonref)
{
    //cout << "new lat " << lat << " new lon " << lon << " lat " << latref << " lon " << lonref << endl;
    NED_struct inner_struct;
    double Rne[3][3];
    float Pned[2];
    unsigned long long Rea = 6378137; //radius of earth in m
    double e = 0.08181919;

    //printf("Current Pose (%f, %f)\n", lat, lon);
    //printf("Referen Pose (%f, %f)\n", latref, lonref);

    //printf("Rea = %llu e = %f \n", Rea, e);

    //convert degrees to radians
    lat = lat*M_PI/180;
    lon = lon*M_PI/180;
    latref = latref*M_PI/180;
    lonref = lonref*M_PI/180;

    //imperfect ellipsoid
    double Ne = Rea/(sqrt((1-pow(e,2)*pow(sin(lat),2))));
    double Neref = Rea/(sqrt((1-pow(e,2)*pow(sin(latref),2))));


    //printf("Ne = %f  Neref = %f \n", Ne, Neref);

    //ECEF coordinates
    double Pe_xe = Ne*cos(lat)*cos(lon);
    double Pe_ye = Ne*cos(lat)*sin(lon);
    double Pe_ze = Ne*(1-pow(e,2))*sin(lat);

    //printf("Pe_xe = %f  Pe_ye = %f \n", Pe_xe, Pe_ye);

    double Peref_xe =  Ne*cos(latref)*cos(lonref);
    double Peref_ye =  Ne*cos(latref)*sin(lonref);
    double Peref_ze = Ne*(1-pow(e,2))*sin(latref);

    //printf("Peref_xe = %f  Peref_ye = %f \n", Peref_xe, Peref_ye);

    //printf("Pexdif = %f  Peydif = %f \n", (Pe_xe - Peref_xe), (Pe_ye - Peref_ye));

    //Rne transformation matrix
    Rne[0][0] = -1*sin(latref)*cos(lonref);
    Rne[0][1] = -1*sin(latref)*sin(lonref);
    Rne[0][2] = cos(latref);
    Rne[1][0] = -1*sin(lonref);
    Rne[1][1] = cos(lonref);
    Rne[1][2] = 0;
    Rne[2][0] = -1*cos(latref)*cos(lonref);
    Rne[2][1] = -1*cos(latref)*sin(lonref);
    Rne[2][2] = -1*sin(latref);

    //NED coordinates
    Pned[0] = (Pe_xe - Peref_xe)*Rne[0][0] + (Pe_ye - Peref_ye)*Rne[0][1] + (Pe_ze - Peref_ze)*Rne[0][2];
    Pned[1] = (Pe_xe - Peref_xe)*Rne[1][0] + (Pe_ye - Peref_ye)*Rne[1][1] + 0;

    //printf("Pned (%f, %f)\n", Pned[0], Pned[1]);
    inner_struct.north = Pned[0];
    inner_struct.east = Pned[1];
    //printf("Struct (%.10f, %.10f)\n", inner_struct.north, inner_struct.east);

    return inner_struct;
}

void tester::loop()
{
	switch(this->task)
	{		
		case START:
		{
			ROS_INFO("Welcome Friend.  Awaiting a command...");
            while(ros::ok() && !newTask)
            {
                ros::spinOnce();
            }
            newTask=false;
			break;
		}
		case STATIONKEEP:
		{
			ROS_INFO("Station Keeping command received.");
            while(ros::ok() && !skGoal)
            {
                ros::spinOnce();
            }
            //publish this sucker to the controller as long as needed
            while(ros::ok()&&!finished)
            {
                ros::Rate loop_rate(60);
                loop_rate.sleep();
                sk_publisher.publish(skPoint);
            }

			this->task=START;
			break;
		}
		case WAYPOINTS:
		{
            //directly to high_to_low_node
            custom_waypoint_array_publisher.publish(theArray);

            ROS_WARN("Publishing the array");
            while(ros::ok()&&!finished)
            {
                ros::Rate loop_rate(60);
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
