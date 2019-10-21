#include<the_planner/missions/tester.h>

tester::tester(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	ROS_DEBUG("tester has started");
	task_subscriber = mission_nh_->subscribe("/vrx/task/info", 10, &tester::task_callback, this);
	station_keeping_subscriber = mission_nh_->subscribe("/vrx/station_keeping/goal", 10, &tester::sk_goal_callback, this);
	sk_publisher = mission_nh_->advertise<geometry_msgs::Pose2D>("/control_target", 10);
	tester_publisher = mission_nh_->advertise<std_msgs::Bool>("shouldIStart", 10);
	traj_client = mission_nh_->serviceClient<the_planner::initTraj>("gen_init_traj");
    newTask=false;
    
    mission_nh_->getParam("/wamv/robot_localization/navsat_transform_node/datum", datum);
    ROS_WARN("DATUM IS %f %f", datum[0], datum[1]);
    
}

tester::~tester()
{

}

void tester::task_callback(const vrx_gazebo::Task::ConstPtr& msg)
{
    ROS_DEBUG("New Task Received");
    theTaskMsg.name=msg->name;	
    theTaskMsg.state=msg->state;	
    theTaskMsg.ready_time=msg->ready_time;	
    theTaskMsg.running_time=msg->running_time;	
    theTaskMsg.elapsed_time=msg->elapsed_time;	
    theTaskMsg.remaining_time=msg->remaining_time;	
    theTaskMsg.timed_out=msg->timed_out;	
    theTaskMsg.score=msg->score;	

    if (theTaskMsg.name=="stationkeeping")
    {
        this->task=STATIONKEEP;
        newTask=true;
    }

}

void tester::sk_goal_callback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
    skGoalMsg.header=msg->header;
    skGoalMsg.pose=msg->pose;

    //convert GPS goal to NED goal
    nedPoint=this->Geo2NED(skGoalMsg.pose.position.latitude, skGoalMsg.pose.position.longitude, datum[0], datum[1]);

    skPoint.x=nedPoint.north;
    skPoint.y=nedPoint.east;

    ROS_INFO("The target in NED is %f %f", nedPoint.north, nedPoint.east);    

    skGoal=true;
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
            while(ros::ok())
            {
                ros::Rate loop_rate(4);
                loop_rate.sleep();
                sk_publisher.publish(skPoint);
            }

			this->task=START;
			break;
		}
		case WAYPOINTS:
		{
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
            //  array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].x);
            //  array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].y);
            //  array.waypoint_array.push_back(theTraj.response.initial_trajectory[i].theta);
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
