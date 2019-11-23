#include <the_planner/missions/dock.h>


dock::dock(ros::NodeHandle &nh) : Mission(nh)
{
    sleep(1);
    ros::spinOnce();
    // got_explore_points = false;
    // got_path_points = false;
    size_explore = 0;
    size_path = 0;
    sk_heading = 0;
    this->task = START;
    // Services
    this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("send_goal");
    this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("circle_target");
    // Subscribers
    this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 1, &dock::goalStatusCallback, this);
    this->state_ned_sub   = mission_nh_->subscribe("/p3d_wamv_ned", 10, &dock::vehiclePosCallback, this);
    this->dockpath_sub    = mission_nh_->subscribe("/dock_path", 10, &dock::dockPathCallback, this);
    this->dockexplore_sub = mission_nh_->subscribe("/dock_explore", 10, &dock::dockExploreCallback, this);
    this->plackard_sub    = mission_nh_->subscribe("/plackard", 10, &dock::plackardCallback, this);
    // this->etat_sk_sub     = mission_nh_->subscribe("/etat_sk", 10, &dock::etatSKCallback, this);
    // Publisher
    this->stationkeep_pub = mission_nh_->advertise<geometry_msgs::Pose2D>("/control_target_sk", 10);
}

dock::~dock()
{

}

void dock::goalStatusCallback(move_base_msgs::MoveBaseActionResult msg)
{
    ROS_INFO("previous_status = %d", previous_status);
    ROS_INFO("Goal Status = %u", msg.status.status);
    if (msg.status.status == 3) {
        goal_reached = true;
    }
    else if (msg.status.status == 2) {
        ROS_INFO("Status = %u", msg.status.status);
        goal_reached = false;
        previous_status = 2;
    }
}

void dock::vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // These coordinates are in NED
    vehicle_pos[0] = msg->pose.pose.position.x;
    vehicle_pos[1] = msg->pose.pose.position.y;
}

void dock::dockPathCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    std::cout << "dock_path = [ ";
    for (int i=0; i<8; i++) {
        dock_path[i] = msg->data[i];
        std::cout << dock_path[i] << " ";
    }
    std::cout << "]" << std::endl;
    got_path_points = true;
    size_path = (int)(sizeof(dock_path)/sizeof(dock_path[0]));
    // ROS_INFO("size_path = %d", size_path);
    double dy = dock_path[size_path-1] - dock_path[1];
    double dx = dock_path[size_path-2] - dock_path[0];
    sk_heading = atan2(dy, dx);
    ROS_INFO("sk_heading = %g", sk_heading);
}

void dock::dockExploreCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    std::cout << "dock_explore = [ ";
    for (int i=0; i<8; i++) {
        dock_explore[i] = msg->data[i];
        std::cout << dock_explore[i] << " ";
    }
    std::cout << "]" << std::endl;
    got_explore_points = true;
    size_explore = (int)(sizeof(dock_explore)/sizeof(dock_explore[0]));
    ROS_INFO("size_explore = %d", size_explore);
}

void dock::plackardCallback(const std_msgs::BoolConstPtr &msg)
{
    std::cout << "plackard_value = " << msg->data << std::endl;
}

// void dock::etatSKCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
// {
//     // Station-keep real-time error in (x,y,psi)
//     sk_etat[0] = msg->data[0];
//     sk_etat[1] = msg->data[1];
//     sk_etat[2] = msg->data[2];
//     got_etat = true;
// }

void dock::loop()
{
    switch (this->task)
    {
        case START:
        {
            // Go straight to the left until plackard is mapped
            // Break when dock_explore[8] points has been acquired
            ros::spinOnce();
            if (got_explore_points) {
                this->task = MAP_DOCK;
                break;
            } 
            else {
                ROS_INFO("The vehicle hasn't found the plackard yet...");
                this->task = MAP_DOCK;
                break;
            }
        }
        case MAP_DOCK:
        {
            ros::spinOnce();
            int j = 0;
            for (int i=0; i<size_explore; i++) {
                // Move the vehicle across the 4 dock_explore points in order to get a map of the dock
                // Break when the vehicle gets to last point and a dock_path[8] has been acquired 
                // ROS_INFO("GOT TO MAP_DOCK SEQUENCE, loop = %d", i);
                ros::spinOnce();
                geometry_msgs::Pose theGoal;
                theGoal.position.x = dock_explore[j];
                theGoal.position.y = dock_explore[j+1];
                wamv_navigation::SendGoal goto_srv;
                goto_srv.request.goal          = theGoal;
                goto_srv.request.vehicle_pos.x = vehicle_pos[0];
                goto_srv.request.vehicle_pos.y = vehicle_pos[1];
                goto_srv.request.dist_stop     = 2.0;
                ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

                if (client_goal.call(goto_srv)) {
                    ROS_INFO("Service call to send-goal server was successful");
                    ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
                }
                else {
                    ROS_INFO("Service call failed");
                    finished = true;
                }
                previous_status = 1;
                while (goal_reached == false) {
                    ros::spinOnce();
                    // ROS_INFO("The vehicle hasn't reached the goal yet");
                    sleep(1);
                }
                if (previous_status == 2) {
                    goal_reached = false;
                    this->task = START;
                }
                else {
                    // The vehicle has actually reached the goal
                    if (i == 3) {
                        this->task = DOCKPATH1_START;
                        goal_reached = false;
                        break;
                    }
                    else {
                        j = j + 2;
                        goal_reached = false;
                    }
                }
            }
        }
        case DOCKPATH1_START:
        {
            // Move to the begining of dock_path corresponding to first docking station
            // Station-keep facing the dock when it gets to the point
            // Break when position and heading is within a defined tolerance
            ros::spinOnce();
            geometry_msgs::Pose theGoal;
            theGoal.position.x = dock_path[0];
            theGoal.position.y = dock_path[1];
            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 5.0;
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            else {
                ROS_INFO("Service call failed");
                finished = true;
            }
            previous_status = 1;
            while (goal_reached == false) {
                ros::spinOnce();
                // ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }
            if (previous_status == 2) {
                goal_reached = false;
                this->task = START;
            }
            else {
                // The vehicle has actually reached the goal
                this->task = DOCKPATH1_START_SK;
                goal_reached = false;
                break;
            }
        }
        case DOCKPATH1_START_SK:
        {
            // Station-keep in the start point of dock-path
            // ROS_INFO("sk_etat = [%g, %g, %g]", sk_etat[0], sk_etat[1], sk_etat[2]);
            skPoint.x = dock_path[0];
            skPoint.y = dock_path[1];
            skPoint.theta = sk_heading;
            this->stationkeep_pub.publish(skPoint);

        }
        case DOCKPATH1_STOP:
        {
            // Move vehicle accross dock_path until it gets to the stop point and break
            break;
        }
        case DOCKPATH1_SK1:
        {
            // Station-keep the vehicle to the sk1 point (just before final station-keep docking point)
            // Break when position and heading is within a defined tolerance
            break;
        }
        case DOCKPATH1_SK2:
        {
            // Station-keep the vehicle to the final docking point
            // Break when info regarding plackard has been acquired
            break;
        }
        case DOCKPATH1_SK_START:
        {
            // Station keep the vehicle to the start point (the vehicle will move in reverse from the docking to the start point)
            // Break when position and heading is within a defined tolerance
            break;
        }
    }
}








/*     switch (this->task)
 *     {
 *         case START:
 *         {
 *             ROS_INFO("The docking operation has started has started...");
 *
 *             // do we have a map of the plackard already?
 *
 *
 *
 *
 *
 *             // move forward in a straight line until the lidar sees the plackard
 *
 *             // Define coordinates in NED convention
 *             geometry_msgs::Pose theGoal;
 *             theGoal.position.x = goal_start[0];
 *             theGoal.position.y = goal_start[1];
 *
 *             ros::spinOnce();
 *             wamv_navigation::SendGoal goto_srv;
 *             goto_srv.request.goal          = theGoal;
 *             goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             goto_srv.request.dist_stop     = 0.0;
 *             ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *             if (client_goal.call(goto_srv)) {
 *                 ROS_INFO("Service call to send-goal server was successful");
 *                 ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *             }
 *             else {
 *                 ROS_INFO("Service call failed");
 *                 finished = true;
 *             }
 *
 *             previous_status = 1;
 *             while (goal_reached == false) {
 *                 ros::spinOnce();
 *                 ROS_INFO("The vehicle hasn't reached the goal yet");
 *                 sleep(1);
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = START;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 this->task = GOTO_TOTEM_1;
 *             }
 *             break;
 *         }
 *         case GOTO_TOTEM_1:
 *         {
 *             ROS_INFO("Starting routine; approach to first totem...");
 *             geometry_msgs::Pose totem_goal;
 *             // This is the actual position of the centroid of the totem
 *             totem_goal.position.x = totem1_pos[0];
 *             totem_goal.position.y = totem1_pos[1];
 *
 *             wamv_navigation::SendGoal goto_srv;
 *             goto_srv.request.goal          = totem_goal;
 *             goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
 *             ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *             if (client_goal.call(goto_srv)) {
 *                 ROS_INFO("Service call to send-goal server was successful");
 *                 ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *             }
 *             else {
 *                 ROS_INFO("Service call failed");
 *                 finished = true;
 *             }
 *
 *             previous_status = 1;
 *             while (goal_reached == false) {
 *                 ros::spinOnce();
 *                 ROS_INFO("The vehicle hasn't reached the goal yet");
 *                 sleep(1);
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = GOTO_TOTEM_1;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 this->task = CIRCLE_TOTEM_1;
 *             }
 *             break;
 *         }
 *         case CIRCLE_TOTEM_1:
 *         {
 *             ROS_INFO("Starting routine to circle first totem...");
 *             // Define all the variables in the service message according to NED convention
 *             wamv_navigation::CircleTarget circle_srv;
 *             circle_srv.request.radius        = radius_circle;
 *             circle_srv.request.clockwise     = clockwise[0];            // clockwise is positive in NED
 *             circle_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             circle_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             circle_srv.request.target_pos.x  = totem1_pos[0];
 *             circle_srv.request.target_pos.y  = totem1_pos[1];
 *             circle_srv.request.num_wp        = 7;
 *             circle_srv.request.return_wp     = 5;   // 5
 *             ROS_INFO("vehicle_pos_x = %g, vehicle_pos_y = %g", circle_srv.request.vehicle_pos.x, circle_srv.request.vehicle_pos.y);
 *
 *             if (client_circle.call(circle_srv)) {
 *                 ROS_INFO("Service call to circle-target server was successful");
 *             }
 *             else {
 *                 ROS_INFO("Service call failed");
 *                 finished = true;
 *             }
 *
 *             // Publish next waypoint as goal, and check if was reached
 *             for (unsigned long i = 0; i < circle_srv.response.circle_path.poses.size(); i++) {
 *                 // Check that current goal is closer than the next goal from the vehicle's position.
 *                 // If not, reroute the vehicle to the next goal instead.
 *                 double current_goal[2] = {circle_srv.response.circle_path.poses[i].position.x,
 *                                           circle_srv.response.circle_path.poses[i].position.y};
 *                 double next_goal[2]    = {circle_srv.response.circle_path.poses[i+1].position.x,
 *                                           circle_srv.response.circle_path.poses[i+1].position.y};
 *                 double dist_current_goal = sqrt(pow(vehicle_pos[0] - current_goal[0], 2) + pow(vehicle_pos[1] - current_goal[1], 2));
 *                 double dist_next_goal    = sqrt(pow(vehicle_pos[0] - next_goal[0], 2) + pow(vehicle_pos[1] - next_goal[1], 2));
 *
 *                 // if (dist_next_goal < dist_current_goal) {
 *                 //     ROS_INFO("Rerouting vehicle to the next_goal instead");
 *                 //     continue;
 *                 // }
 *
 *                 wamv_navigation::SendGoal goto_srv;
 *                 goto_srv.request.goal = circle_srv.response.circle_path.poses[i];
 *                 goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *                 goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *                 goto_srv.request.dist_stop     = 0.0;
 *                 ros::spinOnce();
 *                 ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *                 if (client_goal.call(goto_srv)) {
 *                     ROS_INFO("Service call to send-goal server was successful");
 *                     ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *                     printf("circle-goal from server = [\n");
 *                     printf("goal_x = %g\n", circle_srv.response.circle_path.poses[i].position.x);
 *                     printf("goal_y = %g\n", circle_srv.response.circle_path.poses[i].position.y);
 *                     printf("goal_orient_z = %g\n", circle_srv.response.circle_path.poses[i].orientation.z);
 *                     printf("goal_orient_w = %g\n", circle_srv.response.circle_path.poses[i].orientation.w);
 *                 }
 *                 else {
 *                     ROS_INFO("Service call failed");
 *                     finished = true;
 *                 }
 *
 *                 previous_status = 1;
 *                 while (goal_reached == false) {
 *                     ros::spinOnce();
 *                     ROS_INFO("The vehicle hasn't reached the goal yet");
 *                     sleep(1);
 *                 }
 *                 if (previous_status == 2) {
 *                     goal_reached = false;
 *                 }
 *                 else {
 *                     goal_reached = false;
 *                 }
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = CIRCLE_TOTEM_1;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 this->task = GOTO_TOTEM_2;
 *             }
 *
 *             this->task = GOTO_TOTEM_2;
 *             break;
 *         }
 *         case GOTO_TOTEM_2:
 *         {
 *             ROS_INFO("Starting routine to approach second totem...");
 *             // Fill up goal message and request service
 *             geometry_msgs::Pose totem_goal;
 *             totem_goal.position.x    = totem2_pos[0];
 *             totem_goal.position.y    = totem2_pos[1];
 *
 *             wamv_navigation::SendGoal goto_srv;
 *             goto_srv.request.goal          = totem_goal;
 *             goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
 *             ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *             if (client_goal.call(goto_srv)) {
 *                 ROS_INFO("Service call to send-goal server was successful");
 *                 ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *             }
 *             else {
 *                 ROS_INFO("Service call failed");
 *             }
 *
 *             previous_status = 1;
 *             while (goal_reached == false) {
 *                 ros::spinOnce();
 *                 ROS_INFO("The vehicle hasn't reached the goal yet");
 *                 sleep(1);
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = GOTO_TOTEM_2;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 this->task = CIRCLE_TOTEM_2;
 *             }
 *             break;
 *         }
 *         case CIRCLE_TOTEM_2:
 *         {
 *             ROS_INFO("Starting routine to circle second totem...");
 *
 *             // Define all the variables in the service message according to NED convention
 *             wamv_navigation::CircleTarget circle_srv;
 *             circle_srv.request.radius        = radius_circle;
 *             circle_srv.request.clockwise     = clockwise[1];            // clockwise is positive in NED
 *             circle_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             circle_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             circle_srv.request.target_pos.x  = totem2_pos[0];
 *             circle_srv.request.target_pos.y  = totem2_pos[1];
 *             circle_srv.request.num_wp        = 7;
 *             circle_srv.request.return_wp     = -1;   // 4
 *             ROS_INFO("vehicle_pos_x = %g, vehicle_pos_y = %g", circle_srv.request.vehicle_pos.x, circle_srv.request.vehicle_pos.y);
 *
 *             if (client_circle.call(circle_srv)) {
 *                 ROS_INFO("Service call to circle-target server was successful");
 *             }
 *             else {
 *                 ROS_INFO("Service call to circle-target server failed");
 *                 finished = true;
 *             }
 *
 *             // Publish next waypoint as goal, and check if was reached
 *             for (unsigned long i = 0; i < circle_srv.response.circle_path.poses.size(); i++) {
 *                 // Check that current goal is closer than the next goal from the vehicle's position.
 *                 // If not, reroute the vehicle to the next goal instead.
 *                 double current_goal[2] = {circle_srv.response.circle_path.poses[i].position.x,
 *                                           circle_srv.response.circle_path.poses[i].position.y};
 *                 double next_goal[2]    = {circle_srv.response.circle_path.poses[i+1].position.x,
 *                                           circle_srv.response.circle_path.poses[i+1].position.y};
 *                 double dist_current_goal = sqrt(pow(vehicle_pos[0] - current_goal[0], 2) + pow(vehicle_pos[1] - current_goal[1], 2));
 *                 double dist_next_goal    = sqrt(pow(vehicle_pos[0] - next_goal[0], 2) + pow(vehicle_pos[1] - next_goal[1], 2));
 *                 ROS_INFO("current_goal = [%g, %g]", current_goal[0], current_goal[1]);
 *                 ROS_INFO("next_goal = [%g, %g]", next_goal[0], next_goal[1]);
 *                 ROS_INFO("dist_current_goal = %g", dist_current_goal);
 *                 ROS_INFO("dist_next_goal = %g", dist_next_goal);
 *                 ros::spinOnce();
 *
 *                 // if (dist_next_goal < dist_current_goal) {
 *                 //     ROS_INFO("Rerouting vehicle to the next_goal instead");
 *                 //     continue;
 *                 // }
 *
 *                 wamv_navigation::SendGoal goto_srv;
 *                 goto_srv.request.goal = circle_srv.response.circle_path.poses[i];
 *                 goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *                 goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *                 goto_srv.request.dist_stop     = 0.0;
 *                 ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *                 if (client_goal.call(goto_srv)) {
 *                     ROS_INFO("Service call to send-goal server was successful");
 *                     ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *                     printf("circle-goal from server = [\n");
 *                     printf("goal_x = %g\n", circle_srv.response.circle_path.poses[i].position.x);
 *                     printf("goal_y = %g\n", circle_srv.response.circle_path.poses[i].position.y);
 *                     printf("goal_orient_z = %g\n", circle_srv.response.circle_path.poses[i].orientation.z);
 *                     printf("goal_orient_w = %g\n", circle_srv.response.circle_path.poses[i].orientation.w);
 *                 }
 *                 else {
 *                     ROS_INFO("Service call failed");
 *                     finished = true;
 *                 }
 *
 *                 previous_status = 1;
 *                 while (goal_reached == false) {
 *                     ros::spinOnce();
 *                     ROS_INFO("The vehicle hasn't reached the goal yet");
 *                     sleep(1);
 *                 }
 *                 if (previous_status == 2) {
 *                     goal_reached = false;
 *                 }
 *                 else {
 *                     goal_reached = false;
 *                 }
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = CIRCLE_TOTEM_1;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 this->task = FINISH;
 *             }
 *
 *             break;
 *         }
 *         case FINISH:
 *         {
 *             ROS_INFO(">>>>>>>>>>> Finishing mission <<<<<<<<<<<");
 *             sleep(1);
 *             // Define message and fill it up with the coordinates of the dock entrance
 *             // Define coordinates in NED convention
 *             geometry_msgs::Pose theGoal;
 *             theGoal.position.x = goal_finish[0];
 *             theGoal.position.y = goal_finish[1];
 *
 *             wamv_navigation::SendGoal goto_srv;
 *             goto_srv.request.goal          = theGoal;
 *             goto_srv.request.vehicle_pos.x = vehicle_pos[0];
 *             goto_srv.request.vehicle_pos.y = vehicle_pos[1];
 *             goto_srv.request.dist_stop     = 0.0;
 *             ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
 *
 *             if (client_goal.call(goto_srv)) {
 *                 ROS_INFO("Service call to send-goal server was successful");
 *                 ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
 *             }
 *             else {
 *                 ROS_INFO("Service call failed");
 *                 finished = true;
 *             }
 *
 *             previous_status = 1;
 *             while (goal_reached == false) {
 *                 ros::spinOnce();
 *                 ROS_INFO("The vehicle hasn't reached the goal yet");
 *                 sleep(1);
 *             }
 *
 *             if (previous_status == 2) {
 *                 goal_reached = false;
 *                 this->task = FINISH;
 *             }
 *             else {
 *                 goal_reached = false;
 *                 finished = true;
 *                 break;
 *             }
 *         }
 *     } */
