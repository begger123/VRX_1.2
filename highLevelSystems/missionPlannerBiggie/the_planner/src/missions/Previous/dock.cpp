#include <the_planner/missions/dock.h>


dock::dock(ros::NodeHandle &nh) : Mission(nh)
{
    sleep(1);
    ros::spinOnce();
    size_explore = 0;
    size_path = 0;
    this->task = START;
    // Services
    this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("send_goal");
    this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("circle_target");
    // Subscribers
    this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 10, &dock::goalStatusCallback, this);
    // this->state_ned_sub   = mission_nh_->subscribe("/p3d_wamv_ned", 1, &dock::vehiclePosCallback, this);
    this->pose_sub        = mission_nh_->subscribe("/vehicle_pose", 10, &dock::pose_callback, this);
    this->dockpath_sub    = mission_nh_->subscribe("/dock_path", 10, &dock::dockPathCallback, this);
    this->dockexplore_sub = mission_nh_->subscribe("/dock_explore", 10, &dock::dockExploreCallback, this);
    this->plackard_sub    = mission_nh_->subscribe("/plackard", 10, &dock::plackardCallback, this);
    // this->etat_sk_sub     = mission_nh_->subscribe("/etat_sk", 10, &dock::etatSKCallback, this);
    // Publishers
    this->stationkeep_pub = mission_nh_->advertise<geometry_msgs::Pose2D>("/control_target_sk", 10);
}

dock::~dock()
{

}

void dock::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    x = msg->x;
    y = msg->y;
    yaw_angle = msg->theta;
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

// void dock::vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg)
// {
//     // These coordinates are in NED
//     x = msg->pose.pose.position.x;
//     y = msg->pose.pose.position.y;
// }

void dock::dockPathCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    dock_path.clear();
    std::cout << "dock_path = [ ";
    for (int i=0; i<8; i++) {
        dock_path.push_back(msg->data[i]);
        std::cout << dock_path[i] << " ";
    }
    std::cout << "]" << std::endl;
    got_path_points = true;
    // size_path = (int)(sizeof(dock_path)/sizeof(dock_path[0]));
    // ROS_INFO("size_path = %d", size_path);
    size_path = dock_path.size();
    double dx, dy;
    if (isPlackard == true) {
        ROS_INFO("Use dock_path1................");
        // dock_path1.clear();
        dock_path1 = dock_path;
        dy = dock_path1[size_path-1] - dock_path1[1];
        dx = dock_path1[size_path-2] - dock_path1[0];
        sk_heading1 = atan2(dy, dx);
    } else {
        ROS_INFO("Use dock_path2................");
        // dock_path2.clear();
        dock_path2 = dock_path;
        dy = dock_path2[size_path-1] - dock_path2[1];
        dx = dock_path2[size_path-2] - dock_path2[0];
        sk_heading2 = atan2(dy, dx);
    }
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

// void dock::etatSKCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
// {
//     // Station-keep real-time error in (x,y,psi)
//     sk_etat[0] = msg->data[0];
//     sk_etat[1] = msg->data[1];
//     sk_etat[2] = msg->data[2];
//     got_etat = true;
// }

void dock::plackardCallback(const std_msgs::BoolConstPtr &msg)
{
    ROS_INFO("plackard_value = %d", msg->data);
    isPlackard = msg->data;
}

void dock::loop()
{
    switch (this->task)
    {
        case START:
        {
            // Go straight to the left until plackard is mapped
            // Break when dock_explore[8] points has been acquired
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in START <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            if (got_explore_points) {
                this->task = EXPLORE_DOCK;
                break;
            } 
            else {
                ROS_INFO("The vehicle hasn't found the plackard yet...");
                this->task = START;
            }
            break;
        }
        case EXPLORE_DOCK:
        {
            // Move the vehicle across the 4 dock_explore points in order to get a map of the dock 
            // // Break when the vehicle gets to last point and a dock_path[8] has been acquired 
            // ROS_INFO("GOT TO MAP_DOCK SEQUENCE, loop = %d", i);
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in EXPLORE_DOCK <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            int j = 0;
            for (int i=0; i<size_explore; i++) {
                ros::spinOnce();
                geometry_msgs::Pose theGoal;
                theGoal.position.x = dock_explore[j];
                theGoal.position.y = dock_explore[j+1];
                wamv_navigation::SendGoal goto_srv;
                goto_srv.request.goal          = theGoal;
                goto_srv.request.vehicle_pos.x = x;
                goto_srv.request.vehicle_pos.y = y;
                goto_srv.request.dist_stop     = 2.0;
                ROS_INFO("vehicle_pos = [%g, %g]", x, y);

                if (client_goal.call(goto_srv)) {
                    // ROS_INFO("Service call to send-goal server was successful");
                    // ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
                }
                else {
                    ROS_INFO("Service call failed");
                    finished = true;
                }
                previous_status = 1;
                while (goal_reached == false) {
                    ros::spinOnce();
                    // ROS_INFO("The vehicle hasn't reached the goal yet");
                    // sleep(0.3);
                }
                if (previous_status == 2) {
                    goal_reached = false;
                    this->task = EXPLORE_DOCK;
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
                        this->task = EXPLORE_DOCK;
                    }
                }
            }
            break;
        }
        case DOCKPATH1_START:
        {
            // Move to the begining of dock_path corresponding to first docking station
            // Station-keep facing the dock when it gets to the point
            // Break when position and heading is within a defined tolerance
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_START <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            
            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }

            geometry_msgs::Pose theGoal;
            theGoal.position.x = dock_path1[0];
            theGoal.position.y = dock_path1[1];
            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = x;
            goto_srv.request.vehicle_pos.y = y;
            goto_srv.request.dist_stop     = 4.0;
            ROS_INFO("vehicle_pos = [%g, %g]", x, y);

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
                // sleep(0.3);
            }
            if (previous_status == 2) {
                goal_reached = false;
                this->task = DOCKPATH1_START;
            }
            else {
                // The vehicle has actually reached the goal
                this->task = DOCKPATH1_SK1;
                goal_reached = false;
                break;
            }
            break;
        }
        case DOCKPATH1_SK1:
        {
            // Station-keep in the start point of dock-path
            // Break when sufficiently close to desired values according to sk_etat[3] and predefined tolerances
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_SK1 <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }

            skPoint.x = dock_path1[0];
            skPoint.y = dock_path1[1];
            sk1_heading = sk_heading1;
            skPoint.theta = sk1_heading;
            this->stationkeep_pub.publish(skPoint);

            // double etat_3 = piwrap(sk_etat[2]);
            // etat_3 = etat_3*180/M_PI;
            // double etat_3 = sk_etat[2]*180/M_PI;

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g] (deg)", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                this->task = DOCKPATH1_STOP;
                ROS_INFO("************* DOCKPATH1_SK1 BREAK *****************");
                break;
            } 
            else {
                this->task = DOCKPATH1_SK1;
            }
            break;
        }
        case DOCKPATH1_STOP:
        {
            // Move vehicle accross dock_path until it gets to the stop point and break
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_STOP <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }

            geometry_msgs::Pose theGoal;
            theGoal.position.x = dock_path1[0];
            theGoal.position.y = dock_path1[1];
            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = x;
            goto_srv.request.vehicle_pos.y = y;
            goto_srv.request.dist_stop     = 1.0;
            ROS_INFO("vehicle_pos = [%g, %g]", x, y);

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
                // sleep(0.3);
            }
            if (previous_status == 2) {
                goal_reached = false;
                this->task = DOCKPATH1_STOP;
            } 
            else {
                // The vehicle has actually reached the goal
                this->task = DOCKPATH1_SK2;
                goal_reached = false;
                break;
            }
            break;
        }
        case DOCKPATH1_SK2:
        {
            // Station-keep the vehicle to the sk2 point (just before final station-keep docking point)
            // Break when position and heading is within a defined tolerance
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_SK2 <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }
            
            skPoint.x = dock_path1[4];
            skPoint.y = dock_path1[5];
            sk2_heading = sk_heading1;
            skPoint.theta = sk2_heading;
            this->stationkeep_pub.publish(skPoint);

            // double etat_3 = piwrap(sk_etat[2]);
            // etat_3 = etat_3*180/M_PI;  // Convert to degrees
            // double etat_3 = sk_etat[2]*180/M_PI;
            // ROS_INFO("ETAT_3 = %g", etat_3);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g]", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                this->task = DOCKPATH1_SK3;
                ROS_INFO("************* DOCKPATH1_SK2 BREAK *****************");
                break;
            } 
            else {
                this->task = DOCKPATH1_SK2;
            }
            break;
        }
        case DOCKPATH1_SK3:
        {
            // Station-keep the vehicle to the final docking point
            // Break when info regarding plackard has been acquired
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_SK3 <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }
            
            skPoint.x = dock_path1[6];
            skPoint.y = dock_path1[7];
            sk3_heading = sk_heading1;
            skPoint.theta = sk3_heading;
            this->stationkeep_pub.publish(skPoint);

            // double etat_3 = piwrap(sk_etat[2]);
            // etat_3 = etat_3*180/M_PI;  // Convert to degrees
            // ros::spinOnce();
            // double etat_3 = sk_etat[2]*180/M_PI;
            // ROS_INFO("ETAT_3 = %g", etat_3);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g]", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                if (isPlackard == false) {
                    ROS_INFO("************* DOCKPATH1_SK3 BREAK *****************");
                    this->task = DOCKPATH1_SK1_REV;
                    break;
                }
                else {
                    this->task = DOCKPATH1_SK3;
                }
            }
            else {
                this->task = DOCKPATH1_SK3;
            }
            break;
        }
        case DOCKPATH1_SK1_REV:
        {
            // Station keep the vehicle to the start point (the vehicle will move in reverse from the docking to the start point)
            // Break when position and heading is within a defined tolerance
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_SK1_REV <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path2;
            // } else {
            //     the_path = dock_path1;
            // }
            
            skPoint.x = dock_path1[0];
            skPoint.y = dock_path1[1];
            skPoint.theta = sk1_heading;
            this->stationkeep_pub.publish(skPoint);

            // double etat_3 = piwrap(sk_etat[2]);
            // etat_3 = etat_3*180/M_PI;  // Convert to degrees
            // double etat_3 = sk_etat[2]*180/M_PI;
            // ROS_INFO("sk_etat = [%g, %g, %g]", sk_etat[0], sk_etat[1], sk_etat[2]*180/M_PI);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g]", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                ros::spinOnce();
                // this->task = EXPLORE_DOCK2;
                this->task = DOCKPATH1_SK1_REV;
                ROS_INFO("************* DOCKPATH1_SK1_REV BREAK *****************");
                break;
            } 
            else {
                this->task = DOCKPATH1_SK1_REV;
                ros::spinOnce();
            }
            break;
        }
        case EXPLORE_DOCK2:
        {
            // Move the vehicle across the top 2 points of the dock_explore points in order to move to the second docking station
            // Break when the vehicle gets to the second point
            // ROS_INFO("GOT TO MAP_DOCK SEQUENCE, loop = %d", i);
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in EXPLORE_DOCK2 <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();

            // std::vector<double> the_path;
            // if (isPlackard == true) {
            //     the_path = dock_path1;
            // } else {
            //     the_path = dock_path2;
            // }

            geometry_msgs::Pose theGoal;
            theGoal.position.x = dock_explore[0];
            theGoal.position.y = dock_explore[1];
            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = x;
            goto_srv.request.vehicle_pos.y = y;
            goto_srv.request.dist_stop     = 0.0;
            ROS_INFO("vehicle_pos = [%g, %g]", x, y);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            sleep(2);
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
            }
            if (previous_status == 2) {
                goal_reached = false;
                this->task = EXPLORE_DOCK2;
            }
            else {
                // The vehicle has actually reached the goal
                this->task = DOCKPATH2_START;
                goal_reached = false;
                break;
            }
            break;
        }
        case DOCKPATH2_START:
        {
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH2_START <<<<<<<<<<<<<<<<<<<<<<<<");
            break;
        }

        case EXPLORE_DOCK2:
        {
            // Move the vehicle across the 4 dock_explore points in order to get a map of the dock
            // // Break when the vehicle gets to last point and a dock_path[8] has been acquired
            // ROS_INFO("GOT TO MAP_DOCK SEQUENCE, loop = %d", i);
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in EXPLORE_DOCK <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            int j = 0;
            for (int i=0; i<size_explore; i++) {
                ros::spinOnce();
                geometry_msgs::Pose theGoal;
                theGoal.position.x = dock_explore[j];
                theGoal.position.y = dock_explore[j+1];
                wamv_navigation::SendGoal goto_srv;
                goto_srv.request.goal          = theGoal;
                goto_srv.request.vehicle_pos.x = x;
                goto_srv.request.vehicle_pos.y = y;
                goto_srv.request.dist_stop     = 0.0;
                ROS_INFO("vehicle_pos = [%g, %g]", x, y);

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
                    // sleep(0.3);
                }
                if (previous_status == 2) {
                    goal_reached = false;
                    this->task = EXPLORE_DOCK2;
                }
                else {
                    // The vehicle has actually reached the goal
                    if (i == 1) {
                        this->task = DOCKPATH2_START;
                        goal_reached = false;
                        break;
                    }
                    else {
                        j = j + 2;
                        goal_reached = false;
                        this->task = EXPLORE_DOCK2;
                    }
                }
            }
            break;
        }
    }
}

double dock::piwrap(double angle)
{
   angle = fmod(angle, 2*M_PI);
   if (angle >= 0 && angle <= M_PI) {
       return(angle);
   } else {
       return(angle - 2*M_PI);
   }
}

double dock::twopiwrap(double angle)
{
    angle = fmod(angle, 2*M_PI);
    return(angle);
}



