#include <the_planner/missions/dock.h>


dock::dock(ros::NodeHandle &nh) : Mission(nh)
{
    sleep(10);
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
    this->placard_sub    = mission_nh_->subscribe("/yolo_output", 10, &dock::placardCallback, this);
    this->task_sub    = mission_nh_->subscribe("/vrx/scan_and_dock/placard_symbol", 10, &dock::desiredPlacardCallback, this);
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
    if (isPlacard == true) {
        ROS_INFO("Use dock_path1................");
        // dock_path1.clear();
        dock_path1 = dock_path;
        dy = dock_path1[size_path-1] - dock_path1[1];
        dx = dock_path1[size_path-2] - dock_path1[0];
        sk_heading1 = atan2(dy, dx);
    } 
    else {
        ROS_INFO("Use dock_path2................");
        // dock_path2.clear();
        dock_path2 = dock_path;
        dy = dock_path2[size_path-1] - dock_path2[1];
        dx = dock_path2[size_path-2] - dock_path2[0];
        sk_heading2 = atan2(dy, dx);
    }
    got_dock = true;
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

void dock::placardCallback(const std_msgs::StringConstPtr &msg)
{
    ROS_INFO("placard_value = %s", msg->data.c_str());
    thePlacardString=msg->data;    
    got_placard = true;
}

void dock::desiredPlacardCallback(const std_msgs::StringConstPtr &msg)
{
    ROS_INFO("desired placard_value = %s", msg->data.c_str());
    theDesiredPlacardString = msg->data;
    got_desiredPlacard = true;
}

void dock::checkPlacard()
{
    isPlacard=thePlacardString.compare(theDesiredPlacardString);
}
//what needs to happen is a subscription to /vrx/tasks/placard and also /yolo_output
//once both of these items have successfully been called back, we will compare strings

void dock::loop()
{
    switch (this->task)
    {
        case START:
        {
            // Check if we have info regarding the dock
            // If we have received dock_path[8] it means we have found the dock
            // If NOT, move holding the initial heading for some distance to start moving along
            // a large circle in order to look for the dock in all directions
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in START <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            if (got_dock) {
                // Position the vehicke in front (and at a certain distance) of the closest docking station
                // Use path-following control and then station-keeping controller to face directly at the dock
                ROS_INFO("$$$$$$$$$$$$$$$$$$$$$ DOCK HAS BEEN FOUND $$$$$$$$$$$$$$$$$$$$$$$");
                this->task = DOCKPATH1_START;
                break;
            } 
            else {
                // Check if there is any visible part of the dock
                // If there is, compute a series of safe points around 
                // Perform the circular trajectory in order to search for the dock
                // When finding any part of the dock, compute a series of safe points around the area in order to map the dock
                ROS_INFO("The vehicle hasn't found dock yet...");
                this->task = CIRCLE_SEARCH;
                break;
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
            
            std::vector<double> the_path;
            
            while(ros::ok() && !got_placard && !got_desiredPlacard)
            {
                ros::spinOnce();
                ros::Rate loop_rate(60);
                loop_rate.sleep();
            }
            checkPlacard();
            if (isPlacard) {
                ROS_WARN("Woohoo! we found the gate, dock_path1");
                the_path = dock_path1;
            } else {
                ROS_WARN("Woohoo! we didn't found the gate, dock_path2");
                the_path = dock_path2;
            }

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

            skPoint.x = dock_path1[0];
            skPoint.y = dock_path1[1];
            sk1_heading = sk_heading1;
            skPoint.theta = sk1_heading;
            this->stationkeep_pub.publish(skPoint);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g] (deg)", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                if (isPlacard) {
                    this->task = DOCKPATH1_STOP;
                    ROS_INFO("************* DOCKPATH1_SK1 BREAK *****************");
                    break;
                }
                else {
                    this->task = GOTO_STATION2;
                    ROS_INFO("************* DOCKPATH1_SK1 BREAK *****************");
                    break;
                }
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

            geometry_msgs::Pose theGoal;
            theGoal.position.x = dock_path1[2];
            theGoal.position.y = dock_path1[3];
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
            
            skPoint.x = dock_path1[4];
            skPoint.y = dock_path1[5];
            sk2_heading = sk_heading1;
            skPoint.theta = sk2_heading;
            this->stationkeep_pub.publish(skPoint);

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
            // Break when info regarding placard has been acquired
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in DOCKPATH1_SK3 <<<<<<<<<<<<<<<<<<<<<<<<");
            ros::spinOnce();
            
            skPoint.x = dock_path1[6];
            skPoint.y = dock_path1[7];
            sk3_heading = sk_heading1;
            skPoint.theta = sk3_heading;
            this->stationkeep_pub.publish(skPoint);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g]", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                // Elapsed time
                ros::Time curr_time;
                curr_time = ros::Time::now();
                delta_t = curr_time - prev_time;
                ROS_INFO("ELAPSED TIME = %g", delta_t.toSec());
                if (delta_t.toSec() > 10) {
                    ROS_INFO("************* DOCKPATH1_SK3 BREAK *****************");
                    this->task = DOCKPATH1_SK1_REV;
                    break;
                }
                else {
                    this->task = DOCKPATH1_SK3;
                }
            }
            else {
                prev_time = ros::Time::now();
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
            
            skPoint.x = dock_path1[0];
            skPoint.y = dock_path1[1];
            skPoint.theta = sk1_heading;
            this->stationkeep_pub.publish(skPoint);

            double etat_sk[3];
            etat_sk[0] = x - skPoint.x;
            etat_sk[1] = y - skPoint.y;
            etat_sk[2] = piwrap(yaw_angle - skPoint.theta);
            double etat_3 = etat_sk[2]*180/M_PI;
            ROS_INFO("etat_sk = [%g, %g, %g]", etat_sk[0], etat_sk[1], etat_3);

            if ((abs(etat_sk[0])<tol_x) && (abs(etat_sk[1])<tol_y) && (abs(etat_3)<tol_psi)) {
                this->task = MISSION_FINISHED;
                ROS_INFO("************* DOCKPATH1_SK1_REV BREAK *****************");
                break;
            } 
            else {
                this->task = DOCKPATH1_SK1_REV;
            }
            break;
        }
        case CIRCLE_SEARCH:
        {
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in CIRCLE_SEARCH <<<<<<<<<<<<<<<<<<<<<<<<");
            break;
        }
        case GOTO_STATION2:
        {
            ROS_INFO(">>>>>>>>>>>>>>>>>>>>>> We are in GOTO_STATION2 <<<<<<<<<<<<<<<<<<<<<<<<");
            break;
        }
        case MISSION_FINISHED:
        {
            ROS_INFO("********************** MISSION_FINISHED ***************************");
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
