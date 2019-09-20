#include <the_planner/missions/totems.h>


totems::totems(ros::NodeHandle &nh) : Mission(nh)
{
    // Some circle parameters
    radius_circle = 5.0;
    delta_radius_circle = 3.0;

    // Totems positions in NED
    totem1_pos[0] = 30.0; 
    totem1_pos[1] = 30.0;
    totem2_pos[0] = 30.0; 
    totem2_pos[1] = 0.0;

    // Start and Finish goals definition
    goal_start[0] = 10;
    goal_start[1] = 25;
    goal_finish[0] = 0.0;
    goal_finish[1] = 0.0;

    // Navigation direction through circular path
    clockwise[0] = true;
    clockwise[1] = false;
    
    goal_tol = 2.0;
    this->task = START;
    ROS_INFO("totems has started");
    goal_reached = false;
    previous_status = 1;
    this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("send_goal");
    this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("circle_target");
    this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 1, &totems::goalStatusCallback, this);
    this->path_ned_pub    = mission_nh_->advertise<custom_messages_biggie::waypoint_array>("waypoint_array", 50);
    this->state_ned_sub   = mission_nh_->subscribe("/p3d_wamv_ned", 10, &totems::vehiclePosCallback, this);
}

totems::~totems()
{

}

void totems::goalStatusCallback(move_base_msgs::MoveBaseActionResult msg)
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

void totems::vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // These coordinates are in NED
    vehicle_pos[0] = msg->pose.pose.position.x;
    vehicle_pos[1] = msg->pose.pose.position.y;
}

void totems::loop()
{
    switch (this->task)
    {
        case START:
        {
            // Go to the starting waypoint of the totems field.
            // This is only for the qualifyings.  On finals the totems
            // will be randomly dispossed in the field, so the vehicle
            // will need to circle them on-the-go
            ROS_INFO("The totems has started...");

            // Define message and fill it up with the coordinates of the totems entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = goal_start[0];
            theGoal.position.y = goal_start[1];

            ros::spinOnce();
            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 0.0;
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
                ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = GOTO_TOTEM_1;
            }
            break;
        }
        case GOTO_TOTEM_1:
        {
            ROS_INFO("Starting routine; approach to first totem...");
            geometry_msgs::Pose totem_goal;
            // This is the actual position of the centroid of the totem
            totem_goal.position.x = totem1_pos[0];
            totem_goal.position.y = totem1_pos[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = totem_goal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
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
                ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = GOTO_TOTEM_1;
            }
            else {
                goal_reached = false;
                this->task = CIRCLE_TOTEM_1;
            }
            break;
        }
        case CIRCLE_TOTEM_1:
        {
            ROS_INFO("Starting routine to circle first totem...");
            // Define all the variables in the service message according to NED convention
            wamv_navigation::CircleTarget circle_srv;
            circle_srv.request.radius        = radius_circle;
            circle_srv.request.clockwise     = clockwise[0];            // clockwise is positive in NED
            circle_srv.request.vehicle_pos.x = vehicle_pos[0];
            circle_srv.request.vehicle_pos.y = vehicle_pos[1];
            circle_srv.request.target_pos.x  = totem1_pos[0];
            circle_srv.request.target_pos.y  = totem1_pos[1];
            circle_srv.request.num_wp        = 7;
            circle_srv.request.return_wp     = 5;   // 5
            ROS_INFO("vehicle_pos_x = %g, vehicle_pos_y = %g", circle_srv.request.vehicle_pos.x, circle_srv.request.vehicle_pos.y);

            if (client_circle.call(circle_srv)) {
                ROS_INFO("Service call to circle-target server was successful");
            }
            else {
                ROS_INFO("Service call failed");
                finished = true;
            }

            // Publish next waypoint as goal, and check if was reached
            for (unsigned long i = 0; i < circle_srv.response.circle_path.poses.size(); i++) {
                // Check that current goal is closer than the next goal from the vehicle's position.
                // If not, reroute the vehicle to the next goal instead.
                double current_goal[2] = {circle_srv.response.circle_path.poses[i].position.x,
                                          circle_srv.response.circle_path.poses[i].position.y};
                double next_goal[2]    = {circle_srv.response.circle_path.poses[i+1].position.x,
                                          circle_srv.response.circle_path.poses[i+1].position.y};
                double dist_current_goal = sqrt(pow(vehicle_pos[0] - current_goal[0], 2) + pow(vehicle_pos[1] - current_goal[1], 2));
                double dist_next_goal    = sqrt(pow(vehicle_pos[0] - next_goal[0], 2) + pow(vehicle_pos[1] - next_goal[1], 2));

                // if (dist_next_goal < dist_current_goal) {
                //     ROS_INFO("Rerouting vehicle to the next_goal instead");
                //     continue;
                // }

                wamv_navigation::SendGoal goto_srv;
                goto_srv.request.goal = circle_srv.response.circle_path.poses[i];
                goto_srv.request.vehicle_pos.x = vehicle_pos[0];
                goto_srv.request.vehicle_pos.y = vehicle_pos[1];
                goto_srv.request.dist_stop     = 0.0;
                ros::spinOnce();
                ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

                if (client_goal.call(goto_srv)) {
                    ROS_INFO("Service call to send-goal server was successful");
                    ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
                    printf("circle-goal from server = [\n");
                    printf("goal_x = %g\n", circle_srv.response.circle_path.poses[i].position.x);
                    printf("goal_y = %g\n", circle_srv.response.circle_path.poses[i].position.y);
                    printf("goal_orient_z = %g\n", circle_srv.response.circle_path.poses[i].orientation.z);
                    printf("goal_orient_w = %g\n", circle_srv.response.circle_path.poses[i].orientation.w);
                }
                else {
                    ROS_INFO("Service call failed");
                    finished = true;
                }

                previous_status = 1;
                while (goal_reached == false) {
                    ros::spinOnce();
                    ROS_INFO("The vehicle hasn't reached the goal yet");
                    sleep(1);
                }
                if (previous_status == 2) {
                    goal_reached = false;
                }
                else {
                    goal_reached = false;
                }
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = CIRCLE_TOTEM_1;
            }
            else {
                goal_reached = false;
                this->task = GOTO_TOTEM_2;
            }

            this->task = GOTO_TOTEM_2;
            break;
        }
        case GOTO_TOTEM_2:
        {
            ROS_INFO("Starting routine to approach second totem...");
            // Fill up goal message and request service
            geometry_msgs::Pose totem_goal;
            totem_goal.position.x    = totem2_pos[0];
            totem_goal.position.y    = totem2_pos[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = totem_goal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            else {
                ROS_INFO("Service call failed");
            }

            previous_status = 1;
            while (goal_reached == false) {
                ros::spinOnce();
                ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = GOTO_TOTEM_2;
            }
            else {
                goal_reached = false;
                this->task = CIRCLE_TOTEM_2;
            }
            break;
        }
        case CIRCLE_TOTEM_2:
        {
            ROS_INFO("Starting routine to circle second totem...");

            // Define all the variables in the service message according to NED convention
            wamv_navigation::CircleTarget circle_srv;
            circle_srv.request.radius        = radius_circle;
            circle_srv.request.clockwise     = clockwise[1];            // clockwise is positive in NED
            circle_srv.request.vehicle_pos.x = vehicle_pos[0];
            circle_srv.request.vehicle_pos.y = vehicle_pos[1];
            circle_srv.request.target_pos.x  = totem2_pos[0];
            circle_srv.request.target_pos.y  = totem2_pos[1];
            circle_srv.request.num_wp        = 7;
            circle_srv.request.return_wp     = -1;   // 4
            ROS_INFO("vehicle_pos_x = %g, vehicle_pos_y = %g", circle_srv.request.vehicle_pos.x, circle_srv.request.vehicle_pos.y);

            if (client_circle.call(circle_srv)) {
                ROS_INFO("Service call to circle-target server was successful");
            }
            else {
                ROS_INFO("Service call to circle-target server failed");
                finished = true;
            }
            
            // Publish next waypoint as goal, and check if was reached
            for (unsigned long i = 0; i < circle_srv.response.circle_path.poses.size(); i++) {
                // Check that current goal is closer than the next goal from the vehicle's position.
                // If not, reroute the vehicle to the next goal instead.
                double current_goal[2] = {circle_srv.response.circle_path.poses[i].position.x,
                                          circle_srv.response.circle_path.poses[i].position.y};
                double next_goal[2]    = {circle_srv.response.circle_path.poses[i+1].position.x,
                                          circle_srv.response.circle_path.poses[i+1].position.y};
                double dist_current_goal = sqrt(pow(vehicle_pos[0] - current_goal[0], 2) + pow(vehicle_pos[1] - current_goal[1], 2));
                double dist_next_goal    = sqrt(pow(vehicle_pos[0] - next_goal[0], 2) + pow(vehicle_pos[1] - next_goal[1], 2));
                ROS_INFO("current_goal = [%g, %g]", current_goal[0], current_goal[1]);
                ROS_INFO("next_goal = [%g, %g]", next_goal[0], next_goal[1]);
                ROS_INFO("dist_current_goal = %g", dist_current_goal);
                ROS_INFO("dist_next_goal = %g", dist_next_goal);
                ros::spinOnce();

                // if (dist_next_goal < dist_current_goal) {
                //     ROS_INFO("Rerouting vehicle to the next_goal instead");
                //     continue;
                // }

                wamv_navigation::SendGoal goto_srv;
                goto_srv.request.goal = circle_srv.response.circle_path.poses[i];
                goto_srv.request.vehicle_pos.x = vehicle_pos[0];
                goto_srv.request.vehicle_pos.y = vehicle_pos[1];
                goto_srv.request.dist_stop     = 0.0;
                ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

                if (client_goal.call(goto_srv)) {
                    ROS_INFO("Service call to send-goal server was successful");
                    ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
                    printf("circle-goal from server = [\n");
                    printf("goal_x = %g\n", circle_srv.response.circle_path.poses[i].position.x);
                    printf("goal_y = %g\n", circle_srv.response.circle_path.poses[i].position.y);
                    printf("goal_orient_z = %g\n", circle_srv.response.circle_path.poses[i].orientation.z);
                    printf("goal_orient_w = %g\n", circle_srv.response.circle_path.poses[i].orientation.w);
                }
                else {
                    ROS_INFO("Service call failed");
                    finished = true;
                }

                previous_status = 1;
                while (goal_reached == false) {
                    ros::spinOnce();
                    ROS_INFO("The vehicle hasn't reached the goal yet");
                    sleep(1);
                }
                if (previous_status == 2) {
                    goal_reached = false;
                }
                else {
                    goal_reached = false;
                }
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = CIRCLE_TOTEM_1;
            }
            else {
                goal_reached = false;
                this->task = FINISH;
            }

            break;
        }
        case FINISH:
        {
            ROS_INFO(">>>>>>>>>>> Finishing mission <<<<<<<<<<<");
            sleep(1);
            // Define message and fill it up with the coordinates of the totems entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = goal_finish[0];
            theGoal.position.y = goal_finish[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 0.0;
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
                ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = FINISH;
            }
            else {
                goal_reached = false;
                finished = true;
                break;
            }
        }
    }
}
