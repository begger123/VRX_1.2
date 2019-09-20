#include <the_planner/missions/totems.h>


totems::totems(ros::NodeHandle &nh) : Mission(nh)
{
    radius_circle = 5.0;
    delta_radius_circle = 3.0;
    totem1_pos[0] = 30.0; totem1_pos[1] = 30.0;
    totem2_pos[0] = 30.0; totem2_pos[1] = 0.0;
    goal_tol = 2.0;
    this->task = START;
    ROS_INFO("totems has started");

    totem_count = 0;
    idx = 0;
    // Totems sequence: according to the R-G-B order
    totems_order[0] = 0;              // Red totem is the first in the sequence
    totems_order[1] = 1;              // Green totem is the second in the sequence
    totems_order[2] = 2;              // Blue totem, discarded.

    // Totems directions: according to the R-G-B order
    clockwise[0] = true;            // Red totem goes in the clockwise direction
    clockwise[1] = false;           // Green totem goes in the counter-clockwise direction
    clockwise[2] = false;           // Blue totem goes in the counter-clockwise direction

    // Initialize totems indexes within the structure array
    totem_idx[0] = -1;
    totem_idx[1] = -1;
    totem_idx[2] = -1;

    // // Some fake data for testing purposes.
    // // Suppose only need to circle the Green and Blue totems
    // // Totem 1
    // totems_arr[0].totem_pos.x = 30;
    // totems_arr[0].totem_pos.y = 30;
    // totems_arr[0].color = VERDE;
    // totems_arr[0].clockwise = clockwise[1];
    // totems_arr[0].order = totems_order[1];
    // totems_arr[0].color_detected = true;
    //
    // // Totem 2
    // totems_arr[1].totem_pos.x = 30;
    // totems_arr[1].totem_pos.y = 0;
    // totems_arr[1].color = AZUL;
    // totems_arr[1].clockwise = clockwise[2];
    // totems_arr[1].order = totems_order[2];
    // totems_arr[1].color_detected = true;

    // Initialize the array of structures
    for (int i = 0; i < 3; i++) {
        totems_arr[i].totem_pos.x = -1.0;
        totems_arr[i].totem_pos.y = -1.0;
        totems_arr[i].clockwise = true;
        totems_arr[i].order = -1;
        totems_arr[i].color = -1;
        totems_arr[i].color_detected = false;
    }

    goal_reached = false;
    previous_status = 1;
    this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("/send_goal");
    this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("/circle_target");
    this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 1, &totems::goalStatusCallback, this);
    this->path_ned_pub    = mission_nh_->advertise<custom_messages_biggie::waypoint_array>("/waypoint_array", 50);
    this->state_ned_sub   = mission_nh_->subscribe("/p3d_wamv_ned", 10, &totems::vehiclePosCallback, this);
    this->totems_sub      = mission_nh_->subscribe("/markers", 10, &totems::totemStuffCallback, this);
}

totems::~totems()
{

}

void totems::totemStuffCallback(visualization_msgs::MarkerArray msg)
{
    // if (msg.markers[0].color.r == 1.0) {
    //     totems_arr[0].totem_pos.x = msg.markers[0].pose.position.x;
    //     totems_arr[0].totem_pos.y = msg.markers[0].pose.position.y;
    //     totems_arr[0].pos_detected = true;
    //     totems_arr[0].color = ROJO;
    //     totems_arr[0].clockwise = clockwise[0];
    //     totems_arr[0].order = totems_order[0];
    //     totems_arr[0].color_detected = true;
    // }
    // if (msg.markers[1].color.g == 1.0) {
    //     totems_arr[1].totem_pos.x = msg.markers[1].pose.position.x;
    //     totems_arr[1].totem_pos.y = msg.markers[1].pose.position.y;
    //     // totems_arr[1].pos_detected = true;
    //     // totems_arr[1].color = VERDE;
    //     // totems_arr[1].clockwise = clockwise[1];
    //     // totems_arr[1].order = totems_order[1];
    //     // totems_arr[1].color_detected = true;
    // }
    // if (msg.markers[2].color.b == 1.0) {
    //     totems_arr[2].totem_pos.x = msg.markers[2].pose.position.x;
    //     totems_arr[2].totem_pos.y = msg.markers[2].pose.position.y;
    //     totems_arr[2].pos_detected = true;
    //     totems_arr[2].color = AZUL;
    //     totems_arr[2].clockwise = clockwise[2];
    //     totems_arr[2].order = totems_order[2];
    //     totems_arr[2].color_detected = true;
    // }

    int k = 0;
    double totemPos0[2] = {0.0, 0.0};
    double totemPos1[2] = {0.0, 0.0};
    double totemPos2[2] = {0.0, 0.0};
    double totemCurr[2] = {0.0, 0.0};
    double dist_th = 1.5;
    int curr_color = -1;
    for (int i = 0; i < msg.markers.size(); i++) {
        if (msg.markers[i].ns == "totem") {
            // Extract color information from new data
            if (msg.markers[i].color.r == 1.0) {
                // ROS_INFO("The totem is RED");
                curr_color = 0;
            }
            else if (msg.markers[i].color.g == 1.0) {
                curr_color = 1;
            }
            else if (msg.markers[i].color.b == 1.0) {
                curr_color = 2;
            }
            else {
                curr_color = -1;
            }

            if (k == 0) {
                // Store position info
                totems_arr[k].totem_pos.x = msg.markers[i].pose.position.x;
                totems_arr[k].totem_pos.y = msg.markers[i].pose.position.y;
                totems_arr[k].pos_detected = true;
                if (curr_color > -1) {
                    totems_arr[k].color = curr_color;
                    totems_arr[k].clockwise = clockwise[0];
                    totems_arr[k].order = totems_order[0];
                    totems_arr[k].color_detected = true;
                }
                // ROS_INFO("Totem0 has been introduced in the structure array");
                // ROS_INFO("Totem0 position = [%g, %g]", totems_arr[0].totem_pos.x, totems_arr[0].totem_pos.y);
                k = 1;  // Next totem structure index in the structure array
            }
            else if (k == 1) {
                // Check if there is color info for first totem, and if it matches the current totem instance
                if (curr_color > -1 && totems_arr[0].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[0].totem_pos.x = totemCurr[0];
                    totems_arr[0].totem_pos.y = totemCurr[1];
                    totems_arr[0].pos_detected = true;
                    totems_arr[0].color = curr_color;
                    totems_arr[0].clockwise = clockwise[0];
                    totems_arr[0].order = totems_order[0];
                    totems_arr[0].color_detected = true;
                    continue;
                }
                else {
                    // Compare distance between current totem and totem0. Update value if they're almost the same
                    totemPos0[0] = totems_arr[0].totem_pos.x;
                    totemPos0[1] = totems_arr[0].totem_pos.y;
                    totemCurr[0] = msg.markers[i].pose.position.x;
                    totemCurr[1] = msg.markers[i].pose.position.y;
                    double dist = sqrt(pow(totemCurr[0] - totemPos0[0], 2) + pow(totemCurr[1] - totemPos0[1], 2));
                    if (dist < dist_th) {
                        // Update totem0 position
                        totems_arr[0].totem_pos.x = totemCurr[0];
                        totems_arr[0].totem_pos.y = totemCurr[1];
                        totems_arr[0].pos_detected = true;
                        continue;
                    }
                    else {
                        totems_arr[k].totem_pos.x = totemCurr[0];
                        totems_arr[k].totem_pos.y = totemCurr[1];
                        totems_arr[k].pos_detected = true;
                        if (curr_color > -1) {
                            totems_arr[k].color = curr_color;
                            totems_arr[k].clockwise = clockwise[k];
                            totems_arr[k].order = totems_order[k];
                            totems_arr[k].color_detected = true;
                        }
                        k = 2;
                    }
                }
            }
            else if (k == 2) {
                // Check if there is color info for first totem, and if it matches the current totem instance
                if (curr_color > -1 && totems_arr[0].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[0].totem_pos.x = totemCurr[0];
                    totems_arr[0].totem_pos.y = totemCurr[1];
                    totems_arr[0].color = curr_color;
                    totems_arr[0].color_detected = true;
                    totems_arr[0].clockwise = clockwise[0];
                    totems_arr[0].order = totems_order[0];
                    continue;
                }
                else if (curr_color > -1 && totems_arr[1].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[1].totem_pos.x = totemCurr[0];
                    totems_arr[1].totem_pos.y = totemCurr[1];
                    totems_arr[1].color = curr_color;
                    totems_arr[1].color_detected = true;
                    totems_arr[1].clockwise = clockwise[1];
                    totems_arr[1].order = totems_order[1];
                    continue;
                }
                else {
                    // Compare distance between current totems 0 and 1 . Update value if they're almost the same
                    totemPos0[0] = totems_arr[0].totem_pos.x;
                    totemPos0[1] = totems_arr[0].totem_pos.y;
                    totemPos1[0] = totems_arr[1].totem_pos.x;
                    totemPos1[1] = totems_arr[1].totem_pos.y;
                    totemCurr[0] = msg.markers[i].pose.position.x;
                    totemCurr[1] = msg.markers[i].pose.position.y;
                    double dist0 = sqrt(pow(totemCurr[0] - totemPos0[0], 2) + pow(totemCurr[1] - totemPos0[1], 2));
                    double dist1 = sqrt(pow(totemCurr[0] - totemPos1[0], 2) + pow(totemCurr[1] - totemPos1[1], 2));
                    if (dist0 < dist_th) {
                        // Update totem0 position
                        totems_arr[0].totem_pos.x = totemCurr[0];
                        totems_arr[0].totem_pos.y = totemCurr[1];
                        continue;
                    }
                    else if (dist1 < dist_th) {
                        // Update totem1 position
                        totems_arr[1].totem_pos.x = totemCurr[0];
                        totems_arr[1].totem_pos.y = totemCurr[1];
                        continue;
                    }
                    else {
                        totems_arr[k].totem_pos.x = totemCurr[0];
                        totems_arr[k].totem_pos.y = totemCurr[1];
                        if (curr_color > -1) {
                            totems_arr[k].color = curr_color;
                            totems_arr[k].clockwise = clockwise[0];
                            totems_arr[k].order = totems_order[0];
                            totems_arr[k].color_detected = true;
                        }
                        k = 3;
                    }
                }
            }
            else {
                // Check if there is color info for first totem, and if it matches the current totem instance
                if (curr_color > -1 && totems_arr[0].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[0].totem_pos.x = totemCurr[0];
                    totems_arr[0].totem_pos.y = totemCurr[1];
                    totems_arr[0].pos_detected = true;
                    totems_arr[0].color = curr_color;
                    totems_arr[0].clockwise = clockwise[0];
                    totems_arr[0].order = totems_order[0];
                    totems_arr[0].color_detected = true;
                    continue;
                }
                else if (curr_color > -1 && totems_arr[1].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[1].totem_pos.x = totemCurr[0];
                    totems_arr[1].totem_pos.y = totemCurr[1];
                    totems_arr[1].pos_detected = true;
                    totems_arr[1].color = curr_color;
                    totems_arr[1].clockwise = clockwise[1];
                    totems_arr[1].order = totems_order[1];
                    totems_arr[1].color_detected = true;
                    continue;
                }
                else if (curr_color > -1 && totems_arr[2].color == curr_color) {
                    // Is the same totem0 - Update totem0 position accordingly
                    totems_arr[2].totem_pos.x = totemCurr[0];
                    totems_arr[2].totem_pos.y = totemCurr[1];
                    totems_arr[2].pos_detected = true;
                    totems_arr[2].color = curr_color;
                    totems_arr[2].clockwise = clockwise[2];
                    totems_arr[2].order = totems_order[2];
                    totems_arr[2].color_detected = true;
                    continue;
                }
                else {
                    // Compare distance between current totem and totems 0 and 1 and 2 . Update value if they're almost the same
                    totemPos0[0] = totems_arr[0].totem_pos.x;
                    totemPos0[1] = totems_arr[0].totem_pos.y;
                    totemPos1[0] = totems_arr[1].totem_pos.x;
                    totemPos1[1] = totems_arr[1].totem_pos.y;
                    totemPos2[0] = totems_arr[1].totem_pos.x;
                    totemPos2[1] = totems_arr[1].totem_pos.y;
                    totemCurr[0] = msg.markers[i].pose.position.x;
                    totemCurr[1] = msg.markers[i].pose.position.y;
                    double dist0 = sqrt(pow(totemCurr[0] - totemPos0[0], 2) + pow(totemCurr[1] - totemPos0[1], 2));
                    double dist1 = sqrt(pow(totemCurr[0] - totemPos1[0], 2) + pow(totemCurr[1] - totemPos1[1], 2));
                    double dist2 = sqrt(pow(totemCurr[0] - totemPos2[0], 2) + pow(totemCurr[1] - totemPos2[1], 2));
                    if (dist0 < dist_th) {
                        // Update totem0 position
                        totems_arr[0].totem_pos.x = totemCurr[0];
                        totems_arr[0].totem_pos.y = totemCurr[1];
                        if (curr_color > -1) totems_arr[0].color = curr_color;
                        continue;
                    }
                    else if (dist1 < dist_th) {
                        // Update totem1 position
                        totems_arr[1].totem_pos.x = totemCurr[0];
                        totems_arr[1].totem_pos.y = totemCurr[1];
                        if (curr_color > -1) totems_arr[1].color = curr_color;
                        continue;
                    }
                    else if (dist2 < dist_th) {
                        // Update totem1 position
                        totems_arr[2].totem_pos.x = totemCurr[0];
                        totems_arr[2].totem_pos.y = totemCurr[1];
                        if (curr_color > -1) totems_arr[2].color = curr_color;
                        continue;
                    }
                    else {
                        ROS_INFO("***** The system couldn't classify the totem *****");
                        k = 3;
                    }
                }
            }
        }
    }
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
            theGoal.position.x = 10;
            theGoal.position.y = 25;

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

            // this->task = SEARCH_TOTEMS_1;
            // break;
            
            previous_status = 1;
            while (goal_reached == false && ros::ok()) {
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
                // this->task = GOTO_TOTEM_1;
                this->task = SEARCH_TOTEMS_1;
            }
            break;
        }
        case SEARCH_TOTEMS_1:
        {
            ROS_INFO("Start searching the first totem...");
            // i -> element-structure in the structure array
            // j -> color: {R, G, B}
            // k -> order value in the totem sequence
            bool no_totems = true;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 3; k++) {
                        if (totems_arr[i].color_detected) {
                            if (totems_arr[i].color == j && totems_order[j] == k) {
                                if (k == 0) {
                                    totem_idx[k] = i;
                                    totem_found[k] = true;
                                    // totem_found[i] = true;      // in the order of the structure array
                                    no_totems = false;
                                    printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                                    this->task = GOTO_TOTEM_1;
                                    break;
                                }
                                else {
                                    totem_found[k] = true;
                                    // totem_found[i] = true;      // in the order of the structure array
                                    totem_idx[k] = i;
                                    no_totems = false;
                                }
                            }
                        }
                    }
                }
            }
    
            if (totem_found[1] && totem_found[2]) {
                totem_idx[0] = 3 - (totem_idx[1] + totem_idx[2]);
                totem_found[0] = true;
                printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                this->task = GOTO_TOTEM_1;
                break;
            }

            // At this point we know we haven't got the first totem yet
            double next_goal[2] = {0.0, 0.0};
            int index_next_goal = -1;
            int M = 3;

            if (totem_found[1] == true) {
                // Measure distance between the other two totems left and go to the closest
                double totemA[2];
                double totemB[2];
                double dist_totemA = 1000;
                double dist_totemB = 1000;
                double idx_temp;

                // Get the position indexes of the other two structures in the array
                int totem1 = totem_idx[1];
                int totemL = (((totem1 - 1) % M) + M) % M;  // the structure index at the left  in the structure array
                int totemR = (((totem1 + 1) % M) + M) % M;  // the structure index at the right in the structure array

                if (totems_arr[totemL].pos_detected == true) {
                    totemA[0] = totems_arr[totemL].totem_pos.x;
                    totemA[1] = totems_arr[totemL].totem_pos.y;
                    dist_totemA = sqrt(pow(totemA[0] - vehicle_pos[0], 2) + pow(totemA[1] - vehicle_pos[1], 2));
                } 
                else if (totems_arr[totemR].pos_detected == true) {
                    totemB[0] = totems_arr[totemR].totem_pos.x;
                    totemB[1] = totems_arr[totemR].totem_pos.y;
                    dist_totemB = sqrt(pow(totemB[0] - vehicle_pos[0], 2) + pow(totemB[1] - vehicle_pos[1], 2));
                }
                else {
                    ROS_ERROR("We don't have the positions of the totems, keep searching");
                }

                // Compute the shortest distance
                if (dist_totemA < dist_totemB) {
                    next_goal[0] = totemA[0];
                    next_goal[1] = totemA[1];
                    index_next_goal =  totemL;
                }
                else {
                    next_goal[0] = totemB[0];
                    next_goal[1] = totemB[1];
                    index_next_goal = totemR;
                }

                // Go to the closest totem while collecting data and stopping when finding the actual first totem
                geometry_msgs::Pose totem_goal;
                totem_goal.position.x = next_goal[0];
                totem_goal.position.y = next_goal[1];

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

                // previous_status = 1;
                while (totems_arr[index_next_goal].color_detected == false && ros::ok()) {
                    ros::spinOnce();
                    ROS_INFO("The vehicle hasn't detected the color yet");
                    sleep(1);
                }

                // If color of the totem found corresponds to the first in the sequence, break
                // and go to the next switch case
                if (totems_arr[index_next_goal].color == totems_order[0]) {
                    totem_idx[0] = index_next_goal;
                    totem_found[0] = true;
                    printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                    this->task = GOTO_TOTEM_1;
                    break;
                }
                else {
                    totem_idx[2] = index_next_goal;
                    totem_found[2] = true;
                    totem_idx[0] = 3 - (totem_idx[1] + totem_idx[2]);
                    totem_found[0] = true;
                    printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                    this->task = GOTO_TOTEM_1;
                    break;
                }
            }

            else if (totem_found[2] == true) {
                // Measure distance between the other two totems left and go to the closest
                double totemA[2];
                double totemB[2];
                double dist_totemA = 1000;
                double dist_totemB = 1000;
                double idx_temp;

                // Get the position indexes of the other two structures in the array
                M = 3;
                int totem2 = totem_idx[2];
                int totemL = (((totem2 - 1) % M) + M) % M;  // the structure at the left  in the structure array
                int totemR = (((totem2 + 1) % M) + M) % M;  // the structure at the right in the structure array
                if (totems_arr[totemL].pos_detected == true) {
                    totemA[0] = totems_arr[totemL].totem_pos.x;
                    totemA[1] = totems_arr[totemL].totem_pos.y;
                    dist_totemA = sqrt(pow(totemA[0] - vehicle_pos[0], 2) + pow(totemA[1] - vehicle_pos[1], 2));
                } 
                else if (totems_arr[totemR].pos_detected == true) {
                    totemB[0] = totems_arr[totemR].totem_pos.x;
                    totemB[1] = totems_arr[totemR].totem_pos.y;
                    dist_totemB = sqrt(pow(totemB[0] - vehicle_pos[0], 2) + pow(totemB[1] - vehicle_pos[1], 2));
                }
                else {
                    ROS_ERROR("We don't have the positions of the totems, keep searching");
                }

                // Compute the shortest distance
                if (dist_totemA < dist_totemB) {
                    next_goal[0] = totemA[0];
                    next_goal[1] = totemA[1];
                    index_next_goal = totemL;
                }
                else {
                    next_goal[0] = totemB[0];
                    next_goal[1] = totemB[1];
                    index_next_goal = totemR;
                }
                
                // Go to the closest totem while collecting data and stopping when finding the actual first totem
                geometry_msgs::Pose totem_goal;
                totem_goal.position.x = next_goal[0];
                totem_goal.position.y = next_goal[1];

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
                while (totems_arr[index_next_goal].color_detected == false && ros::ok()) {
                    ros::spinOnce();
                    ROS_INFO("The vehicle hasn't detected the color yet");
                    sleep(1);
                }

                // If color of the totem found corresponds to the first in the sequence, break
                // and go to the next switch case
                if (totems_arr[index_next_goal].color == totems_order[0]) {
                    totem_idx[0] = index_next_goal;
                    totem_found[0] = true;
                    printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                    this->task = GOTO_TOTEM_1;
                    break;
                }
                else {
                    totem_idx[1] = index_next_goal;
                    totem_found[1] = true;
                    totem_idx[0] = 3 - (totem_idx[1] + totem_idx[2]);
                    totem_found[0] = true;
                    printf(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
                    this->task = GOTO_TOTEM_1;
                    break;
                }
            }
        }
        case GOTO_TOTEM_1:
        {
            ROS_INFO("Starting routine; approach the first totem in the sequence...");
            ROS_INFO("totem_idx[0] = %d", totem_idx[0]);
            ROS_INFO("First totem position = [%g, %g]", totems_arr[totem_idx[0]].totem_pos.x, totems_arr[totem_idx[0]].totem_pos.y);
            geometry_msgs::Pose totem_goal;
            totem_goal.position.x = totems_arr[totem_idx[0]].totem_pos.x;
            totem_goal.position.y = totems_arr[totem_idx[0]].totem_pos.y;

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = totem_goal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            // goto_srv.request.dist_stop     = radius_circle - 4;
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
            while (goal_reached == false && ros::ok()) {
                ros::spinOnce();
                ROS_INFO("The vehicle hasn't reached the goal yet");
                sleep(1);
            }

            if (previous_status == 2) {
                goal_reached = false;
                this->task = GOTO_TOTEM_1;
                break;
            }
            else {
                goal_reached = false;
                this->task = CIRCLE_TOTEM_1;
                break;
            }
        }
        case CIRCLE_TOTEM_1:
        {
            ROS_INFO("Starting routine to circle first totem...");
            // Define all the variables in the service message according to NED convention
            wamv_navigation::CircleTarget circle_srv;
            circle_srv.request.radius        = radius_circle;
            circle_srv.request.clockwise     = true;            // clockwise is positive in NED
            circle_srv.request.vehicle_pos.x = vehicle_pos[0];
            circle_srv.request.vehicle_pos.y = vehicle_pos[1];
            circle_srv.request.target_pos.x  = totem1_pos[0];
            circle_srv.request.target_pos.y  = totem1_pos[1];
            circle_srv.request.num_wp        = 7;
            circle_srv.request.return_wp     = 5;
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
                while (goal_reached == false && ros::ok()) {
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
                this->task = SEARCH_TOTEMS_2;
                break;
            }
        }
        case SEARCH_TOTEMS_2:
        {
            ROS_INFO("Starting routine; approach the second totem in the sequence...");
            // double next_goal[2] = {0.0, 0.0};
            // int index_next_goal = -1;
            // int M = 3;

            if (totem_found[1]) {
                this->task = GOTO_TOTEM_2;
                ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>> Totem 2 has been found <<<<<<<<<<<<<<<<<<<<<<");
                break;
            }
            else if(totem_found[2]) {
                // At this point we know that the totem left is actually the second totem
                // we are looking for
                totem_idx[1] = 3 - (totem_idx[0] + totem_idx[2]);
                this->task = GOTO_TOTEM_2;
                ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>> Totem 2 has been found <<<<<<<<<<<<<<<<<<<<<<");
                break;
            }

            // else if (totem_found[0] == true) {
            //     // Measure distance between the other two totems left and go to the closest
            //     double totemA[2];
            //     double totemB[2];
            //     double dist_totemA = 1000;
            //     double dist_totemB = 1000;
            //     double idx_temp;
            //
            //     // Get the position indexes of the other two structures in the array
            //     int totem0 = totem_idx[0];
            //     int totemL = (((totem0 - 1) % M) + M) % M;  // the structure index at the left  in the structure array
            //     int totemR = (((totem0 + 1) % M) + M) % M;  // the structure index at the right in the structure array
            //
            //     if (totems_arr[totemL].pos_detected == true) {
            //         totemA[0] = totems_arr[totemL].totem_pos.x;
            //         totemA[1] = totems_arr[totemL].totem_pos.y;
            //         dist_totemA = sqrt(pow(totemA[0] - vehicle_pos[0], 2) + pow(totemA[1] - vehicle_pos[1], 2));
            //     }
            //     else if (totems_arr[totemR].pos_detected == true) {
            //         totemB[0] = totems_arr[totemR].totem_pos.x;
            //         totemB[1] = totems_arr[totemR].totem_pos.y;
            //         dist_totemB = sqrt(pow(totemB[0] - vehicle_pos[0], 2) + pow(totemB[1] - vehicle_pos[1], 2));
            //     }
            //     else {
            //         ROS_ERROR("We don't have the positions of the totems, keep searching");
            //     }
            //
            //     // Compute the shortest distance
            //     if (dist_totemA < dist_totemB) {
            //         next_goal[0] = totemA[0];
            //         next_goal[1] = totemA[1];
            //         index_next_goal =  totemL;
            //     }
            //     else {
            //         next_goal[0] = totemB[0];
            //         next_goal[1] = totemB[1];
            //         index_next_goal = totemR;
            //     }
            //
            //     // Go to the closest totem while collecting data and stopping when finding the actual first totem
            //     geometry_msgs::Pose totem_goal;
            //     totem_goal.position.x = next_goal[0];
            //     totem_goal.position.y = next_goal[1];
            //
            //     wamv_navigation::SendGoal goto_srv;
            //     goto_srv.request.goal          = totem_goal;
            //     goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            //     goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            //     goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            //     ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
            //
            //     if (client_goal.call(goto_srv)) {
            //         ROS_INFO("Service call to send-goal server was successful");
            //         ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            //     }
            //     else {
            //         ROS_INFO("Service call failed");
            //         finished = true;
            //     }
            //
            //     // previous_status = 1;
            //     while (totems_arr[index_next_goal].color_detected == false && ros::ok()) {
            //         ros::spinOnce();
            //         ROS_INFO("The vehicle hasn't detected the color yet");
            //         sleep(1);
            //     }
            //
            //     // If color of the totem found corresponds to the first in the sequence, break
            //     // and go to the next switch case
            //     if (totems_arr[index_next_goal].color == totems_order[0]) {
            //         totem_idx[0] = index_next_goal;
            //         totem_found[0] = true;
            //         this->task = GOTO_TOTEM_1;
            //         ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
            //         break;
            //     }
            //     else {
            //         totem_idx[2] = index_next_goal;
            //         totem_found[2] = true;
            //         totem_idx[0] = 3 - (totem_idx[1] + totem_idx[2]);
            //         totem_found[0] = true;
            //         this->task = GOTO_TOTEM_1;
            //         ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>> Totem 1 has been found <<<<<<<<<<<<<<<<<<<<<<");
            //         break;
            //     }
            // }
            
            
        }
        case GOTO_TOTEM_2:
        {
            ROS_INFO("Starting routine to approach second totem...");
            ROS_INFO("Second totem position = [%g, %g]", totems_arr[totem_idx[1]].totem_pos.x, totems_arr[totem_idx[1]].totem_pos.y);
            // Fill up goal message and request service
            geometry_msgs::Pose totem_goal;
            totem_goal.position.x = totems_arr[totem_idx[1]].totem_pos.x;
            totem_goal.position.y = totems_arr[totem_idx[1]].totem_pos.y;

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = totem_goal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            // goto_srv.request.dist_stop     = radius_circle - 4;
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful");
                ROS_INFO("goal_heading_enu = %g deg", goto_srv.response.goal_heading_enu);
            }
            else {
                ROS_INFO("Service call failed");
            }

            previous_status = 1;
            while (goal_reached == false && ros::ok()) {
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
            circle_srv.request.clockwise     = false;            // clockwise is positive in NED
            circle_srv.request.vehicle_pos.x = vehicle_pos[0];
            circle_srv.request.vehicle_pos.y = vehicle_pos[1];
            circle_srv.request.target_pos.x  = totem2_pos[0];
            circle_srv.request.target_pos.y  = totem2_pos[1];
            circle_srv.request.num_wp        = 7;
            circle_srv.request.return_wp     = 4;
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
                while (goal_reached == false && ros::ok()) {
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
            theGoal.position.x = 0.0;
            theGoal.position.y = 0.0;

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
            while (goal_reached == false && ros::ok()) {
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
