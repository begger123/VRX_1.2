#include <the_planner/missions/acoustics_channel.h>


acoustics_channel::acoustics_channel(ros::NodeHandle &nh) : Mission(nh)
{
    radius_circle = 6.0;
    delta_radius_circle = 3.0;
    marker_pos[0] = 0; 
    marker_pos[1] = 30;
    go_initial[0] = -8;
    go_initial[1] = 5;
    go_midpoint[0] = -8;
    go_midpoint[1] = 10;
    go_intermed_in[0] = -8;
    go_intermed_in[1] = 15;
    go_marker[0] = 0;
    go_marker[1] = 30;
    ret_intermed_in[0] = 8;
    ret_intermed_in[1] = 15;
    ret_midpoint[0] = 8;
    ret_midpoint[1] = 10;
    ret_finish[0] = 8;
    ret_finish[1] = 5;
    ret_home[0] = 0;
    ret_home[1] = 0;


    this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("/send_goal");
    this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("/circle_target");
    this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 1, &acoustics_channel::goalStatusCallback, this);
    this->path_ned_pub    = mission_nh_->advertise<custom_messages_biggie::waypoint_array>("/waypoint_array", 50);
    this->state_ned_sub   = mission_nh_->subscribe("/p3d_wamv_ned", 10, &acoustics_channel::vehiclePosCallback, this);
}

acoustics_channel::~acoustics_channel()
{

}

void acoustics_channel::goalStatusCallback(move_base_msgs::MoveBaseActionResult msg)
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

void acoustics_channel::vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // These coordinates are in NED
    vehicle_pos[0] = msg->pose.pose.position.x;
    vehicle_pos[1] = msg->pose.pose.position.y;
}


void acoustics_channel::loop()
{
    switch (this->task)
    {
        case START:
        {
            ROS_INFO("The acoustics_channel has started...");

            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = go_initial[0];
            theGoal.position.y = go_initial[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 0.0;
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = GOTO_MIDPOINT;
            }
            break;
        }
        case GOTO_MIDPOINT:
        {
            ROS_INFO("GOTO_MIDPOINT case has started");

            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = go_midpoint[0]; 
            theGoal.position.y = go_midpoint[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 0.0;
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = GOTO_MARKER;
            }
            break;
        }
        case GOTO_INTERMEDIATE_IN:
        {
            ROS_INFO("GOTO_MARKER case has started");
            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = go_intermed_in[0]; 
            theGoal.position.y = go_intermed_in[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = 0.0;
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = GOTO_MARKER;
            }
            break;
        }
        case GOTO_MARKER:
        {
            ROS_INFO("GOTO_MARKER case has started");
            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = marker_pos[0];
            theGoal.position.y = marker_pos[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = CIRCLE_MARKER;
            }
            break;
        }
        case CIRCLE_MARKER:
        {
            ROS_INFO("Starting routine to circle first totem...");
            // Define all the variables in the service message according to NED convention
            wamv_navigation::CircleTarget circle_srv;
            circle_srv.request.radius        = radius_circle;
            circle_srv.request.clockwise     = false;            // clockwise is positive in NED
            circle_srv.request.vehicle_pos.x = vehicle_pos[0];
            circle_srv.request.vehicle_pos.y = vehicle_pos[1];
            circle_srv.request.target_pos.x  = marker_pos[0];
            circle_srv.request.target_pos.y  = marker_pos[1];
            circle_srv.request.num_wp        = 7;
            circle_srv.request.return_wp     = 4;
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
                this->task = CIRCLE_MARKER;
            }
            else {
                goal_reached = false;
                this->task = RETURN_INTERMEDIATE_IN;
            }

            this->task = RETURN_INTERMEDIATE_IN;
            break;
        }
        case RETURN_INTERMEDIATE_IN:
        {
            ROS_INFO("GOTO_MARKER case has started");
            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = ret_intermed_in[0];
            theGoal.position.y = ret_intermed_in[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = RETURN_INTERMEDIATE_IN;
            }
            else {
                goal_reached = false;
                this->task = RETURN_MIDPOINT;
            }
            break;
        }
        case RETURN_MIDPOINT:
        {
            ROS_INFO("GOTO_MARKER case has started");
            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = ret_midpoint[0];
            theGoal.position.y = ret_midpoint[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
                this->task = START;
            }
            else {
                goal_reached = false;
                this->task = FINISH;
            }
            break;
        }
        case FINISH:
        {
            ROS_INFO("GOTO_MARKER case has started");
            // Define message and fill it up with the coordinates of the acoustics_channel entrance
            // Define coordinates in NED convention
            geometry_msgs::Pose theGoal;
            theGoal.position.x = ret_finish[0];
            theGoal.position.y = ret_finish[1];

            wamv_navigation::SendGoal goto_srv;
            goto_srv.request.goal          = theGoal;
            goto_srv.request.vehicle_pos.x = vehicle_pos[0];
            goto_srv.request.vehicle_pos.y = vehicle_pos[1];
            goto_srv.request.dist_stop     = radius_circle + delta_radius_circle;   // Allows the vehicle to stop before the circle area
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            if (client_goal.call(goto_srv)) {
                ROS_INFO("Service call to send-goal server was successful from START");
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
            }
            break;
        }
    }
}

