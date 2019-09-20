#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <wamv_navigation/SendGoal.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>


/* This node receives a single goal position and produces a global trajectory
 * using the global planner from the move_base node.
 * Change the topic "/single_goal", to whatever message Travis is using to send the
 * goal poses. */

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class SendSingleGoal
{
    private:
        ros::NodeHandle node;
        ros::Subscriber new_goal_sub;
        ros::ServiceServer goal_service;
        double vehicle_pos_enu[2];
        double goal_enu[2];
        double goal_heading_enu;
        bool reached_wp;
        Client client;

    public:
        SendSingleGoal() : client("move_base", true)
        {
            vehicle_pos_enu[0] = 0.0;
            vehicle_pos_enu[1] = 0.0;
            goal_enu[0] = 0.0;
            goal_enu[1] = 0.0;
            goal_heading_enu = 0.0;
            reached_wp = false;
            goal_service = node.advertiseService<wamv_navigation::SendGoal::Request, wamv_navigation::SendGoal::Response>("send_goal", 
                                                                        boost::bind(&SendSingleGoal::sendGoalCallback, this, _1, _2));
            
            // Wait for the action server to come up
            while (!client.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Action server started, sending goal.");
        }

    private:
        bool sendGoalCallback(wamv_navigation::SendGoal::Request &req, wamv_navigation::SendGoal::Response &res)
        {
            // Current position of the vehicle. It is received in NED and
            // transformed to ENU:
            vehicle_pos_enu[0] = req.vehicle_pos.y;
            vehicle_pos_enu[1] = req.vehicle_pos.x;
            goal_enu[0]        = req.goal.position.y;
            goal_enu[1]        = req.goal.position.x;

            // Compute a suitable goal_heading
            double vect_goal[2] = {goal_enu[0] - vehicle_pos_enu[0], goal_enu[1] - vehicle_pos_enu[1]};
            goal_heading_enu = atan2(vect_goal[1], vect_goal[0]);
            goal_heading_enu = normalize_angle(goal_heading_enu);       // Converts from [-pi, pi] to [0, 2*pi] in ENU
            printf("<<<<<<<<<<goal_heading_enu = %g>>>>>>>>>>>>>\n", goal_heading_enu*180/M_PI);
            // Convert heading angle to quaternion
            geometry_msgs::Quaternion heading_quat_enu = tf::createQuaternionMsgFromYaw(goal_heading_enu);
            // Send service response
            res.goal_heading_enu = goal_heading_enu*180/M_PI;

            // Recompute goal in order to stop the vehicle a certain distance in advance
            double x_s, y_s = 0.0;
            if (req.dist_stop > 0) {
                double dist_stop = req.dist_stop;

                if (goal_heading_enu < M_PI/2.0) {
                    // The goal is in the first quadrant:
                    x_s = -dist_stop * cos(goal_heading_enu);
                    y_s = -dist_stop * sin(goal_heading_enu);
                }
                else if (goal_heading_enu < M_PI && goal_heading_enu > M_PI/2.0) {
                    // The goal is in the second quadrant:
                    double alpha2 = M_PI - goal_heading_enu;
                    x_s =  dist_stop * cos(alpha2);
                    y_s = -dist_stop * sin(alpha2);
                }
                else if (goal_heading_enu < 3*M_PI/2.0 && goal_heading_enu > M_PI) {
                    // The goal is in the third quadrant
                    double alpha3 = 3*M_PI/2.0 - goal_heading_enu;
                    x_s = dist_stop * cos(alpha3);
                    y_s = dist_stop * sin(alpha3);
                }
                else if (goal_heading_enu < 2*M_PI && goal_heading_enu > 3*M_PI/2.0) {
                    // The goal is in the fourth quadrant
                    double alpha4 = 2*M_PI - goal_heading_enu;
                    x_s = -dist_stop * cos(alpha4);
                    y_s =  dist_stop * sin(alpha4);
                }
                else {
                    if (goal_heading_enu == 0) x_s = -dist_stop;
                    else if (goal_heading_enu == M_PI/2.0) y_s = -dist_stop;
                    else if (goal_heading_enu == M_PI) x_s = dist_stop;
                    else if (goal_heading_enu == 3*M_PI/2.0) y_s = dist_stop;
                    else if (goal_heading_enu == 2*M_PI) x_s = -dist_stop;
                    else ROS_ERROR("Some error ocurred recomputing goal in path_to_goal.cpp");
                }
            }
            
            // Fill in the message here...
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id   = "wamv/odom";
            goal.target_pose.header.stamp      = ros::Time::now();
            goal.target_pose.pose.position.x   = goal_enu[0] + x_s;
            goal.target_pose.pose.position.y   = goal_enu[1] + y_s;
            goal.target_pose.pose.orientation  = heading_quat_enu;

            client.sendGoal(goal,
                            boost::bind(&SendSingleGoal::doneGoal, this, _1));  // "_1" comes from doneGoal() having only one argument

            return true;
        }
        
    public:
        void doneGoal(const actionlib::SimpleClientGoalState &state)
        {
            // Check if vehicle has reached the goal pose
            if (client.getState() == state.SUCCEEDED) {
                ROS_INFO(">>>>>> The vehicle reached the waypoint <<<<<<");
            }
            else {
                ROS_INFO("The vehicle failed to move");
            }
            ROS_INFO("Finished in state [%s]", state.toString().c_str());
            return;
            // ros::shutdown();
        }

        double normalize_angle(double x)
        {
            // This function converts the ENU heading from [-pi, pi] range 
            // given from applying atan2(), into [0, 2*pi] range.
            x = fmod(x, 2*M_PI);
            if (x < 0)
                x += 2*M_PI;

            return x;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_goal");
    SendSingleGoal sendSingleGoal;
    ros::spin();
    return 0;
}



