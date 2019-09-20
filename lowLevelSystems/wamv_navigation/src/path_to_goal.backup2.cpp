#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "wamv_navigation/SendGoal.h"

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
        bool reached_wp;
        Client client;

    public:
        SendSingleGoal() : client("move_base", true)
        {
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
            move_base_msgs::MoveBaseGoal goal;
            // Fill in the message here...
            goal.target_pose.header.frame_id    = "odom";
            goal.target_pose.header.stamp       = ros::Time::now();
            goal.target_pose.pose.position.x    = req.goal.position.x;
            goal.target_pose.pose.position.y    = req.goal.position.y;
            goal.target_pose.pose.orientation.z = req.goal.orientation.z;
            goal.target_pose.pose.orientation.w = req.goal.orientation.w;

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_goal");
    SendSingleGoal sendSingleGoal;
    ros::spin();
    return 0;
}



