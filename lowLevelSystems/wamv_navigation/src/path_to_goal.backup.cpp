#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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
        Client client;

    public:
        SendSingleGoal() : client("move_base", true)
        {
            new_goal_sub = node.subscribe("/single_goal", 10, &SendSingleGoal::sendGoalCallback, this);
            
            // Wait for the action server to come up
            while (!client.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Action server started, sending goal.");
        }

    private:
        void sendGoalCallback(const geometry_msgs::Pose::ConstPtr& msg)
        {
            move_base_msgs::MoveBaseGoal goal;
            // Fill in the message here...
            goal.target_pose.header.frame_id    = "odom";
            goal.target_pose.header.stamp       = ros::Time::now();
            goal.target_pose.pose.position.x    = msg->position.x;
            goal.target_pose.pose.position.y    = msg->position.y;
            goal.target_pose.pose.orientation.z = msg->orientation.z;
            goal.target_pose.pose.orientation.w = msg->orientation.w;

            client.sendGoal(goal,
                            boost::bind(&SendSingleGoal::doneGoal, this, _1));  // "_1" comes from doneGoal() having only one argument
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
            // ros::shutdown();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_single_goal");
    SendSingleGoal sendSingleGoal;
    ros::spin();
    return 0;
}



