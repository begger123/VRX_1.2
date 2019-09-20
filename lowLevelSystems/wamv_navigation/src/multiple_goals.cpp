#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>

/* This node receives an array of goals and produces a global trajectory
 * using the global planner from the move_base node, for each pair in the sequence.
 * Change the topic "/single_goal", to whatever message Travis is using to send the 
 * goal poses.
 */

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class ExploreMission
{
    private:
        ros::NodeHandle node;
        ros::Subscriber array_goals_sub;
        ros::Subscriber intrude_goal_sub;
        Client client;
        geometry_msgs::PoseArray goals_array;
        geometry_msgs::Pose current_goal;
        geometry_msgs::Pose current_pose;
        int array_size;
        int ind;    // index of the goals_array
        std::string itsArray;
        std::string itsIntrude;

    public:
        ExploreMission() : client("move_base", true)
        {
            ind = 0;
            array_size = 0;
            itsArray = "array";
            itsIntrude = "intrude";
            array_goals_sub = node.subscribe("/array_goals", 1, &ExploreMission::getMissionCallback, this);
            intrude_goal_sub = node.subscribe("/single_goal", 1, &ExploreMission::getGoalCallback, this);
            
            // Wait for the action server to come up
            while (!client.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Action server started, sending goal.");
        }

    private:
        void getMissionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
        {
            // Change array index when it has reached current goal
            array_size = msg->poses.size();
            ROS_INFO("Got array of goals...");
            ROS_INFO("Array size is: %d", array_size);

            for(int i = 0; i < array_size; i++) {
                goals_array.poses.push_back(msg->poses[i]);
            } 

            current_goal = goals_array.poses[ind];
            goto_next_goal(current_goal, itsArray);
            array_goals_sub.shutdown();
        }

        void getGoalCallback(const geometry_msgs::Pose::ConstPtr &msg)
        {
            current_goal.position.x = msg->position.x;
            current_goal.position.y = msg->position.y;
            current_goal.orientation.z = msg->orientation.z;
            current_goal.orientation.w = msg->orientation.w;
            goto_next_goal(current_goal, itsIntrude);
        }
        
    public:
        void goto_next_goal(geometry_msgs::Pose msg, std::string source)
        {
            move_base_msgs::MoveBaseGoal goal;
            // Fill in the message here...
            goal.target_pose.header.frame_id    = "wamv/odom";
            goal.target_pose.header.stamp       = ros::Time::now();
            goal.target_pose.pose.position.x    = msg.position.x;
            goal.target_pose.pose.position.y    = msg.position.y;
            goal.target_pose.pose.orientation.z = msg.orientation.z;
            goal.target_pose.pose.orientation.w = msg.orientation.w;

            if (source == "array") {
                client.sendGoal(goal,
                                boost::bind(&ExploreMission::doneArrayGoal, this, _1));
            }
            else {
                client.sendGoal(goal,
                                boost::bind(&ExploreMission::doneIntrudeGoal, this, _1));
            }
        }

        void activeGoal()
        {
            ROS_INFO("Goal just went active");
        }

        void feedbackGoal(const move_base_msgs::MoveBaseActionFeedback::ConstPtr & feedback)
        {
            float x = feedback->feedback.base_position.pose.position.x;
            float y = feedback->feedback.base_position.pose.position.y;
            float z = feedback->feedback.base_position.pose.orientation.z;
            float w = feedback->feedback.base_position.pose.orientation.w;

            current_pose.position.x = x;
            current_pose.position.y = y;
            current_pose.orientation.z = z;
            current_pose.orientation.w = w;

            ROS_INFO("Position = [%g, %g]", x, y);
        }

        void doneArrayGoal(const actionlib::SimpleClientGoalState &state)
        {
            // Check if vehicle has reached the goal pose
            if (client.getState() == state.SUCCEEDED) {
                ROS_INFO(">>>>>> The vehicle reached the waypoint <<<<<<");
            }
            else {
                ROS_INFO("The vehicle failed to move");
            }
            if (ind == array_size - 1) {
                ROS_INFO("Finished in state [%s]", state.toString().c_str());
                ros::shutdown();
            }
            ind++;
            goto_next_goal(goals_array.poses[ind], itsArray);
        }

        void doneIntrudeGoal(const actionlib::SimpleClientGoalState &state)
        {
            // Check if vehicle has reached the goal pose
            if (client.getState() == state.SUCCEEDED) {
                ROS_INFO(">>>>>> The vehicle reached the waypoint <<<<<<");
            }
            else {
                ROS_INFO("The vehicle failed to move");
            }
            goto_next_goal(goals_array.poses[ind], itsArray);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_single_goal");
    ExploreMission exploreMission;
    ros::spin();
    return 0;
}




