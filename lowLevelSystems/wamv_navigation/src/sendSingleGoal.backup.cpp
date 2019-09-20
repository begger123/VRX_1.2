#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>

/* This node sends a single goal as a latched message */

void connectCallback(const ros::SingleSubscriberPublisher &pub)
{
    geometry_msgs::Pose theGoal;
    theGoal.position.x = 30;
    theGoal.position.y = 30;
    theGoal.orientation.z = 0;
    theGoal.orientation.w = 1.0;
    pub.publish(theGoal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_single_goal", 1);
    ros::NodeHandle node;
    bool latched = true;
    ros::Publisher goal_pub = node.advertise<geometry_msgs::Pose>("single_goal", 1, latched);
    // ros::Rate loop_rate(1);
    
    while (goal_pub.getNumSubscribers() == 0) {
        ROS_ERROR("Waiting for subscribers.");
        sleep(1);
    }
    ROS_ERROR("Got subscriber");

    // Define message and fill it up...
    geometry_msgs::Pose theGoal;
    theGoal.position.x = 30;
    theGoal.position.y = 30;
    theGoal.orientation.z = 0;
    theGoal.orientation.w = 1.0;
    goal_pub.publish(theGoal);

    ros::spinOnce();

    // while (ros::ok()) {
    //     loop_rate.sleep();
    // }

    /* The commented block below is used whenever I want to send the goal
     * constantly to the move_base node, instead of sending it once and latching 
     * */

    // ros::Rate loop_rate(1);
    // geometry_msgs::Pose theGoal;
    // while (ros::ok()) {
    //     for (int i = 0; i < 4; i++) {
    //         theGoal.position.x = 30;
    //         theGoal.position.y = 30;
    //         theGoal.orientation.z = 0.0;
    //         theGoal.orientation.w = 1.0;
    //     }
    //     // ROS_INFO("Size of Array = %lu", theArray.poses.size());
    //     single_goal_pub.publish(theGoal);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    return 0;
}



