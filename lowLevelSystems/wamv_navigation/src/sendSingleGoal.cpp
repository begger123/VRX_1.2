#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "wamv_navigation/SendGoal.h"

/* This node sends a single goal as a latched message */

// void goal_callback(move_base_msgs::MoveBaseActionFeedback msg)
// {
//     ROS_INFO("Goal Status = %d", msg.status.status);
// }

void goal_callback(move_base_msgs::MoveBaseActionResult msg)
{
    ROS_INFO("Goal Status = %u", msg.status.status);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_single_goal", 1);
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<wamv_navigation::SendGoal>("send_goal");
    // ros::Subscriber goal_feedback_sub = node.subscribe("/move_base/feedback", 10, goal_callback);
    ros::Subscriber goal_feedback_sub = node.subscribe("/move_base/result", 10, goal_callback);

    
    // Define message and fill it up...
    geometry_msgs::Pose theGoal;
    theGoal.position.x = 10;
    theGoal.position.y = 10;
    theGoal.orientation.z = 0;
    theGoal.orientation.w = 1.0;

    wamv_navigation::SendGoal service;
    service.request.goal = theGoal;

    if (client.call(service)) {
        ROS_INFO("Service call successful");
    }
    else {
        ROS_ERROR("Service call failed");
        return 1;
    }

    ros::spin();

    return 0;
}






// #include <ros/ros.h>
// #include <geometry_msgs/PoseArray.h>
// #include <unistd.h>
// #include "wamv_navigation/SendGoal.h"
//
// [> This node sends a single goal as a latched message <]
//
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "send_single_goal", 1);
//     ros::NodeHandle node;
//     ros::ServiceClient client = node.serviceClient<wamv_navigation::SendGoal>("send_goal");
//
//
//     // Define message and fill it up...
//     geometry_msgs::Pose theGoal;
//     theGoal.position.x = 10;
//     theGoal.position.y = 10;
//     theGoal.orientation.z = 0;
//     theGoal.orientation.w = 1.0;
//
//     wamv_navigation::SendGoal service;
//     service.request.goal = theGoal;
//
//     if (client.call(service)) {
//         ROS_INFO("Service call successful");
//     }
//     else {
//         ROS_ERROR("Service call failed");
//         return 1;
//     }
//
//     return 0;
// }
