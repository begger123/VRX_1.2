#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>

/* This node is sends an array of goals, emulating the interplay with 
 * the real system onboard the vehicle. */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_array_goals", 10);
    ros::NodeHandle node;
    ros::Publisher array_goals_pub = node.advertise<geometry_msgs::PoseArray>("array_goals", 1, true);

    int array_goals[4][3] = {{15,0,0}, {30,0,0}, {30,15,0}, {30,-15,0}};
    geometry_msgs::PoseArray theArray; 
    ros::Rate loop_rate(10);

    // for (int i = 0; i < 4; i++) {
    //     geometry_msgs::Pose elem;
    //     elem.position.x = array_goals[i][0];
    //     elem.position.y = array_goals[i][1];
    //     elem.orientation.z = array_goals[i][2];
    //     elem.orientation.w = 1.0;
    //     theArray.poses.push_back(elem);
    // }
    // array_goals_pub.publish(theArray);
    // ROS_INFO("Array message has been published.");
    // ros::spinOnce();

    while (ros::ok()) {
        for (int i = 0; i < 4; i++) {
            geometry_msgs::Pose elem;
            elem.position.x = array_goals[i][0];
            elem.position.y = array_goals[i][1];
            elem.orientation.z = array_goals[i][2];
            elem.orientation.w = 1.0;
            theArray.poses.push_back(elem);
        }
        // ROS_INFO("Size of Array = %lu", theArray.poses.size());
        array_goals_pub.publish(theArray);
        theArray.poses.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}


