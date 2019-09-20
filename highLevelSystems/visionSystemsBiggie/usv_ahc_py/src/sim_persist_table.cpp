#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "sim_persistance_table");
    ros::NodeHandle node;
    ros::Publisher totems_pub = node.advertise<visualization_msgs::MarkerArray>("totems", 10);
    ros::Rate loop_rate(5);

    visualization_msgs::MarkerArray markerArr;

    while (ros::ok()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "totem";
        marker.id = 0;
        marker.type = 3;
        marker.action = 0;
        marker.lifetime = ros::Duration::Duration(0.5);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.position.x = 30;
        marker.pose.position.y = 30;
        marker.pose.position.z = 0.2;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1;
        markerArr.markers.push_back(marker);
        totems_pub.publish(msg);

        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "totem";
        marker.id = 0;
        marker.type = 3;
        marker.action = 0;
        marker.lifetime = ros::Duration::Duration(0.5);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.position.x = 30;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0.2;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1;
        markerArr.markers.push_back(marker);
        totems_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

