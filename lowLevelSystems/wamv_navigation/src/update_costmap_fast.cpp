#include <ros.ros.h>
#include <map_msgs/OccupancyGridUpdate.h>


void map_update_CB(map_msgs::OccupancyGridUpdateConstPtr msg)
{
    map_.data = msg->data;
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "update_costmap_fast");
    ros::NodeHandle node;

    ros::Publisher costmap_update_pub = node.advertise<map_msgs::OccupancyGridUpdate>
