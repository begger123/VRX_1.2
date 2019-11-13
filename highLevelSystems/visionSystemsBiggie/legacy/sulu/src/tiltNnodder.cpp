#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "laser_assembler/AssembleScans.h"
#include "sensor_msgs/PointCloud.h"
#include <math.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tiltNnoddder");


  ros::NodeHandle n;

/* Instantiate laser assembler service */
 	ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans>  ("assemble_scans");
  laser_assembler::AssembleScans srv;

/* wait for laser assembler to get its shit together */
  ros::service::waitForService("assemble_scans");

/* Declare publisher for laser sweeps */
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("/laser_sweeps",10);

/* This publishes to the joint controller */
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/tilt_controller/command", 10);

/* Zero the servo position */
	std_msgs::Float64 servozero;
	servozero.data = 0;
	chatter_pub.publish(servozero);

/* Specify max forward and backward position of servo */
    std_msgs::Float64 servoforward;
    std_msgs::Float64 servobackward;
    servoforward.data = 0.15;
    servobackward.data = -0.10;


  while (ros::ok())
  {
    /* Start laser sweep here for forward stroke */
     srv.request.begin = ros::Time::now();

    /* Tilter' forrard! */
    chatter_pub.publish(servoforward);

    ros::Duration(.5).sleep();

    /* End forward laser sweep here, call laser assembler service
     then publish point cloud */
    srv.request.end = ros::Time::now();
    client.call(srv);
    cloud_pub.publish(srv.response.cloud);

    /* Begine new sweep for back stroke */
    srv.request.begin = ros::Time::now();

    /* Now tilter' back! */
    chatter_pub.publish(servobackward);

    ros::Duration(.5).sleep();

    /* end laser sweep, assemble cloud, and publish */
    srv.request.end = ros::Time::now();


    client.call(srv);
    cloud_pub.publish(srv.response.cloud);

    ros::spinOnce();


  }


  return 0;
}
