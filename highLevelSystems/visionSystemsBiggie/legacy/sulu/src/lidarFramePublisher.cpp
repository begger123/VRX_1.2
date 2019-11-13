#include "ros/ros.h"
#include "dynamixel_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include <iostream>

using namespace std;

namespace lidarOrigin
{
	double x,y,z;
}

void stateCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{ 
 
	/* Create tf transform for broadcast */

	static tf::TransformBroadcaster br; /* broadcaster */
	tf::Transform lidarTransform; /* actual transform to broadcast */
	
	tf::Quaternion q; /* quaternion that will hold current lidar position */
	q.setRPY(0,msg->current_pos,0); /* set pitch angle to lidar current pitch */
	
	lidarTransform.setOrigin(tf::Vector3(lidarOrigin::x, lidarOrigin::y, lidarOrigin::z));
	lidarTransform.setRotation(q); /* Feed quaternion into our lidarTransform object */
br.sendTransform(tf::StampedTransform(lidarTransform,ros::Time::now(),"world","laser") ); 

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "lidarFramePublisher");
	
	ros::NodeHandle n;
	n.param<double>("lidarOrigin/x", lidarOrigin::x, 0);
	n.param<double>("lidarOrigin/y", lidarOrigin::y, 0);
	n.param<double>("lidarOrigin/z", lidarOrigin::z, 0);


/* Get joint position from tilt_controler/state topic, call back function will create transform and broadcast it */

	ros::Subscriber sub = n.subscribe("tilt_controller/state", 10, stateCallback);

	ros::spin();

  	return 0;
}
