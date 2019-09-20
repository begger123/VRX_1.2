//This will be the code that calls the specified controller
#include "high_to_low/waypoints_to_waypoint.h"
//#include <vehicle_control/fully_controller.h>
//#include <vehicle_control/pid_controller.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "high_to_low");
	ros::NodeHandle nh;
	waypoints_to_waypoint::ws2w thews2w(nh);
	thews2w.run();
}