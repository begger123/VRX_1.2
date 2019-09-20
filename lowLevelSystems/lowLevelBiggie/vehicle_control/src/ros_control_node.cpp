//This will be the code that calls the specified controller
#include "vehicle_control/pid_controller.h"
//#include <vehicle_control/fully_controller.h>
//#include <vehicle_control/pid_controller.h>
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pid_controller");
	ros::NodeHandle nh;
	pid_controller::pid thePid(nh);
	thePid.run();
}