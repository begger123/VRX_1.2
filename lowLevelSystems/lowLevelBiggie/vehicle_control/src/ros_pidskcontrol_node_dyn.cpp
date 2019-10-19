//This will be the code that calls the specified controller
#include "vehicle_control/stationKeep_controller.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pid_skcontroller_dyn");
	ros::NodeHandle nh;
	pid_controller::sk thePid(nh);
	thePid.run();
}
