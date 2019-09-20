//This will be the code that calls the specified controller
#include "vehicle_control/sliding_mode_station_keeping.h"
//#include <vehicle_control/fully_controller.h>
//#include <vehicle_control/pid_controller.h>
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sliding_mode_controller");
	ros::NodeHandle nh;
	sm_controller::sl_mode_st_keep theSMSK(nh);
	theSMSK.run();
}