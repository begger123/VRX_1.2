#include "vehicle_state/vehicle_state_class.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_state");
	ros::NodeHandle nh;
	vehicle_state the_state(nh);
	return the_state.loop();
}
