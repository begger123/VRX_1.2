#include "vehicle_state/vehicle_state_class_sim.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_state_sim");
	ros::NodeHandle nh;
	vehicle_state_sim the_state_sim(nh);
	return the_state_sim.loop();
}
