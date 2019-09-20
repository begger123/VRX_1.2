#include "vehicle_allocation/scaled_underactuated_allocation_sim.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_allocation");
	ros::NodeHandle nh;
	alloc::scaledUnderactuatedAllocationSim wamv_allocation(nh);
	return wamv_allocation.run();
}
