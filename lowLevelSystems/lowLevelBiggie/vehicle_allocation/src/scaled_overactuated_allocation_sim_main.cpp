#include "vehicle_allocation/scaled_overactuated_allocation_sim.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_allocation");
	ros::NodeHandle nh;
	alloc::scaledOveractuatedAllocationSim wamv_allocation(nh);
	return wamv_allocation.run();
}
