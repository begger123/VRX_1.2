#include "vehicle_allocation/scaled_underactuated_allocation.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"vehicle_allocation");
	ros::NodeHandle nh;
	alloc::scaledUnderactuatedAllocation wamv_allocation(nh);
	return wamv_allocation.run();
}
