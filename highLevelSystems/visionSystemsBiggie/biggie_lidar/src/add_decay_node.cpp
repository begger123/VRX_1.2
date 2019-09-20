//This will be the code that calls the specified controller
#include <biggie_lidar/add_decay.h>
//#include <vehicle_control/fully_controller.h>
//#include <vehicle_control/pid_controller.h>
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "lidar_decay_buffer");
	ros::NodeHandle nh;
	vision::add_decay decayBuffer(nh);
	while(ros::ok())
	{
		if(decayBuffer.newPC2)
		{
			decayBuffer.pop_and_push();	
			decayBuffer.newPC2=false;
		}
		ros::spinOnce();
	}
}