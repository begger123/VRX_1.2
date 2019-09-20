#include <vehicle_compass/compass_class.h>

///////////////////////main function//////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "compass_node");
	ros::NodeHandle nh;
	serial_compass the_compass(nh);
	the_compass.loop();
	return 01;
}////////////////////////////////END OF MAIN////////////////////////////////////////