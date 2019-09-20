#include <the_planner/initialTrajectory/initial_trajectory.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "init_traj_server");
	ros::NodeHandle nh;
	high_level::gen_trajectory theTrajectory(nh);

	ros::ServiceServer service=nh.advertiseService("gen_init_traj", &high_level::gen_trajectory::generate, &theTrajectory);
	ros::spin();
}