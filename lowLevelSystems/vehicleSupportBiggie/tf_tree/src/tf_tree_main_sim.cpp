#include <tf_tree/tf_tree_class_sim.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"tf_tree");
	ros::NodeHandle nh;
	tf_tree_sim the_tf_tree(nh);
	return the_tf_tree.loop();
}
