#include <tf_tree/tf_tree_class.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"tf_tree");
	ros::NodeHandle nh;
	tf_tree the_tf_tree(nh);
	return the_tf_tree.loop();
}
