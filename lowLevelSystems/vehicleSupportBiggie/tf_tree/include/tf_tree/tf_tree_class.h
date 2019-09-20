#include <ros/ros.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

class tf_tree
{
public:
	tf_tree(ros::NodeHandle&nh);
	~tf_tree();
	int loop();

private:
	void set_static_transforms();
	void send_transforms();

	ros::NodeHandle *tf_nh_;	

    tf::TransformBroadcaster ned_tf_br;
    tf::TransformBroadcaster enu_tf_br;
	
	tf::Transform ned_tf;
	tf::Transform enu_tf;

	tf::Quaternion q_ned;
	tf::Quaternion q_enu;

	ros::Rate loop_rate;
};
