#include <tf_tree/tf_tree_class_sim.h>

//the purpose of this class perform the static publishers for world to ned and world to odom
tf_tree_sim::tf_tree_sim(ros::NodeHandle &nh) : tf_nh_(&nh), loop_rate(10000)
{	
	this->set_static_transforms();
}

tf_tree_sim::~tf_tree_sim()
{	

}

void tf_tree_sim::set_static_transforms()
{
	//GPS origin to NED origin
	ned_tf.setOrigin(tf::Vector3(0, 0, 0.0));
	q_ned.setRPY(M_PI, 0, M_PI/2);
	q_ned.normalize();
	ned_tf.setRotation(q_ned);

	//GPS origin to ENU origin
	enu_tf.setOrigin(tf::Vector3(0, 0, 0.0));
	q_enu.setRPY(0, 0, 0);
	q_enu.normalize();
	enu_tf.setRotation(q_enu);

	//base_link to base_link_ned
	ned_tf.setOrigin(tf::Vector3(0, 0, 0.0));
	q_ned.setRPY(M_PI, 0, M_PI/2);
	q_ned.normalize();
	ned_tf.setRotation(q_ned);
}

void tf_tree_sim::send_transforms()
{
	ned_tf_br.sendTransform(tf::StampedTransform(ned_tf, ros::Time::now(), "wamv/odom", "ned_origin"));
	enu_tf_br.sendTransform(tf::StampedTransform(enu_tf, ros::Time::now(), "wamv/odom", "enu_origin"));
    // base_link_ned_br.sendTransform(tf::StampedTransform(ned_tf, ros::Time::now(), "base_link", "base_link_ned"));
}

int tf_tree_sim::loop()
{
	while(ros::ok())
	{
		this->send_transforms();
		loop_rate.sleep();
	}
	return 0;
}
