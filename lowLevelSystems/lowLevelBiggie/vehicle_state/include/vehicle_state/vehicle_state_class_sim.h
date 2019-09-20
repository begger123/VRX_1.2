#include <ros/ros.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"

typedef struct{
double north;
double east;
} NED_struct;

class vehicle_state_sim
{
public:
	vehicle_state_sim(ros::NodeHandle&nh);
	~vehicle_state_sim();
	void sim_callback(const nav_msgs::Odometry::ConstPtr& msg);
	int loop();
	geometry_msgs::Pose2D the_pose;


private:
	void set_transforms();
	void send_transforms();
	ros::NodeHandle *state_nh_;
	ros::Subscriber sim_sub;
	ros::Publisher base_link_ned_pub;
	ros::Publisher base_link_pose2d_pub;

	tf::Transform enu_to_ned_tf;

	nav_msgs::Odometry the_odometry;
	double yawAngle;

    tf::TransformBroadcaster base_link_ned_tf_br;
    tf::TransformBroadcaster lidar_ned_tf_br;
    tf::TransformBroadcaster camera_ned_tf_br;
	
	tf::Transform base_link_ned_tf;
	tf::Transform lidar_ned_tf;
	tf::Transform camera_ned_tf;

	tf::Quaternion q_base_link_ned;
	tf::Quaternion q_lidar_ned;
	tf::Quaternion q_camera_ned;

	float lidarXOffset;
	float lidarYOffset;
	float lidarZOffset;
	float cameraXOffset;
	float cameraYOffset;
	float cameraZOffset;
	
	ros::Rate loop_rate;
};
