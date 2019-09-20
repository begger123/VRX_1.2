#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

typedef struct{
double north;
double east;
} NED_struct;

class vehicle_state
{
public:
	vehicle_state(ros::NodeHandle&nh);
	~vehicle_state();
	void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void compass_callback(const geometry_msgs::Quaternion::ConstPtr& msg);
	void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	int loop();


private:

	void set_static_transforms();
	void set_transforms();
	void send_transforms();
	NED_struct Geo2NED(double lat, double lon, double latref, double lonref);

	ros::NodeHandle *state_nh_;
	ros::Subscriber gps_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber compass_sub;
	ros::Subscriber vel_sub;
	ros::Publisher base_link_ned_pub;
	ros::Publisher base_link_enu_pub;
	ros::Publisher base_link_pose2d_pub;	

    tf::TransformBroadcaster base_link_ned_tf_br;
    tf::TransformBroadcaster lidar_ned_tf_br;
    tf::TransformBroadcaster camera_ned_tf_br;
    tf::TransformBroadcaster base_link_enu_tf_br;
	
	tf::Transform base_link_ned_tf;
	tf::Transform lidar_ned_tf;
	tf::Transform camera_ned_tf;
	tf::Transform base_link_enu_tf;

	tf::Quaternion q_base_link_ned;
	tf::Quaternion q_lidar_ned;
	tf::Quaternion q_camera_ned;
	tf::Quaternion q_base_link_enu;

	NED_struct nedPoint;
	nav_msgs::Odometry the_odometry;
	double yawAngle;
	geometry_msgs::Pose2D the_pose;

	ros::Rate loop_rate;

	bool gpsFlag, imuFlag, compassFlag;

	double latRef;
	double lonRef;
	float lidarXOffset;
	float lidarYOffset;
	float lidarZOffset;
	float cameraXOffset;
	float cameraYOffset;
	float cameraZOffset;
};
