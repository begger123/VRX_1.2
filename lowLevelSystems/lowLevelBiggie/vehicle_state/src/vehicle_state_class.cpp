#include <vehicle_state/vehicle_state_class.h>

//the purpose of this class is to assemble the ROS xsense node and the compass node
vehicle_state::vehicle_state(ros::NodeHandle &nh) : state_nh_(&nh), loop_rate(100)
{
	gps_sub = state_nh_->subscribe("/wamv/sensors/gps/gps/fix", 10, &vehicle_state::gps_callback, this);
	imu_sub = state_nh_->subscribe("/wamv/sensors/imu/imu/data", 10, &vehicle_state::imu_callback, this);
	vel_sub = state_nh_->subscribe("/wamv/sensors/gps/gps/fix_velocity", 10, &vehicle_state::vel_callback, this);
	compass_sub = state_nh_->subscribe("/compass", 10, &vehicle_state::compass_callback, this);
	base_link_ned_pub = state_nh_->advertise<nav_msgs::Odometry>("/p3d_wamv_ned", 10);
	base_link_enu_pub = state_nh_->advertise<nav_msgs::Odometry>("/p3d_wamv_enu", 10);
	base_link_pose2d_pub = state_nh_->advertise<geometry_msgs::Pose2D>("/vehicle_pose", 10);

	//read params
  	ros::param::get("state/ref/latRef", latRef);
  	ros::param::get("state/ref/lonRef", lonRef);
  	ros::param::get("state/tf/lidarXOffset", lidarXOffset);
  	ros::param::get("state/tf/lidarYOffset", lidarYOffset);
  	ros::param::get("state/tf/lidarZOffset", lidarZOffset);
  	ros::param::get("state/tf/cameraXOffset", cameraXOffset);
  	ros::param::get("state/tf/cameraYOffset", cameraYOffset);
  	ros::param::get("state/tf/cameraZOffset", cameraZOffset);
	ROS_DEBUG("The latRef is %f and the lonRef is %f ", latRef, lonRef);
}

vehicle_state::~vehicle_state()
{

}


void vehicle_state::set_transforms()
{
	//NED origin to base_link_ned
	base_link_ned_tf.setOrigin(tf::Vector3(the_odometry.pose.pose.position.x, the_odometry.pose.pose.position.y, 0));
	q_base_link_ned.setRPY(0, 0, yawAngle);
	q_base_link_ned.normalize();
	base_link_ned_tf.setRotation(q_base_link_ned);

	//base_link_ned to lidar
	lidar_ned_tf.setOrigin(tf::Vector3(lidarXOffset, lidarYOffset, lidarZOffset));
	q_lidar_ned.setRPY(M_PI, 0, 0);
	q_lidar_ned.normalize();
	lidar_ned_tf.setRotation(q_lidar_ned);

	//base_link_ned to camera
	camera_ned_tf.setOrigin(tf::Vector3(cameraXOffset, cameraYOffset, cameraZOffset));
	q_camera_ned.setRPY(M_PI/2, 0, M_PI/2);
	q_camera_ned.normalize();
	camera_ned_tf.setRotation(q_camera_ned);

	//odom to base_link
	base_link_enu_tf.setOrigin(tf::Vector3(the_odometry.pose.pose.position.y, the_odometry.pose.pose.position.x, 0));
	q_base_link_enu.setRPY(0, 0, -yawAngle);
	q_base_link_enu.normalize();
	base_link_enu_tf.setRotation(q_base_link_enu);
}

void vehicle_state::send_transforms()
{
	base_link_ned_tf_br.sendTransform(tf::StampedTransform(base_link_ned_tf, ros::Time::now(), "ned_origin", "base_link_ned"));
	lidar_ned_tf_br.sendTransform(tf::StampedTransform(lidar_ned_tf, ros::Time::now(), "base_link_ned", "lidar_nwu"));
	camera_ned_tf_br.sendTransform(tf::StampedTransform(camera_ned_tf, ros::Time::now(), "base_link_ned", "camera_optical"));
	base_link_enu_tf_br.sendTransform(tf::StampedTransform(base_link_enu_tf, ros::Time::now(), "wamv/odom", "wamv/base_link"));
}

void vehicle_state::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	//converts from GPS to NED
	nedPoint = Geo2NED(msg->latitude, msg->longitude, latRef, lonRef);

	the_odometry.pose.pose.position.x=nedPoint.north;
	the_odometry.pose.pose.position.y=nedPoint.east;
	the_odometry.pose.pose.position.z=msg->altitude;

	gpsFlag=true;
}

void vehicle_state::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	//only angular velocity and linear acceleration from available imu
	//acceleration not used in nav_msgs/Odometry message
	//yawAngle=tf::getYaw(tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));

	//attitude of the vehicle comes from compass, only based on heading
	//the_odometry.pose.pose.orientation=tf::createQuaternionMsgFromYaw(yawAngle);


	//In BFF from NED
	the_odometry.twist.twist.angular.x=msg->angular_velocity.x*cos(yawAngle)-msg->angular_velocity.y*sin(yawAngle);
	the_odometry.twist.twist.angular.y=msg->angular_velocity.x*sin(yawAngle)+msg->angular_velocity.y*cos(yawAngle);
	the_odometry.twist.twist.angular.z=msg->angular_velocity.z;

	imuFlag=true;
}

void vehicle_state::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	//set linear velocities, converts from NED to BFF
	the_odometry.twist.twist.linear.x = msg->twist.linear.x*cos(yawAngle)-msg->twist.linear.y*sin(yawAngle);
	the_odometry.twist.twist.linear.y = msg->twist.linear.x*sin(yawAngle)+msg->twist.linear.y*cos(yawAngle);
	the_odometry.twist.twist.linear.z = msg->twist.linear.z;
}

void vehicle_state::compass_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	//create heading
	yawAngle=tf::getYaw(tf::Quaternion(msg->x,msg->y,msg->z,msg->w));

	//attitude of the vehicle comes from compass, only based on heading
	the_odometry.pose.pose.orientation=tf::createQuaternionMsgFromYaw(yawAngle);

	compassFlag=true;
}

NED_struct vehicle_state::Geo2NED(double lat, double lon, double latref, double lonref)
{
	//cout << "new lat " << lat << " new lon " << lon << " lat " << latref << " lon " << lonref << endl;
	NED_struct inner_struct;
	double Rne[3][3];
	float Pned[2];
	unsigned long long Rea = 6378137; //radius of earth in m
	double e = 0.08181919;

	//printf("Current Pose (%f, %f)\n", lat, lon);
	//printf("Referen Pose (%f, %f)\n", latref, lonref);

	//printf("Rea = %llu e = %f \n", Rea, e);

	//convert degrees to radians
	lat = lat*M_PI/180;
	lon = lon*M_PI/180;
	latref = latref*M_PI/180;
	lonref = lonref*M_PI/180;

	//imperfect ellipsoid
	double Ne = Rea/(sqrt((1-pow(e,2)*pow(sin(lat),2))));
	double Neref = Rea/(sqrt((1-pow(e,2)*pow(sin(latref),2))));


	//printf("Ne = %f  Neref = %f \n", Ne, Neref);

	//ECEF coordinates
	double Pe_xe = Ne*cos(lat)*cos(lon);
	double Pe_ye = Ne*cos(lat)*sin(lon);
	double Pe_ze = Ne*(1-pow(e,2))*sin(lat);

	//printf("Pe_xe = %f  Pe_ye = %f \n", Pe_xe, Pe_ye);

	double Peref_xe =  Ne*cos(latref)*cos(lonref);
	double Peref_ye =  Ne*cos(latref)*sin(lonref);
	double Peref_ze = Ne*(1-pow(e,2))*sin(latref);

	//printf("Peref_xe = %f  Peref_ye = %f \n", Peref_xe, Peref_ye);

	//printf("Pexdif = %f  Peydif = %f \n", (Pe_xe - Peref_xe), (Pe_ye - Peref_ye));

	//Rne transformation matrix
	Rne[0][0] = -1*sin(latref)*cos(lonref);
	Rne[0][1] = -1*sin(latref)*sin(lonref);
	Rne[0][2] = cos(latref);
	Rne[1][0] = -1*sin(lonref);
	Rne[1][1] = cos(lonref);
	Rne[1][2] = 0;
	Rne[2][0] = -1*cos(latref)*cos(lonref);
	Rne[2][1] = -1*cos(latref)*sin(lonref);
	Rne[2][2] = -1*sin(latref);

	//NED coordinates
	Pned[0] = (Pe_xe - Peref_xe)*Rne[0][0] + (Pe_ye - Peref_ye)*Rne[0][1] + (Pe_ze - Peref_ze)*Rne[0][2];
	Pned[1] = (Pe_xe - Peref_xe)*Rne[1][0] + (Pe_ye - Peref_ye)*Rne[1][1] + 0;

	//printf("Pned (%f, %f)\n", Pned[0], Pned[1]);
	inner_struct.north = Pned[0];
	inner_struct.east = Pned[1];
	//printf("Struct (%.10f, %.10f)\n", inner_struct.north, inner_struct.east);

	return inner_struct;
}

int vehicle_state::loop()
{
	while(ros::ok())
	{
		//perform callbacks
		ros::spinOnce();
		//publish
		the_odometry.header.seq++;
		the_odometry.header.stamp=ros::Time::now();
		the_odometry.header.frame_id="ned_origin";
		the_odometry.child_frame_id="base_link_ned";
		this->set_transforms();
		this->send_transforms();
		base_link_ned_pub.publish(the_odometry);
		the_pose.x=the_odometry.pose.pose.position.x;
		the_pose.y=the_odometry.pose.pose.position.y;
		the_pose.theta=yawAngle;
		base_link_pose2d_pub.publish(the_pose);
		loop_rate.sleep();
	}
	return 0;
}
