#include <vehicle_state/vehicle_state_class_sim.h>

//the purpose of this class take the p3d_wamv nav_msgs/Odometry message from the VMRC simulator and
//convert it so that the system behaves the same whether running with real or simulated data
vehicle_state_sim::vehicle_state_sim(ros::NodeHandle &nh) : state_nh_(&nh), loop_rate(100)
{
	sim_sub = state_nh_->subscribe("/wamv/robot_localization/odometry/filtered", 10, &vehicle_state_sim::sim_callback, this);
	base_link_ned_pub = state_nh_->advertise<nav_msgs::Odometry>("/p3d_wamv_ned", 10);
	base_link_pose2d_pub = state_nh_->advertise<geometry_msgs::Pose2D>("/vehicle_pose", 10);
  	ros::param::get("state/tf/lidarXOffset", lidarXOffset);
  	ros::param::get("state/tf/lidarYOffset", lidarYOffset);
  	ros::param::get("state/tf/lidarZOffset", lidarZOffset);
  	ros::param::get("state/tf/cameraXOffset", cameraXOffset);
  	ros::param::get("state/tf/cameraYOffset", cameraYOffset);
  	ros::param::get("state/tf/cameraZOffset", cameraZOffset);
}

vehicle_state_sim::~vehicle_state_sim()
{

}

void vehicle_state_sim::set_transforms()
{
	//NED origin to base_link
	base_link_ned_tf.setOrigin(tf::Vector3(the_odometry.pose.pose.position.x, the_odometry.pose.pose.position.y, the_odometry.pose.pose.position.z));
	q_base_link_ned.setRPY(0, 0, yawAngle);

    // ROS_INFO("yawAngle = %f", yawAngle);

	q_base_link_ned.normalize();
	base_link_ned_tf.setRotation(q_base_link_ned);

	//base_link to lidar
	lidar_ned_tf.setOrigin(tf::Vector3(lidarXOffset, lidarYOffset, lidarZOffset));
	q_lidar_ned.setRPY(M_PI, 0, 0);
	q_lidar_ned.normalize();
	lidar_ned_tf.setRotation(q_lidar_ned);

	//base_link to camera
	camera_ned_tf.setOrigin(tf::Vector3(cameraXOffset, cameraYOffset, cameraZOffset));
	//q_camera_ned.setRPY(M_PI/2, 0, M_PI/2);
	// ROS_WARN("WEIRD ROTATION IN BASE_LINK_TO_CAMERA");
	q_camera_ned.setRPY(M_PI/2, 0,  -M_PI/8);
	q_camera_ned.normalize();
	camera_ned_tf.setRotation(q_camera_ned);
}

void vehicle_state_sim::send_transforms()
{	
	base_link_ned_tf_br.sendTransform(tf::StampedTransform(base_link_ned_tf, ros::Time::now(), "ned_origin", "base_link_ned"));
	lidar_ned_tf_br.sendTransform(tf::StampedTransform(lidar_ned_tf, ros::Time::now(), "base_link_ned", "lidar_nwu"));
	camera_ned_tf_br.sendTransform(tf::StampedTransform(camera_ned_tf, ros::Time::now(), "base_link_ned", "camera_optical"));
}

void vehicle_state_sim::sim_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//converts from ENU to NED
	the_odometry.header.seq++;
	the_odometry.header.frame_id="ned_origin";
	//the_odometry.child_frame_id="base_link_ned";

	//Convert Position from ENU to NED with 2 Rotations
	//The conversion from ENU to BFF is a 90 degree rotation about Z (yaw), and a 180 degree rotation about X (roll)
	//R_y << cos(M_PI), 0, sin(M_PI), 0, 1, 0, -sin(M_PI), 0, cos(M_PI);
	//R_z << cos(M_PI/2), sin(M_PI/2), 0, -sin(M_PI/2), cos(M_PI/2), 0, 0, 0, 1;

	tf::Matrix3x3 R_z(cos(-M_PI/2), sin(-M_PI/2), 0, -sin(-M_PI/2), cos(-M_PI/2), 0, 0, 0, 1);
    // tf::Matrix3x3 R_x(cos(M_PI), 0, sin(M_PI), 0, 1, 0, -sin(M_PI), 0, cos(M_PI));
    tf::Matrix3x3 R_x(1, 0, 0, 0, cos(M_PI), sin(M_PI), 0, -sin(M_PI), cos(M_PI));
	tf::Vector3 enuHolder(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf::Vector3 nedHolder=R_z*R_x*enuHolder;
	the_odometry.pose.pose.position.x=nedHolder[0];
	the_odometry.pose.pose.position.y=nedHolder[1];
	the_odometry.pose.pose.position.z=nedHolder[2];

 	//from my testing this approach appears to be very slow, maxing out at about 2hz
	//tf::TransformListener enu_to_ned_listener;
    //tf::StampedTransform enu_to_ned_tf;
    //try
    //{
    //	//with a faster waitForTransform this code appears to have issues where "frame base_link does not exist"
    //	enu_to_ned_listener.waitForTransform("/ned_origin", "/odom", ros::Time::now(), ros::Duration(0.15));
    //	enu_to_ned_listener.lookupTransform("/ned_origin", "/odom", ros::Time(0), enu_to_ned_tf);
    //}
    //catch (tf::TransformException ex)
    //{
	//	ROS_ERROR("%s",ex.what());
    //}

	tf::Quaternion q_enu_temp;
	tf::quaternionMsgToTF(msg->pose.pose.orientation,q_enu_temp);
	tf::Quaternion enu_to_ned_tf;
	enu_to_ned_tf.setRPY(M_PI,0,-M_PI/2);
	tf::quaternionTFToMsg((q_enu_temp*enu_to_ned_tf).normalize(), the_odometry.pose.pose.orientation);

	tf::Vector3 enuLinearVel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
	tf::Vector3 nedLinearVel=R_z*R_x*enuLinearVel;
	the_odometry.twist.twist.linear.x=nedLinearVel[0];
	the_odometry.twist.twist.linear.y=nedLinearVel[1];
	the_odometry.twist.twist.linear.z=nedLinearVel[2];

	tf::Vector3 enuAngularVel(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
	tf::Vector3 nedAngularVel=R_z*R_x*enuAngularVel;
	the_odometry.twist.twist.angular.x=nedAngularVel[0];
	the_odometry.twist.twist.angular.y=nedAngularVel[1];
	the_odometry.twist.twist.angular.z=nedAngularVel[2];
	
	//create heading
	//im not sure why, but the above rotation "enu_to_ned_tf.setRPY(M_PI,0,-M_PI/2);" is not properly rotating about the roll, therefore we apply a negative sign below
	// yawAngle=-tf::getYaw(tf::Quaternion(the_odometry.pose.pose.orientation.x,the_odometry.pose.pose.orientation.y,the_odometry.pose.pose.orientation.z,the_odometry.pose.pose.orientation.w));

	yawAngle=tf::getYaw(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    
    // Convert yawAngle from ENU to NED convention
    if (yawAngle < M_PI/2) {
        yawAngle = M_PI/2 - yawAngle;
    }
    else {
        yawAngle = 5*M_PI/2 - yawAngle;
    }


    // ROS_INFO("yawAngle_vstate = %f", yawAngle);
    // ROS_INFO("orientation_x = %f", the_odometry.pose.pose.orientation.x);
    // ROS_INFO("orientation_y = %f", the_odometry.pose.pose.orientation.y);
    // ROS_INFO("orientation_z = %f", the_odometry.pose.pose.orientation.z);
    // ROS_INFO("orientation_w = %f", the_odometry.pose.pose.orientation.w);
}

int vehicle_state_sim::loop()
{
	while(ros::ok())
	{
		//perform callbacks
		ros::spinOnce();
		//publish
		this->set_transforms();
		this->send_transforms();
		the_odometry.header.stamp=ros::Time::now();
		base_link_ned_pub.publish(the_odometry);
		the_pose.x=the_odometry.pose.pose.position.x;
		the_pose.y=the_odometry.pose.pose.position.y;
		the_pose.theta=yawAngle;
        // ROS_INFO("yawAngle_vstate = %f", yawAngle);
		base_link_pose2d_pub.publish(the_pose);
		loop_rate.sleep();
	}
	return 0;
}
