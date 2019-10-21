#include <vehicle_control/stationKeep_controller.h>

// Program written by Armando J. Sinisterra for the VRX challenge
// October 19, 2019

pid_controller::sk::sk(ros::NodeHandle &nh) : sk_nh(&nh), loop_rate(60) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("Entering initializer, next stop get_params");
    pose_sub   = sk_nh->subscribe("/vehicle_pose", 10, &pid_controller::sk::pose_callback, this);
	state_sub = sk_nh->subscribe("/p3d_wamv_ned", 10, &pid_controller::sk::velo_callback, this);
    control_pub = sk_nh->advertise<custom_messages_biggie::control_effort>("/control_effort", 10);  // published tau = {Tx, Ty, Mz}

    // Initialize some variables
    double x_des = 20;
    double y_des = 0;
    double psi_des = M_PI/2;
    eta_des << x_des, y_des, psi_des;
    initialized = false;
    newPose = false;
    newVelo = false;
    newControl = false;
    init_heading = 0;

    ros::Duration(2).sleep();
}


pid_controller::sk::~sk()
{

}


double pid_controller::sk::piwrap(double angle)
{
   angle = fmod(angle, 2*M_PI);
   if (angle >= 0 && angle <= M_PI) {
       return(angle);
   }
   else {
       return(angle - 2*M_PI);
   }
}


void pid_controller::sk::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // curr_time2 = ros::Time::now();
    // delta_t2 = curr_time2 - prev_time2;
    // prev_time2 = curr_time2;
    // ROS_INFO("delta_t2 = %g", delta_t2.toSec());

    // Initialize system
    if (!initialized) {
        yaw_angle = msg->theta;                         // it's already pi-wrapped
        init_heading = yaw_angle;
        Jt_prev << cos(yaw_angle), sin(yaw_angle), 0,
                  -sin(yaw_angle), cos(yaw_angle), 0,
                   0,              0,              1;
        prev_time = ros::Time::now();
        initialized = true;
    }
    else {
        x = msg->x;
        y = msg->y;
        yaw_angle = msg->theta;
        eta << x, y, yaw_angle;
        eta_t = eta - eta_des;
        eta_t(2) = piwrap(eta_t(2));
        ROS_INFO("eta_t = [%g, %g, %g]", eta_t(0), eta_t(1), eta_t(2));
        newPose = true;
    }
}


void pid_controller::sk::velo_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (initialized) {
        Eigen::Matrix3d J;
        J << cos(yaw_angle), -sin(yaw_angle), 0,
             sin(yaw_angle),  cos(yaw_angle), 0,
             0,               0,              1;

        Eigen::Vector3d velo_bff;
        velo_bff << msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.angular.z;
        eta_td = J*velo_bff;

        newVelo = true;
    }
}


void pid_controller::sk::compute_tau()
{
    if (initialized && newPose && newVelo) {
        Eigen::Matrix3d Jt;
        Jt << cos(yaw_angle), sin(yaw_angle), 0,
             -sin(yaw_angle), cos(yaw_angle), 0,
              0,              0,              1;

        // Compute time step...
        ros::Time curr_time = ros::Time::now();
        delta_t = curr_time - prev_time;

        // ROS_INFO("delta_t = %g", delta_t.toSec());
        // ROS_INFO("yaw_angle = %g", yaw_angle);
        // ROS_INFO("init_heading = %g", init_heading);

        Eigen::Matrix3d Jt_d;
        Jt_d = (Jt - Jt_prev)/delta_t.toSec();
        tau = -Kp*Jt*eta_t - Kd*(Jt_d*eta_t + Jt*eta_td);

        Jt_prev = Jt;
        prev_time = curr_time;

        newPose = false;
        newVelo = false;
        newControl = true;
    }
}


void pid_controller::sk::pub_control() 
{
    if (newControl) {
        std_msgs::Float64 temp;
	    custom_messages_biggie::control_effort control_effort_msg;
	    control_effort_msg.header.stamp = prev_time;
	    control_effort_msg.type.data="PID-SK";
        temp.data = tau(0);
        control_effort_msg.tau.push_back(temp);
        temp.data = tau(1);
        control_effort_msg.tau.push_back(temp);
        temp.data = tau(2);
        control_effort_msg.tau.push_back(temp);

	    control_pub.publish(control_effort_msg);

        newControl = false;
    }
}


void pid_controller::sk::get_theSKGains(vehicle_control::pidSKGainsConfig &config, uint32_t level)
{
    Kp = config.Kp;
    Ki = config.Ki;
    Kd = config.Kd;

    // ROS_INFO("Heading controller gains: [Kp, Ki, Kd] = [%g, %g, %g]", Kp, Ki, Kd);
}


void pid_controller::sk::run()
{
	while(ros::ok())
	{
		ros::spinOnce();
        this->compute_tau();
        this->pub_control();
        // dynamic_reconfigure stuff
        f = boost::bind(&pid_controller::sk::get_theSKGains, this, _1, _2);
        server.setCallback(f);
		loop_rate.sleep();
	}
}


