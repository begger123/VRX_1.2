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
    target_sub   = sk_nh->subscribe("/control_target_sk", 10, &pid_controller::sk::target_callback, this);
    pose_sub   = sk_nh->subscribe("/vehicle_pose", 10, &pid_controller::sk::pose_callback, this);
    state_sub = sk_nh->subscribe("/p3d_wamv_ned", 10, &pid_controller::sk::velo_callback, this);
    //state_sub = sk_nh->subscribe("/wamv/sensors/position/p3d_wamv", 10, &pid_controller::sk::velo_callback, this);
    control_pub = sk_nh->advertise<custom_messages_biggie::control_effort>("/control_effort", 10);  // published tau = {Tx, Ty, Mz}

    // Initialize some variables
    //double x_des = 10;
    //double y_des = 2;
    //double psi_des = M_PI/2;
    //eta_des << x_des, y_des, psi_des;
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

void pid_controller::sk::target_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    eta_des << msg->x, msg->y, msg->theta;
    newCommand=true;
}

void pid_controller::sk::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // curr_time2 = ros::Time::now();
    // delta_t2 = curr_time2 - prev_time2;
    // prev_time2 = curr_time2;
    // ROS_INFO("delta_t2 = %g", delta_t2.toSec());

    // Initialize system
    if (!initialized) {
        //yaw_angle = msg->theta;                         // it's already pi-wrapped
        //init_heading = yaw_angle;
        //Jt_prev << cos(yaw_angle), sin(yaw_angle), 0,
        //          -sin(yaw_angle), cos(yaw_angle), 0,
        //           0,              0,              1;
        //prev_time = ros::Time::now();
        initialized = true;
    }
    //else {
    //    x = msg->y;
    //    y = msg->x;
    //    yaw_angle = msg->theta;
    //    eta << x, y, yaw_angle;
    //    eta_t = eta - eta_des;
    //    eta_t(2) = piwrap(eta_t(2));
    //    ROS_INFO("eta_t = [%g, %g, %g]", eta_t(0), eta_t(1), eta_t(2));
        newPose = true;
    //}
    yaw_angle = msg->theta;                         // it's already pi-wrapped
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
                    -msg->twist.twist.angular.z;
        eta_td = J*velo_bff;

        newVelo = true;
    
        state_data_.header=msg->header;
        state_data_.child_frame_id=msg->child_frame_id;
        state_data_.pose=msg->pose;
        state_data_.twist=msg->twist;
        
        eta << state_data_.pose.pose.position.x, state_data_.pose.pose.position.y, yaw_angle;
        eta_t = eta - eta_des;
        eta_t(2) = piwrap(eta_t(2));
        ROS_INFO("eta_t = [%g, %g, %g]", eta_t(0), eta_t(1), eta_t(2));
    }
}

void pid_controller::sk::set_timestep()
{
    ROS_DEBUG("In set timestep");
    if (!prev_time.isZero()) //This case will typically only happens when the program is first starting
    {
        delta_t = ros::Time::now() - prev_time;
        prev_time = ros::Time::now();
        if (0 == delta_t.toSec())
        {   
            ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
            return;
        }
    }
    else
    {
        ROS_INFO("prev_time is 0, doing nothing");
        prev_time = ros::Time::now();
        return;
    }
}

void pid_controller::sk::integrate_eta_t()
{
    ROS_DEBUG("In integrate eta_t");
    this->set_timestep();
    //ROS_ERROR("The integral is set as non cumulative");
    integral_eta_t += eta_t * delta_t.toSec();
}

void pid_controller::sk::compute_tau()
{
    if (initialized && newPose && newVelo) {
        Eigen::Matrix3d Jt;
        Jt << cos(yaw_angle), sin(yaw_angle), 0,
             -sin(yaw_angle), cos(yaw_angle), 0,
              0,              0,              1;

        // Compute time step...
        //ros::Time curr_time = ros::Time::now();
        //delta_t = curr_time - prev_time;

        Eigen::Matrix3d J_dot_;
        Eigen::Matrix3d Jt_dot_;
        //note that in this derivate, the proper sign convnetion would be [- - 0; + - 0; 0 0 0].  However, because the angular velocity is in ENU, we must negate the sign
        J_dot_<<     sin(yaw_angle)*state_data_.twist.twist.angular.z, cos(yaw_angle)*state_data_.twist.twist.angular.z, 0.0,
                    -cos(yaw_angle)*state_data_.twist.twist.angular.z, sin(yaw_angle)*state_data_.twist.twist.angular.z, 0.0,
                     0.0,                                              0.0,                                             0.0;

        Jt_dot_=J_dot_.transpose();

        // ROS_INFO("delta_t = %g", delta_t.toSec());
        // ROS_INFO("yaw_angle = %g", yaw_angle);
        // ROS_INFO("init_heading = %g", init_heading);

        ROS_WARN("Heading controller gains: [Kp, Ki, Kd] = [%g, %g, %g]", Kp, Ki, Kd);
        
        //Jt_d = (Jt - Jt_prev)/delta_t.toSec();
        tau = -Kp*Jt*eta_t - Ki*Jt*integral_eta_t - Kd*(Jt_dot_*eta_t + Jt*eta_td);

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
        if(newCommand)
        {
            this->compute_tau();
            this->pub_control();
            newCommand=false;
        }
        // dynamic_reconfigure stuff
        f = boost::bind(&pid_controller::sk::get_theSKGains, this, _1, _2);
        server.setCallback(f);
		loop_rate.sleep();
	}
}


