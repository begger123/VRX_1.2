//This will be Eduoardo Sarda's fully actuated sliding mode controller station keeping
#include <vehicle_control/sliding_mode_station_keeping.h>

//sliding mode equations
//input:
//eta_t:= eta-eta_d
//eta - vehicle pose in NED [x,y,psi]
//eta_d - desired vehicle pose in NED [x,y,psi]
//internal:
//s:=eta_t_dot+2*lamda*eta_t+lamda^2*integral(eta_t*dt):0:t
//eta_r_dot:=eta_d_dot-2*lamda*eta_t-lamda^2*integral(eta_t*dt):0:t
//eta_t:=eta-eta_d
//eta_t_dot:=eta_dot-eta_d_dot 
//eta_d_dot:=0 
//output:
//tau:=M*(J(eta)^T*eta_r_dot_dot+J(eta)_dot^T*eta_r_dot)+C(v)*J(eta)^T*eta_r_dot+D(v)*J(eta)^T*eta_r_dot-J(eta)^T*R*sat(E^-1*s)
sm_controller::sl_mode_st_keep::sl_mode_st_keep(ros::NodeHandle &nh) : sm_sk_nh_(&nh), loop_rate(60) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("Entering initializer, next stop get_params");
	target_sub_ = sm_sk_nh_->subscribe("control_target", 10, &sm_controller::sl_mode_st_keep::target_callback, this);
	state_sub_ = sm_sk_nh_->subscribe("/p3d_wamv_ned", 10, &sm_controller::sl_mode_st_keep::state_callback, this);
	pose_sub_ = sm_sk_nh_->subscribe("/vehicle_pose", 10, &sm_controller::sl_mode_st_keep::pose_callback, this);
	accel_sub_ = sm_sk_nh_->subscribe("/wamv/sensors/imu/imu/data", 10, &sm_controller::sl_mode_st_keep::accel_callback, this);
	control_effort_pub_ = sm_sk_nh_->advertise<custom_messages_biggie::control_effort>("/control_effort", 10); //published TAU = {X,Y,Eta}

	this->get_params();

	//lyapunov exponent matrix
	lamda_m3x3_ << 	lamda_, 0.0, 0.0, 
					0.0, lamda_, 0.0, 
					0.0, 0.0, lamda_;

	//uncertainty matrix
	R_m3x3_ 	<<  r_kx_, 0.0, 0.0, 
					0.0, r_ky_, 0.0, 
					0.0, 0.0, r_kpsi_; 

	//error bounds
	e_m3x3_ 	<<  e_kx_, 0.0, 0.0, 
					0.0, e_ky_, 0.0, 
					0.0, 0.0, e_kpsi_;

	//Setup Hydrodynamic matrices
	M_m3x3_ 	<<	(m_-Xu_dot_), 0.0, 0.0,
					0.0, (m_-Yv_dot_), 0.0,
					0.0, 0.0, (Iz_-Nr_dot_);


	
    //Since the C matrix varies with velocity, that will be set in the state callback function
    //set last_accel
    last_accel_.data=0;
    integral_eta_t_ << 0, 0, 0;
    eta_t_ << 0, 0, 0;
    eta_ << 0, 0, 0;
}

sm_controller::sl_mode_st_keep::~sl_mode_st_keep()
{

}

void sm_controller::sl_mode_st_keep::get_params()
{
	//ROS_DEBUG("Entering get_params, next stop check_pid_gains");
	//This style of obtaining params is used because it resolves the param relative to the namespace of the node
	//Rigid Body Inertial terms
  	ros::param::get("smsk/m", m_);
  	ros::param::get("smsk/Iz", Iz_);

	//Added Mass Inertial terms
  	ros::param::get("smsk/Xu_dot", Xu_dot_);
  	ros::param::get("smsk/Yv_dot", Yv_dot_);
  	ros::param::get("smsk/Nr_dot", Nr_dot_);

  	//Drag terms
  	ros::param::get("smsk/Xu", Xu_);
  	ros::param::get("smsk/Yv", Yv_);
  	ros::param::get("smsk/Nr", Nr_);

  	//Control matrix Lamda
  	ros::param::get("smsk/lamda", lamda_);

  	//Uncertainty matrix terms
  	ros::param::get("smsk/r_kx_", r_kx_);
  	ros::param::get("smsk/r_ky_", r_ky_);
  	ros::param::get("smsk/r_kpsi_", r_kpsi_);

  	//Error bound terms
  	ros::param::get("smsk/e_kx_", e_kx_);
  	ros::param::get("smsk/e_ky_", e_ky_);
  	ros::param::get("smsk/e_kpsi_", e_kpsi_);
}

void sm_controller::sl_mode_st_keep::target_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ROS_DEBUG("In target callback");
	//desired waypoint in NED
	eta_d_ << msg->x, msg->y, msg->theta;

    //Because this is strictly the station keeping type, the desired velocities are 0
	eta_d_dot_ << 0.0, 0.0, 0;
	eta_d_dot_dot_ << 0.0, 0.0, 0;
	
    newCommand=true;
}

void sm_controller::sl_mode_st_keep::accel_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_DEBUG("In accel callback");
    accel_data_.header=msg->header;
    accel_data_.linear_acceleration=msg->linear_acceleration;
    newAccel=true;
}
void sm_controller::sl_mode_st_keep::state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("In state callback");
	state_data_.header=msg->header;
	state_data_.child_frame_id=msg->child_frame_id;
	state_data_.pose=msg->pose;
	state_data_.twist=msg->twist;

    bool wait;
    wait=true;
    //lets make sure we get new heading and accleration data
    //while(ros::ok()&&wait)
    //{
    //    ROS_WARN("Spinning for heading and accel");
    //    if(newHeading&&newAccel)
    //    {
    //        newHeading=false;
    //        newAccel=false;
    //        wait=false;
    //    }
    //    ros::spinOnce();
    //    loop_rate.sleep();
    //}

    //Herein we will populate the following variables  as such:
    //eta_=[x y psi]^T
    //nu_=[u v r]^T
    
    //eta_dot_=[x_dot y_dot psi_dot]^T
    //eta_t_= eta-eta_d
    //eta_t_dot_ 
    //eta_d
    //eta_d_dot
    eta_ << state_data_.pose.pose.position.x, state_data_.pose.pose.position.y, yaw_angle;
    nu_ << state_data_.twist.twist.linear.x, state_data_.twist.twist.linear.y, state_data_.twist.twist.angular.z;
    //as no angular acceleration data is provided, we must calculat the angular acceleration about the z axis
    nu_dot_ << accel_data_.linear_acceleration.x, -accel_data_.linear_acceleration.y, (last_accel_.data+state_data_.twist.twist.angular.z)/delta_t.toSec(); 
    //nu_dot_ << state_data_.twist.twist.linear.x, state_data_.twist.twist.linear.y, state_data_.twist.twist.angular.z;
    last_accel_.data=-state_data_.twist.twist.angular.z;
    
    this->set_C();		
    this->set_J_and_J_dot();		
    
    // eta_dot_ = J(eta_)*nu_
    eta_dot_ = J_*nu_; 
    eta_dot_dot_ = J_dot_*nu_+J_*nu_dot_;

    eta_t_ = -(eta_ - eta_d_);
    eta_t_(2) = this->wrap_heading(eta_t_(2));//ensures the vehicle turns the shortest distance to address heading error
    eta_t_dot_ = -(eta_dot_ - eta_d_dot_);
    //eta_t_dot_(2)=-eta_t_dot_(2);
    eta_t_dot_dot_ = -(eta_dot_dot_ - eta_d_dot_dot_);
    //eta_t_dot_dot_(2) = -eta_t_dot_dot_(2);

    ROS_DEBUG("J_ is %f %f %f", J_(0,0), J_(0,1), J_(0,2));
    ROS_DEBUG("J_ is %f %f %f", J_(1,0), J_(1,1), J_(1,2));
    ROS_DEBUG("J_ is %f %f %f", J_(2,0), J_(2,1), J_(2,2));

    ROS_DEBUG("J_dot_ is %f %f %f", J_dot_(0,0), J_dot_(0,1), J_dot_(0,2));
    ROS_DEBUG("J_dot_ is %f %f %f", J_dot_(1,0), J_dot_(1,1), J_dot_(1,2));
    ROS_DEBUG("J_dot_ is %f %f %f", J_dot_(2,0), J_dot_(2,1), J_dot_(2,2));

    ROS_DEBUG("nu_dot_is %f %f %f", nu_dot_(0), nu_dot_(1), nu_dot_(2));

    ROS_DEBUG("This should be in NED");
	ROS_DEBUG("The eta_(0) is %f.", eta_(0));
	ROS_DEBUG("The eta_(1) is %f.", eta_(1));
	ROS_DEBUG("The eta_(2) and yaw_angle are %f.", eta_(2));

    //ThDEBUGhould be in NED, as this is a tracking error metric, eta(2) aka yaw error, should have a magnitude less than pi always
    ROS_DEBUG("This is the position tracking error metric");
	ROS_DEBUG("The eta_t_(0) is %f.", eta_t_(0));
	ROS_DEBUG("The eta_t_(1) is %f.", eta_t_(1));
	ROS_DEBUG("The eta_t_(2) is %f.", eta_t_(2));
    
    ROS_DEBUG("This is the velocity tracking error metric");
	ROS_DEBUG("The eta_t_dot_(0) is %f.", eta_t_dot_(0));
	ROS_DEBUG("The eta_t_dot_(1) is %f.", eta_t_dot_(1));
	ROS_DEBUG("The eta_t_dot_(2) is %f.", eta_t_dot_(2));
   
    ROS_DEBUG("This is acceleration tracking error metric");
	ROS_DEBUG("The eta_t_dot_dot_(0) is %f.", eta_t_dot_dot_(0));
	ROS_DEBUG("The eta_t_dot_dot_(1) is %f.", eta_t_dot_dot_(1));
	ROS_DEBUG("The eta_t_dot_dot_(2) is %f.", eta_t_dot_dot_(2));
    
    newState=true;
}

void sm_controller::sl_mode_st_keep::set_C()
{
	//This matrix is velocity dependent, therefore handled here
    //TODO - The velocities here need to be in the body fixed frame
	C_m3x3_.setZero();//ensures we start fresh every time we have a new state
	C_m3x3_ 	<<	0.0, 0.0, -(m_-Yv_dot_)*state_data_.twist.twist.linear.y,
					0.0, 0.0, (m_-Xu_dot_)*state_data_.twist.twist.linear.x,
					(m_-Yv_dot_)*state_data_.twist.twist.linear.y, -(m_-Xu_dot_)*state_data_.twist.twist.linear.x, 0.0;
    
    D_m3x3_ 	<<	-(Xu_ + 72.4*abs(state_data_.twist.twist.linear.x)), 0.0, 0.0,
					0.0, -Yv_, 0.0,
					0.0, 0.0, -Nr_;
}

void sm_controller::sl_mode_st_keep::set_J_and_J_dot()
{
	J_ 	<<          cos(yaw_angle), -sin(yaw_angle), 0.0,
					sin(yaw_angle),  cos(yaw_angle), 0.0,
					0.0, 			 0.0, 			 1;
    //note that in this derivate, the proper sign convnetion would be [- - 0; + - 0; 0 0 0].  However, because the angular velocity is in ENU, we must negate the sign
    J_dot_<<        sin(yaw_angle)*state_data_.twist.twist.angular.z, cos(yaw_angle)*state_data_.twist.twist.angular.z, 0.0,
					-cos(yaw_angle)*state_data_.twist.twist.angular.z, sin(yaw_angle)*state_data_.twist.twist.angular.z, 0.0,
					 0.0, 			  								   0.0, 			  								0.0;

	J_trans_=J_.transpose();
	J_trans_dot_=J_dot_.transpose();
}

void sm_controller::sl_mode_st_keep::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_DEBUG("In heading callback");
    yaw_angle=msg->theta;
    newHeading=true;
}

double sm_controller::sl_mode_st_keep::wrap_heading(double heading)
{
	//ROS_WARN("In heading wrapper");
	//ROS_DEBUG("Entering wrap_heading");
    //return atan2(sin(heading),cos(heading));
	
    if(heading>=M_PI)
	{
		return heading-=2*M_PI;
	}
	else if(heading<-M_PI)
	{
		return heading+=2*M_PI;
	}
	else
	{
		return heading;
	}
}

void sm_controller::sl_mode_st_keep::set_timestep()
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

void sm_controller::sl_mode_st_keep::integrate_eta_t()
{
	ROS_DEBUG("In integrate eta_t");
	this->set_timestep();
    ROS_ERROR("The integral is set as non cumulative");
	integral_eta_t_ += eta_t_ * delta_t.toSec();

    ROS_DEBUG("This is position tracking error metric in integrate");
	ROS_DEBUG("The eta_t_(0) is %f.", eta_t_(0));
	ROS_DEBUG("The eta_t_(1) is %f.", eta_t_(1));
	ROS_DEBUG("The eta_t_(2) is %f.", eta_t_(2));
	
    ROS_DEBUG("The delta_t is %f.", delta_t.toSec());
    
    ROS_WARN("This is the integral of the position tracking error metric");
	ROS_WARN("The integral_eta_t_(0) is %f.", integral_eta_t_(0));
	ROS_WARN("The integral_eta_t_(1) is %f.", integral_eta_t_(1));
	ROS_WARN("The integral_eta_t_(2) is %f.", integral_eta_t_(2));
}

Eigen::Vector3f sm_controller::sl_mode_st_keep::sat_function(Eigen::Vector3f errorBoundTimesSurface)
{
	ROS_DEBUG("In sat function");
	Eigen::Vector3f	saturatedSurface_t;
	if(errorBoundTimesSurface(0)>1.0)
	{
		saturatedSurface_t(0)=1.0;
	}
	else if(errorBoundTimesSurface(0)<-1.0)
	{
		saturatedSurface_t(0)=-1.0;
	}
	else
	{
		saturatedSurface_t(0)=errorBoundTimesSurface(0);
	}

	if(errorBoundTimesSurface(1)>1.0)
	{
		saturatedSurface_t(1)=1.0;
	}
	else if(errorBoundTimesSurface(1)<-1.0)
	{
		saturatedSurface_t(1)=-1.0;
	}
	else
	{
		saturatedSurface_t(1)=errorBoundTimesSurface(1);
	}

	if(errorBoundTimesSurface(2)>1.0)
	{
		saturatedSurface_t(2)=1.0;
	}
	else if(errorBoundTimesSurface(2)<-1.0)
	{
		saturatedSurface_t(2)=-1.0;
	}
	else
	{
		saturatedSurface_t(2)=errorBoundTimesSurface(2);
	}
	ROS_DEBUG("saturatedSurface_t %f %f %f",saturatedSurface_t(0),saturatedSurface_t(1),saturatedSurface_t(2));
	return saturatedSurface_t;
}

void sm_controller::sl_mode_st_keep::calc_variales()
{
	ROS_DEBUG("In calc variables");
	this->integrate_eta_t();
	eta_r_dot_=eta_t_dot_-2*lamda_m3x3_*eta_t_;//-lamda_m3x3_*lamda_m3x3_*integral_eta_t_;
	eta_r_dot_dot_=eta_t_dot_dot_-2*lamda_m3x3_*eta_t_dot_;//-lamda_m3x3_*lamda_m3x3_*eta_t_;
    
    s_=eta_t_dot_+2*lamda_m3x3_*eta_t_+lamda_m3x3_*lamda_m3x3_*integral_eta_t_;

    ROS_WARN("eta_r_dot_ is %f %f %f", eta_r_dot_(0), eta_r_dot_(1), eta_r_dot_(2));
    ROS_WARN("eta_r_dot_dot_ is %f %f %f", eta_r_dot_dot_(0), eta_r_dot_dot_(1), eta_r_dot_dot_(2));
    ROS_WARN("s_ is %f %f %f", s_(0), s_(1), s_(2));

}

void sm_controller::sl_mode_st_keep::calc_tau()
{
	ROS_DEBUG("In calc tau");
    tau_=M_m3x3_*(J_trans_*eta_r_dot_dot_+J_trans_dot_*eta_r_dot_)+C_m3x3_*J_trans_*eta_r_dot_+D_m3x3_*J_trans_*eta_r_dot_-J_trans_*R_m3x3_*this->sat_function(e_m3x3_.inverse()*s_);
    //tau_=M_m3x3_*(J_trans_*eta_r_dot_dot_+J_trans_dot_*eta_r_dot_)+C_m3x3_*J_trans_*eta_r_dot_+D_m3x3_*J_trans_*eta_r_dot_-J_trans_*R_m3x3_*this->sat_function(e_m3x3_.inverse()*s_);
    //tau_=M_m3x3_*(J_trans_*eta_dot_dot_+J_trans_dot_*eta_dot_)+C_m3x3_*J_trans_*eta_dot_+D_m3x3_*J_trans_*eta_dot_-J_trans_*R_m3x3_*this->sat_function(e_m3x3_.inverse()*s_);

    //tau_(1)=tau_(1);
    //tau_(2)=tau_(2);

    ROS_WARN("tau_ %f %f %f",tau_(0),tau_(1),tau_(2));
	
    ROS_WARN("lamda_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",lamda_m3x3_(0,0),lamda_m3x3_(0,1),lamda_m3x3_(0,2),lamda_m3x3_(1,0),lamda_m3x3_(1,1),lamda_m3x3_(1,2),lamda_m3x3_(2,0),lamda_m3x3_(2,1),lamda_m3x3_(2,2));
	ROS_WARN("eta_t_ %f %f %f",eta_t_(0),eta_t_(1),eta_t_(2));
	ROS_WARN("eta_t_dot_ %f %f %f",eta_t_dot_(0),eta_t_dot_(1),eta_t_dot_(2));

	//taWARN(J(eta)^T*eta_r_dot_dot+J(eta)_dot^T*eta_r_dot)+C(v)*J(eta)^T*eta_r_dot+D(v)*J(eta)^T*eta_r_dot-J(eta)^T*R*sat(E^-1*s)
	ROS_WARN("M_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",M_m3x3_(0,0),M_m3x3_(0,1),M_m3x3_(0,2),M_m3x3_(1,0),M_m3x3_(1,1),M_m3x3_(1,2),M_m3x3_(2,0),M_m3x3_(2,1),M_m3x3_(2,2));
	ROS_WARN("C_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",C_m3x3_(0,0),C_m3x3_(0,1),C_m3x3_(0,2),C_m3x3_(1,0),C_m3x3_(1,1),C_m3x3_(1,2),C_m3x3_(2,0),C_m3x3_(2,1),C_m3x3_(2,2));
	ROS_WARN("D_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",D_m3x3_(0,0),D_m3x3_(0,1),D_m3x3_(0,2),D_m3x3_(1,0),D_m3x3_(1,1),D_m3x3_(1,2),D_m3x3_(2,0),D_m3x3_(2,1),D_m3x3_(2,2));
	ROS_WARN("R_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",R_m3x3_(0,0),R_m3x3_(0,1),R_m3x3_(0,2),R_m3x3_(1,0),R_m3x3_(1,1),R_m3x3_(1,2),R_m3x3_(2,0),R_m3x3_(2,1),R_m3x3_(2,2));
	ROS_WARN("e_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",e_m3x3_(0,0),e_m3x3_(0,1),e_m3x3_(0,2),e_m3x3_(1,0),e_m3x3_(1,1),e_m3x3_(1,2),e_m3x3_(2,0),e_m3x3_(2,1),e_m3x3_(2,2));
	ROS_WARN("J_trans_ %f %f %f \n %f %f %f \n %f %f %f",J_trans_(0,0),J_trans_(0,1),J_trans_(0,2),J_trans_(1,0),J_trans_(1,1),J_trans_(1,2),J_trans_(2,0),J_trans_(2,1),J_trans_(2,2));
	ROS_WARN("J_trans_dot_ %f %f %f \n %f %f %f \n %f %f %f",J_trans_dot_(0,0),J_trans_dot_(0,1),J_trans_dot_(0,2),J_trans_dot_(1,0),J_trans_dot_(1,1),J_trans_dot_(1,2),J_trans_dot_(2,0),J_trans_dot_(2,1),J_trans_dot_(2,2));
}

void sm_controller::sl_mode_st_keep::pub()
{
	ROS_DEBUG("In pub");
	custom_messages_biggie::control_effort control_effort_msg;

	control_effort_msg.header.stamp=ros::Time::now();
	control_effort_msg.type.data="SM";
	
	std_msgs::Float64 temp;

	temp.data=tau_(0);
	control_effort_msg.tau.push_back(temp);//X
	temp.data=tau_(1);	
	control_effort_msg.tau.push_back(temp);//Y
	temp.data=tau_(2);
	control_effort_msg.tau.push_back(temp);//Eta

	//ROS_DEBUG("control_effort_heading %f", control_effort_heading);
	//ROS_DEBUG("control_effort_velocity %f",control_effort_velocity);

	control_effort_pub_.publish(control_effort_msg);
}

void sm_controller::sl_mode_st_keep::run()
{
	while(ros::ok())
	{
		if(newCommand&&newState)
		{
			newCommand=false;
			newState=false;
			this->calc_variales();
			this->calc_tau();
			this->pub();
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
}
