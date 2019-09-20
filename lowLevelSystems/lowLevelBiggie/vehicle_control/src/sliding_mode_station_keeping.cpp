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
//eta_t_dot - 
//eta_d_dot - 
//output:
//tau:=M*(J(eta)^T*eta_r_dot_dot+J(eta)_dot^T*eta_r_dot)+C(v)*J(eta)^T*eta_r_dot+D(v)*J(eta)^T*eta_r_dot-J(eta)^T*R*sat(E^-1*s)
sm_controller::sl_mode_st_keep::sl_mode_st_keep(ros::NodeHandle &nh) : sm_sk_nh_(&nh), loop_rate(4) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("Entering initializer, next stop get_params");
	target_sub_ = sm_sk_nh_->subscribe("control_target", 10, &sm_controller::sl_mode_st_keep::target_callback, this);
	state_sub_ = sm_sk_nh_->subscribe("/p3d_wamv_ned", 10, &sm_controller::sl_mode_st_keep::state_callback, this);
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

	D_m3x3_ 	<<	Xu_, 0.0, 0.0,
					0.0, Yv_, 0.0,
					0.0, 0.0, Nr_;

	//Because this is strictly the station keeping type, the desired velocity is 0
	eta_d_dot_ << 0.0, 0.0, 0;
	//Because thius is strictly the station keeping type, eta_t_dot_ is the body fixed velocity
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
	eta_d_ << msg->x, msg->y, 0.0;
	newCommand=true;
}

void sm_controller::sl_mode_st_keep::state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_DEBUG("In state callback");
	state_data_.header=msg->header;
	state_data_.child_frame_id=msg->child_frame_id;
	state_data_.pose=msg->pose;
	state_data_.twist=msg->twist;

	//NED Frame
	//the same issue applies here as in the the sim state class
	yaw_angle=tf::getYaw(state_data_.pose.pose.orientation); //use tf::transform_datatypes function getYaw to convert Odom's quaternion to yaw

	eta_ << state_data_.pose.pose.position.x, state_data_.pose.pose.position.y, yaw_angle;

	if(is_sim)
	{
		yaw_angle=(-1)*yaw_angle;
	}
	if(yaw_angle<0)
	{
		yaw_angle=2*M_PI+yaw_angle;
	}

	//ROS_WARN("The yaw angle is %f.", yaw_angle);

	//Because thius is strictly the station keeping type, eta_t_dot_ is the body fixed velocity
	eta_t_dot_ << state_data_.twist.twist.linear.x, state_data_.twist.twist.linear.y, state_data_.twist.twist.linear.z;

	//This matrix is velocity dependent, therefore handled here
	C_m3x3_.setZero();//ensures we start fresh every time we have a new state
	C_m3x3_ 	<<	0.0, 0.0, -(m_-Yv_dot_)*state_data_.twist.twist.linear.y,
					0.0, 0.0, (m_-Xu_dot_)*state_data_.twist.twist.linear.x,
					(m_-Yv_dot_)*state_data_.twist.twist.linear.y, -(m_-Xu_dot_)*state_data_.twist.twist.linear.x, 0.0;

	J_ 	<<  cos(yaw_angle), -sin(yaw_angle), 0.0,
					sin(yaw_angle),  cos(yaw_angle), 0.0,
					0.0, 			 0.0, 			 1;

	J_dot_<<  -sin(yaw_angle)*state_data_.twist.twist.linear.z, -cos(yaw_angle)*state_data_.twist.twist.linear.z, 0.0,
					 cos(yaw_angle)*state_data_.twist.twist.linear.z, -sin(yaw_angle)*state_data_.twist.twist.linear.z, 0.0,
					 0.0, 			  								   0.0, 			  								0.0;

	J_trans_=J_.transpose();
	J_trans_dot_=J_dot_.transpose();
		
	newState=true;
}

double sm_controller::sl_mode_st_keep::wrap_heading(double heading)
{
	ROS_DEBUG("In heading wrapper");
	//ROS_DEBUG("Entering wrap_heading");
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

void sm_controller::sl_mode_st_keep::calc_eta_t()
{
	ROS_DEBUG("In calc eta_t");
	eta_t_=eta_-eta_d_;
	double temp=
	eta_t_(2)=this->wrap_heading(eta_t_(2));

}

void sm_controller::sl_mode_st_keep::integrate_eta_t()
{
	ROS_DEBUG("In integrate eta_t");
	integral_eta_t_ += eta_t_ * delta_t.toSec();
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
	this->calc_eta_t();
	this->integrate_eta_t();
	eta_r_dot_=eta_d_dot_-2*lamda_m3x3_*eta_t_;//-lamda_m3x3_*lamda_m3x3_*integral_eta_t_;
	s_=eta_t_dot_+2*lamda_m3x3_*eta_t_;//+lamda_m3x3_*lamda_m3x3_*integral_eta_t_;
}

void sm_controller::sl_mode_st_keep::calc_tau()
{
	ROS_DEBUG("In calc tau");
	//remember:
	//eta_r_dot is calculated
	//eta_r_dot_dot is -2*lamda*eta_t_dot-lamda^2*eta_t
	eta_r_dot_dot_=-2*lamda_m3x3_*eta_t_dot_-lamda_m3x3_*lamda_m3x3_*eta_t_;
	ROS_DEBUG("lamda_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",lamda_m3x3_(0,0),lamda_m3x3_(0,1),lamda_m3x3_(0,2),lamda_m3x3_(1,0),lamda_m3x3_(1,1),lamda_m3x3_(1,2),lamda_m3x3_(2,0),lamda_m3x3_(2,1),lamda_m3x3_(2,2));
	ROS_DEBUG("eta_t_ %f %f %f",eta_t_(0),eta_t_(1),eta_t_(2));
	ROS_DEBUG("eta_t_dot_ %f %f %f",eta_t_dot_(0),eta_t_dot_(1),eta_t_dot_(2));

	//tau:=M*(J(eta)^T*eta_r_dot_dot+J(eta)_dot^T*eta_r_dot)+C(v)*J(eta)^T*eta_r_dot+D(v)*J(eta)^T*eta_r_dot-J(eta)^T*R*sat(E^-1*s)
	ROS_DEBUG("M_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",M_m3x3_(0,0),M_m3x3_(0,1),M_m3x3_(0,2),M_m3x3_(1,0),M_m3x3_(1,1),M_m3x3_(1,2),M_m3x3_(2,0),M_m3x3_(2,1),M_m3x3_(2,2));
	ROS_DEBUG("C_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",C_m3x3_(0,0),C_m3x3_(0,1),C_m3x3_(0,2),C_m3x3_(1,0),C_m3x3_(1,1),C_m3x3_(1,2),C_m3x3_(2,0),C_m3x3_(2,1),C_m3x3_(2,2));
	ROS_DEBUG("D_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",D_m3x3_(0,0),D_m3x3_(0,1),D_m3x3_(0,2),D_m3x3_(1,0),D_m3x3_(1,1),D_m3x3_(1,2),D_m3x3_(2,0),D_m3x3_(2,1),D_m3x3_(2,2));
	ROS_DEBUG("R_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",R_m3x3_(0,0),R_m3x3_(0,1),R_m3x3_(0,2),R_m3x3_(1,0),R_m3x3_(1,1),R_m3x3_(1,2),R_m3x3_(2,0),R_m3x3_(2,1),R_m3x3_(2,2));
	ROS_DEBUG("e_m3x3_ %f %f %f \n %f %f %f \n %f %f %f",e_m3x3_(0,0),e_m3x3_(0,1),e_m3x3_(0,2),e_m3x3_(1,0),e_m3x3_(1,1),e_m3x3_(1,2),e_m3x3_(2,0),e_m3x3_(2,1),e_m3x3_(2,2));
	ROS_DEBUG("J_trans_ %f %f %f \n %f %f %f \n %f %f %f",J_trans_(0,0),J_trans_(0,1),J_trans_(0,2),J_trans_(1,0),J_trans_(1,1),J_trans_(1,2),J_trans_(2,0),J_trans_(2,1),J_trans_(2,2));
	ROS_DEBUG("J_trans_dot_ %f %f %f \n %f %f %f \n %f %f %f",J_trans_dot_(0,0),J_trans_dot_(0,1),J_trans_dot_(0,2),J_trans_dot_(1,0),J_trans_dot_(1,1),J_trans_dot_(1,2),J_trans_dot_(2,0),J_trans_dot_(2,1),J_trans_dot_(2,2));

	ROS_DEBUG("eta_r_dot_ %f %f %f",eta_r_dot_(0),eta_r_dot_(1),eta_r_dot_(2));
	ROS_DEBUG("eta_r_dot_dot_ %f %f %f",eta_r_dot_dot_(0),eta_r_dot_dot_(1),eta_r_dot_dot_(2));
	ROS_DEBUG("s_ %f %f %f",s_(0),s_(1),s_(2));
	tau_=M_m3x3_*(J_trans_*eta_r_dot_dot_+J_trans_dot_*eta_r_dot_)+C_m3x3_*J_trans_*eta_r_dot_+D_m3x3_*J_trans_*eta_r_dot_-J_trans_*R_m3x3_*this->sat_function(e_m3x3_.inverse()*s_);
	ROS_INFO("tau_ %f %f %f",tau_(0),tau_(1),tau_(2));
}

void sm_controller::sl_mode_st_keep::pub()
{
	ROS_DEBUG("In pub");
	custom_messages_biggie::control_effort control_effort_msg;

	control_effort_msg.header.stamp=ros::Time::now();
	control_effort_msg.type.data="PID";
	
	std_msgs::Float64 temp;

	temp.data=tau_(0);
	control_effort_msg.tau.push_back(temp);//X
	temp.data=0;	
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
			this->set_timestep();
			this->calc_variales();
			this->calc_tau();
			this->pub();
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
}