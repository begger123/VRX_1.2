//This will be a cascaded heading and velocity controller
#include <vehicle_control/pid_controller.h>

pid_controller::pid::pid(ros::NodeHandle &nh) : pid_nh(&nh), loop_rate(4) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("Entering initializer, next stop get_params");
	target_sub = pid_nh->subscribe("control_target", 10, &pid_controller::pid::target_callback, this);
    pose_sub   = pid_nh->subscribe("/vehicle_pose", 10, &pid_controller::pid::pose_callback, this);
	//state_sub = pid_nh->subscribe("vehicle_state", 10, &pid_controller::pid::state_callback, this);//use this for real vehicle
	state_sub = pid_nh->subscribe("/p3d_wamv_ned", 10, &pid_controller::pid::state_callback, this);//use this for simulation
	control_effort_pub = pid_nh->advertise<custom_messages_biggie::control_effort>("/control_effort", 10); //published TAU = {X,Y,Eta}

	this->get_params();
	previous_waypoint.x=0;
	previous_waypoint.y=0;
	previous_waypoint.z=0;
}

pid_controller::pid::~pid()
{

}

void pid_controller::pid::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    yaw_angle = msg->theta;
    // ROS_INFO("yaw_angle = %f", yaw_angle);
}

void pid_controller::pid::target_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//control on cross track error
	//desired waypoint in NED
	setpoint << msg->x, msg->y, 0;
	//this function uses the z channel of the geometry_msgs/Point message to control the velocity of the vehicle
	velocity_command=msg->theta;

	//if the previous message is different from the current message, it means we need to set the flag so that that initial position
	//of the vehicle is updated in the cross track error calculation

	float xDelta=fabs(fabs(previous_waypoint.x)-fabs(msg->x));
	float yDelta=fabs(fabs(previous_waypoint.y)-fabs(msg->y));
	float zDelta=fabs(fabs(previous_waypoint.z)-fabs(msg->theta));

	
	if(xDelta < 0.001 && yDelta < 0.001 && zDelta < 0.001)
	{
		//do nothing
	}
	else
	{	
		ROS_DEBUG("WAYPOINT HAS BEEN RESET");
		ROS_DEBUG("xDelta:%f",xDelta);
		ROS_DEBUG("yDelta:%f",yDelta);
		ROS_DEBUG("zDelta:%f",zDelta);
		resetOrigWaypoint=true;
		previous_waypoint.x=setpoint[0];
		previous_waypoint.y=setpoint[1];
		previous_waypoint.z=velocity_command;
	}


	//These messages are based on the original implementation of this controller where the input message to this function was of type
	//custom_messages_biggie::control_target
	//Left to be used as a reference in case that functionality is implemented in addition to the current functionality later
	//if(target_data.type.data!="PID")
	//{
	//	ROS_WARN("The type of command sent to the pid controller is incorrect");
	//}
	//else if(target_data.control_command.size()!=2)
	//{
	//	ROS_WARN("The pid command message is of the incorrect format.");
	//	ROS_WARN("Please ensure that it is of format [heading velocity].");
	//}
	//ROS_DEBUG("The size of control command is %lu",target_data.control_command.size());
	//ROS_DEBUG("The value of heading command is %f",target_data.control_command.at(0).data);
	//ROS_DEBUG("The value of velocity command is %f",target_data.control_command.at(1).data);

	newCommand=true;
}

void pid_controller::pid::state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	state_data.header=msg->header;
	state_data.child_frame_id=msg->child_frame_id;
	state_data.pose=msg->pose;
	state_data.twist=msg->twist;

	float vel=sqrt(state_data.twist.twist.linear.x*state_data.twist.twist.linear.x+state_data.twist.twist.linear.y*state_data.twist.twist.linear.y);
    // ROS_INFO("vel is %f", vel);

	//NED Frame
	//the same issue applies here as in the the sim state class
    // yaw_angle=tf::getYaw(state_data.pose.pose.orientation); //use tf::transform_datatypes function getYaw to convert Odom's quaternion to yaw
	
	// if(is_sim)
	// {
	//     yaw_angle=(-1)*yaw_angle;
	// }

	//ROS_WARN("The yaw angle is %f.", yaw_angle);

	nedLocation << state_data.pose.pose.position.x, state_data.pose.pose.position.y, state_data.pose.pose.position.z;

	newState=true;
}

void pid_controller::pid::set_error_los()
{
	heading_error.at(2) = heading_error.at(1);
	heading_error.at(1) = heading_error.at(0);
	
	ROS_DEBUG("setpoint is %f, %f, %f", setpoint[0], setpoint[1], setpoint[2]);
	ROS_DEBUG("nedLocation is %f, %f, %f", nedLocation[0], nedLocation[1], nedLocation[2]);

	float thetaD=atan2(setpoint[1]-nedLocation[1],setpoint[0]-nedLocation[0]);
	float headingError=thetaD-yaw_angle;

	ROS_DEBUG("thetaD is %f", thetaD);
	ROS_DEBUG("yaw_angle is %f", yaw_angle);
	ROS_DEBUG("heading error after wrapping is BEFORE %f", heading_error.at(0));
	
	heading_error.at(0)=headingError;
	heading_error.at(0)=this->wrap_heading(heading_error.at(0));

	//ROS_INFO("heading error after wrapping is BEFORE %f", heading_error.at(0));

	//set velocity error
	velocity_error.at(2) = velocity_error.at(1);
	velocity_error.at(1) = velocity_error.at(0);

	//set velocity error
	velocity_error.at(2) = velocity_error.at(1);
	velocity_error.at(1) = velocity_error.at(0);
	//velocity is NED body fixed
	float vel=sqrt(state_data.twist.twist.linear.x*state_data.twist.twist.linear.x+state_data.twist.twist.linear.y*state_data.twist.twist.linear.y);
	velocity_error.at(0) = velocity_command-vel;// Current error goes to slot 0
	//ROS_DEBUG("velocity error is %f", velocity_error.at(0));
	//ROS_DEBUG("control_command is %f", velocity_command);
	//ROS_DEBUG("vel is %f", vel);
}
void pid_controller::pid::set_error_ebs()
{
	bool feasible=true;
	//ROS_DEBUG("Entering set_error");
	//set heading error
	heading_error.at(2) = heading_error.at(1);
	heading_error.at(1) = heading_error.at(0);

	//original waypoint
	if(resetOrigWaypoint)
	{
		originalPoint << nedLocation[0],nedLocation[1],nedLocation[2];
		resetOrigWaypoint=false;
	}
	
	//ROS_DEBUG("setpoint %f, %f, %f", setpoint[0], setpoint[1], setpoint[2]);
	//ROS_DEBUG("originalPoint is %f, %f, %f", originalPoint[0], originalPoint[1], originalPoint[2]);
	//ROS_DEBUG("nedLocation is %f, %f, %f", nedLocation[0], nedLocation[1], nedLocation[2]);

	//Based on Fossen's Enclosure-Based Steering from Marine Handbook
	//These calculations are done in the NED frame
	//After the calculations are complete, we will convert the the desired theta to ENU
	//there are two cases
	delta_x=setpoint[0]-originalPoint[0];
	delta_y=setpoint[1]-originalPoint[1];
	d=delta_y/delta_x;
	g=originalPoint[1]-d*originalPoint[0];
	
	a=1+pow(d,2);
	b=2*(d*g-d*nedLocation[1]-nedLocation[0]);
	c=pow(nedLocation[0],2)+pow(nedLocation[1],2)+pow(g,2)-2*g*nedLocation[1]-pow(R,2);
	if(fabs(delta_x)>0)
	{
		if(pow(b,2)-4*a*c >= 0)
		{
			if(delta_x>0)
			{
				ROS_DEBUG("delta_x>0");
				x_los=(-b+sqrt(pow(b,2)-4*a*c))/(2*a);
				y_los=d*(x_los-originalPoint[0])+originalPoint[1];
			}
			else if(delta_x<0)
			{
				ROS_DEBUG("delta_x<0");
				x_los=(-b-sqrt(pow(b,2)-4*a*c))/(2*a);	
				y_los=d*(x_los-originalPoint[0])+originalPoint[1];
			}
		}
		else
		{
			feasible=false;
		}
	}
	else
	{
		if(pow(R,2)-pow((setpoint[0]-nedLocation[0]),2)>=0)
		{
			ROS_DEBUG("delta_x=0");
			x_los=setpoint[0];
			if(delta_y>0)
			{
				ROS_DEBUG("delta_y>0");
				y_los=nedLocation[1]+sqrt(pow(R,2)-pow(x_los-nedLocation[0],2));
			}
			else if (delta_y<0)
			{
				ROS_DEBUG("delta_y<0");
				y_los=nedLocation[1]-sqrt(pow(R,2)-pow(x_los-nedLocation[0],2));
			}
			else
			{
				ROS_WARN("delta_x=0 and delta_y=0, this should not happen");
			}
		}
		else
		{
			feasible=false;
		}
	}
	float thetaD;
	if (feasible)
    {
    	thetaD=atan2(y_los-nedLocation[1],x_los-nedLocation[0]);
    }
    else
    {
        //If we cannot compute a course, use a simple LOS point to desired waypoint
        ROS_WARN("EBS trajectory not available, using LOS");
		thetaD=atan2(setpoint[1]-nedLocation[1],setpoint[0]-nedLocation[0]);
    }

    // ROS_INFO("x_los %f", x_los);
    // ROS_INFO("y_los %f", y_los);
    // ROS_INFO("delta_x %f", delta_x);
    // ROS_INFO("delta_y %f", delta_y);

	// ROS_INFO("thetaD is %f", thetaD);
	// ROS_INFO("yaw_angle is %f", yaw_angle);
	// ROS_DEBUG("heading error after wrapping is BEFORE %f", heading_error.at(0));

	float headingError=thetaD-yaw_angle;
	heading_error.at(0)=headingError;
	heading_error.at(0)=this->wrap_heading(heading_error.at(0));

	// ROS_INFO("heading error after wrapping is BEFORE %f", heading_error.at(0));

	//set velocity error
	velocity_error.at(2) = velocity_error.at(1);
	velocity_error.at(1) = velocity_error.at(0);
	//velocity is NED body fixed
	// float vel=sqrt(state_data.twist.twist.linear.x*state_data.twist.twist.linear.x+state_data.twist.twist.linear.y*state_data.twist.twist.linear.y);
    float vel = state_data.twist.twist.linear.x;
	velocity_error.at(0) = velocity_command-vel;// Current error goes to slot 0
	//ROS_DEBUG("velocity error is %f", velocity_error.at(0));
	//ROS_DEBUG("control_command is %f", velocity_command);
    ROS_DEBUG("vel is %f", vel);
}

double pid_controller::pid::wrap_heading(double heading)
{
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

void pid_controller::pid::set_timestep()
{
	//ROS_DEBUG("Entering set_timestep");
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

void pid_controller::pid::integrate_error()
{
	//ROS_DEBUG("Entering integrate_error");
	integral_heading_error += heading_error.at(0) * delta_t.toSec();
	integral_velocity_error += velocity_error.at(0) * delta_t.toSec();
}

//This version of anti-windup is back calculation according to Alberto Bemporad
void pid_controller::pid::anti_windup()
{
	//ROS_DEBUG("Entering anti_windup");
	// Apply windup limit to limit the size of the heading integral term
	if(integral_heading_error > fabsf(anti_windup_limit_h))
	{
		integral_heading_error = fabsf(anti_windup_limit_h);
	}
	if(integral_heading_error < -fabsf(anti_windup_limit_h))
	{
		integral_heading_error = -fabsf(anti_windup_limit_h);
	}
	// Apply windup limit to limit the size of the velocity term	
	if(integral_velocity_error > fabsf(anti_windup_limit_v))
	{
		integral_velocity_error = fabsf(anti_windup_limit_v);
	}
	if(integral_velocity_error < -fabsf(anti_windup_limit_v))
	{
		integral_velocity_error = -fabsf(anti_windup_limit_v);
	}
}

//Pretty sure this way is garbage because once this term is saturated, it will stay saturated until there is error in the other direction
void pid_controller::pid::anti_windup_ros_pid()
{
	// Apply windup limit to limit the size of the heading integral term
	if(integral_heading_error > fabsf(anti_windup_limit_h))
	{
		integral_heading_error = fabsf(anti_windup_limit_h);
	}
	if(integral_heading_error < -fabsf(anti_windup_limit_h))
	{
		integral_heading_error = -fabsf(anti_windup_limit_h);
	}
	// Apply windup limit to limit the size of the velocity term	
	if(integral_velocity_error > fabsf(anti_windup_limit_v))
	{
		integral_velocity_error = fabsf(anti_windup_limit_v);
	}
	if(integral_velocity_error < -fabsf(anti_windup_limit_v))
	{
		integral_velocity_error = -fabsf(anti_windup_limit_v);
	}
}

void pid_controller::pid::design_filter()
{
	//ROS_DEBUG("Entering design_filter");
	// My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
	if(cutoff_frequency_h != -1)
	{
		// Check if tan(_) is really small, could cause c = NaN
		tan_filt_h = tan( (cutoff_frequency_h*2*M_PI)*delta_t.toSec()/2 );

		// Avoid tan(0) ==> NaN
		if((tan_filt_h<=0.) && (tan_filt_h>-0.01))
		{
			tan_filt_h = -0.01;
		}
		if((tan_filt_h>=0.) && (tan_filt_h<0.01))
		{
			tan_filt_h = 0.01;
		}

		c_h = 1/tan_filt_h;
	}
	// My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
	if(cutoff_frequency_v != -1)
	{
		// Check if tan(_) is really small, could cause c = NaN
		tan_filt_v = tan( (cutoff_frequency_v*2*M_PI)*delta_t.toSec()/2 );

		// Avoid tan(0) ==> NaN
		if((tan_filt_v<=0.) && (tan_filt_v>-0.01))
		{
			tan_filt_v = -0.01;
		}
		if((tan_filt_v>=0.) && (tan_filt_v<0.01))
		{
			tan_filt_v = 0.01;
		}

		c_v = 1/tan_filt_v;
	}
}

void pid_controller::pid::filter_error()
{
	filtered_heading_error.at(2) = filtered_heading_error.at(1);
	filtered_heading_error.at(1) = filtered_heading_error.at(0); 
	filtered_heading_error.at(0) = (1/(1+c_h*c_h+1.414*c_h))*(heading_error.at(2)+2*heading_error.at(1)+heading_error.at(0)-(c_h*c_h-1.414*c_h+1)*filtered_heading_error.at(2)-(-2*c_h*c_h+2)*filtered_heading_error.at(1));

    // ROS_INFO("heading_error = %f", heading_error.at(0));

	filtered_velocity_error.at(2) = filtered_velocity_error.at(1);
	filtered_velocity_error.at(1) = filtered_velocity_error.at(0); 
	filtered_velocity_error.at(0) = (1/(1+c_v*c_v+1.414*c_v))*(velocity_error.at(2)+2*velocity_error.at(1)+velocity_error.at(0)-(c_v*c_v-1.414*c_v+1)*filtered_velocity_error.at(2)-(-2*c_v*c_v+2)*filtered_velocity_error.at(1));
}

void pid_controller::pid::differentiate_error()
{
	//first take the derivative of the unfiltered data because this needs to have the filter applied to it
	//Heading
	heading_error_deriv.at(2) = heading_error_deriv.at(1);
	heading_error_deriv.at(1) = heading_error_deriv.at(0);
	heading_error_deriv.at(0) = (heading_error.at(0)-heading_error.at(1))/delta_t.toSec();

	filtered_heading_error_deriv.at(2) = filtered_heading_error_deriv.at(1);
	filtered_heading_error_deriv.at(1) = filtered_heading_error_deriv.at(0);

	//Velocity
	velocity_error_deriv.at(2) = velocity_error_deriv.at(1);
	velocity_error_deriv.at(1) = velocity_error_deriv.at(0);
	velocity_error_deriv.at(0) = (velocity_error.at(0)-velocity_error.at(1))/delta_t.toSec();

	filtered_velocity_error_deriv.at(2) = filtered_velocity_error_deriv.at(1);
	filtered_velocity_error_deriv.at(1) = filtered_velocity_error_deriv.at(0);

	if(loop_counter>2) //Let some data accumulate before applying filter
	{
		filtered_heading_error_deriv.at(0) = (1/(1+c_h*c_h+1.414*c_h))*(heading_error_deriv.at(2)+2*heading_error_deriv.at(1)+heading_error_deriv.at(0)-(c_h*c_h-1.414*c_h+1)*filtered_heading_error_deriv.at(2)-(-2*c_h*c_h+2)*filtered_heading_error_deriv.at(1));
		filtered_velocity_error_deriv.at(0) = (1/(1+c_v*c_v+1.414*c_v))*(velocity_error_deriv.at(2)+2*velocity_error_deriv.at(1)+velocity_error_deriv.at(0)-(c_v*c_v-1.414*c_v+1)*filtered_velocity_error_deriv.at(2)-(-2*c_v*c_v+2)*filtered_velocity_error_deriv.at(1));
	}
	else
	{
		loop_counter++;
	}
}

void pid_controller::pid::accumulate_control_effort()
{
	proportional_heading = Kp_h * filtered_heading_error.at(0);
	integral_heading = Ki_h * integral_heading_error;
	derivative_heading = Kd_h * filtered_heading_error_deriv.at(0);
    // ROS_INFO("filtered_heading_error_0 = %f", filtered_heading_error.at(0));
    // ROS_INFO("filtered_heading_error_deriv_0 = %f", filtered_heading_error_deriv.at(0));

	control_effort_heading = proportional_heading + integral_heading + derivative_heading;
      // ROS_INFO("Proportional heading effort %f", proportional_heading);
      // ROS_INFO("Integral heading effort %f", integral_heading);
      // ROS_INFO("Derivative heading effort %f", derivative_heading);

	proportional_velocity = Kp_v * filtered_velocity_error.at(0);
    // ROS_INFO("filtered_velocity_error_0 = %f", filtered_velocity_error.at(0));
	integral_velocity = Ki_v * integral_velocity_error;
	derivative_velocity = Kd_v * filtered_velocity_error_deriv.at(0);
    // ROS_INFO("filtered_velocity_error_deriv_0 = %f", filtered_velocity_error_deriv.at(0));
	control_effort_velocity = proportional_velocity + integral_velocity + derivative_velocity;
      // ROS_DEBUG("Proportional velocity effort %f", proportional_velocity);
      // ROS_DEBUG("Integral velocity effort %f", integral_velocity);
      // ROS_DEBUG("Derivative velocity effort %f", derivative_velocity);
}
void pid_controller::pid::pub()
{
	custom_messages_biggie::control_effort control_effort_msg;

	control_effort_msg.header.stamp=ros::Time::now();
	control_effort_msg.type.data="PID";
	
	std_msgs::Float64 temp;

	temp.data=control_effort_velocity*velocity_scalar;	
	control_effort_msg.tau.push_back(temp);//X
	temp.data=0;	
	control_effort_msg.tau.push_back(temp);//Y
	temp.data=control_effort_heading*heading_scalar;	
	control_effort_msg.tau.push_back(temp);//Eta

	//ROS_DEBUG("control_effort_heading %f", control_effort_heading);
	//ROS_DEBUG("control_effort_velocity %f",control_effort_velocity);

	control_effort_pub.publish(control_effort_msg);
}

void pid_controller::pid::get_params()
{
	//ROS_DEBUG("Entering get_params, next stop check_pid_gains");
	//This style of obtaining params is used because it resolves the param relative to the namespace of the node
  	ros::param::get("pid/Kp_h", Kp_h);
  	ros::param::get("pid/Ki_h", Ki_h);
  	ros::param::get("pid/Kd_h", Kd_h);
  	ros::param::get("pid/anti_windup_limit_h", anti_windup_limit_h);
  	ros::param::get("pid/diff_filter_coefficient_h", cutoff_frequency_h);
  	ros::param::get("pid/heading_scalar", heading_scalar);

  	ros::param::get("pid/Kp_v", Kp_v);
  	ros::param::get("pid/Ki_v", Ki_v);
  	ros::param::get("pid/Kd_v", Kd_v);
  	ros::param::get("pid/anti_windup_limit_v", anti_windup_limit_v);
  	ros::param::get("pid/diff_filter_coefficient_v", cutoff_frequency_v);
  	ros::param::get("pid/velocity_scalar", velocity_scalar);
  	ros::param::get("pid/is_sim", is_sim);
  	ros::param::get("pid/R", R);

  	ros::param::param<std::vector<double>>("pid/max_thrust", max_thrust);

  	ROS_DEBUG("Kp_h %f, Ki_h %f, Kd_h %f", Kp_h, Ki_h, Kd_h);
  	ROS_DEBUG("Kp_v %f, Ki_v %f, Kd_v %f", Kp_v, Ki_v, Kd_v);

  	this->check_pid_gains();
}

void pid_controller::pid::check_pid_gains()
{
	ROS_DEBUG("Entering check_pid_gains, now its up to spin");
	if(!(Kp_h>=0. && Ki_h>=0. && Kd_h>=0.) ) // All 3 gains should be positive non zero
	{
    	ROS_WARN("All three heading gains (Kp, Ki, Kd) should have the same sign for stability.");
  	}

	if(!(Kp_v>=0. && Ki_v>=0. && Kd_v>=0.) ) // All 3 gains should be positive non zero
	{
    	ROS_WARN("All three velocity gains (Kp, Ki, Kd) should have the same sign for stability.");
  	}
}

void pid_controller::pid::run()
{
	this->design_filter();

	while(ros::ok())
	{
		if(newCommand&&newState)
		{
			newCommand=false;
			newState=false;
			this->set_error_ebs(); //sets the error every time a new state message is received
			this->set_timestep();
			this->integrate_error();
			this->anti_windup();
			this->filter_error();
			this->differentiate_error();
			this->accumulate_control_effort();
			this->pub();
		}
		ros::spinOnce();
		loop_rate.sleep();

	}
}


//Saving for a rainy day
//convert yaw angle from ENU to BFF
/*	
	float bffYawAngle;
	if(yaw_angle>=0&&yaw_angle<M_PI/2)
	{
		bffYawAngle=M_PI/2-yaw_angle;
	}
	else if(yaw_angle>=M_PI/2&&yaw_angle<M_PI)
	{
		bffYawAngle=yaw_angle-M_PI/2;
	}
	else if(yaw_angle<0&&yaw_angle>-M_PI/2)
	{
		bffYawAngle=yaw_angle-M_PI/2;
	}
	else
	{
		bffYawAngle=(3*M_PI/2)+yaw_angle;
	}

	//convert yaw angle from ENU [PI:0;0:-PI] to NED [0:2PI]
	//if(yaw_angle>=0&&yaw_angle<M_PI/2)
	//{
	//	yaw_angle=yaw_angle+3*M_PI/2;
	//}
	//else if(yaw_angle>=M_PI/2&&yaw_angle<M_PI)
	//{
	//	yaw_angle=yaw_angle-M_PI/2;
	//}
	//else if(yaw_angle<0&&yaw_angle>-M_PI/2)
	//{
	//	yaw_angle=(3*M_PI/2)+yaw_angle;
	//}
	//else
	//{
	//	yaw_angle=(3*M_PI/2)+yaw_angle;
	//}

*/

