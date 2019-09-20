#include "vehicle_allocation/scaled_underactuated_allocation.h"

static bool tau_flg = false;

using namespace std;
using namespace Eigen;

namespace alloc {

	scaledUnderactuatedAllocation::scaledUnderactuatedAllocation(ros::NodeHandle &nh) : alloc_nh(&nh), loop_rate(4)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

		control_effort_sub = alloc_nh->subscribe("control_effort", 10, &alloc::scaledUnderactuatedAllocation::tau_callback_, this);
		
		//actuation_pub = alloc_nh->advertise<custom_messages_biggie::control_effort>("/ship/actuation", 10); //published TAU = {X,Y,Eta}
		sim_port_pub = alloc_nh->advertise<std_msgs::Float32>("right_thrust_cmd", 10);
		sim_stbd_pub = alloc_nh->advertise<std_msgs::Float32>("left_thrust_cmd", 10);

		get_ros_params_();
		T_over_.resize(3,4);
		T_inv_over_.resize(3,4);

		// set up inverse of transformation matirix
		T_inv_under_ = ((Matrix2f() << 1.0, 1.0, -ly_,ly_).finished()).inverse();

		T_over_ <<	1.0,	0.0,	1.0,	0.0,
					0.0,	1.0,	0.0,	1.0,
					ly_,	-lx_,	-ly_,	-lx_;

		// Moore-Penrose pseudoinverse
		T_inv_over_ = T_over_.transpose()*((T_over_*T_over_.transpose()).inverse());
	}

	scaledUnderactuatedAllocation::~scaledUnderactuatedAllocation()
	{
		
	}

	Eigen::Vector4f scaledUnderactuatedAllocation::allocate_under(const Vector3f& ctrlForces)
	{
		//checks to see if the error is large enough to warrant cutting speed and fixing heading
		if(ctrlForces(2)>150||ctrlForces(2)<-150)
		{
			Vector2f tau((Vector2f() << 0.0, ctrlForces(2)).finished());
			// calculate the tranformation and actuator inputs
			Vector2f u = T_inv_under_*tau;
			// return u in four element manner
			return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
		}
		else
		{
			Vector2f tau((Vector2f() << ctrlForces(0), ctrlForces(2)).finished());
			//we scale tau in a coupled manner such that:
			//if both need scaling, we scale 1:1
			//if only one needs scaling, we scale 
			if(tau(0)>420&&tau(1)>360)
			{
				//scale both
				if(tau(0)>tau(1))
				{
					//scale moment by (moment/velocity)*maxMoment
					tau(1)=tau(1)*420/tau(0);
					//saturate velocity
					tau(0)=420;
				}
				else if(tau(0)>tau(1))
				{
					//scale velocity by (velocity/moment)*maxVelocity
					tau(0)=tau(0)*360/tau(1);
					//saturate velocity
					tau(1)=360;
				}
				else
				{
					tau(0)=420;
					tau(1)=360;
				}
			}
			else if(tau(0)<-300&&tau(1)<-360)
			{
				//scale both
				if(tau(0)<tau(1))
				{
					//scale moment by (moment/velocity)*maxMoment
					tau(1)=tau(1)*-300/tau(0);
					//saturate velocity
					tau(0)=-300;
				}
				else if(tau(0)>tau(2))
				{
					//scale velocity by (velocity/moment)*maxVelocity
					tau(0)=tau(0)*-360/tau(1);
					//saturate velocity
					tau(1)=-360;
				}
				else
				{
					tau(0)=-300;
					tau(1)=-360;
				}
			}
			else if(tau(0)>420 && tau(1)<360 && tau(1)>-360)
			{
				//scale moment by (moment/velocity)*maxMoment
				tau(1)=tau(1)*420/tau(0);
				//saturate velocity
				tau(0)=420;
			}
			else if(tau(0)<-300 && tau(1)<360 && tau(1)>-360)
			{
				//scale moment by (moment/velocity)*maxMoment
				tau(1)=tau(1)*-300/tau(0);
				//saturate velocity
				tau(0)=-300;
			}
			else if(tau(0)<420 && tau(0)>-300 && tau(1)>360)
			{
				//scale velocity by (velocity/moment)*maxVelocity
				tau(0)=tau(0)*360/tau(1);
				//saturate velocity
				tau(1)=360;
			}
			else if(tau(0)<420 && tau(0)>-300 && tau(1)<-360)
			{
				//scale velocity by (velocity/moment)*maxVelocity
				tau(0)=tau(0)*360/tau(1);
				//saturate velocity
				tau(1)=360;
			}
			else
			{
				ROS_DEBUG("No Saturation Required");
			}
			// calculate the tranformation and actuator inputs
			Vector2f u = T_inv_under_*tau;

			// scaled desired control force/moment on vehicle
			//if(tau(1)>0)
			//{
			//	u(0) -= (ctrlForces(2)/2);
			//	u(1) -= (ctrlForces(2)/2);
			//}
			//else
			//{
			//	u(0) += (ctrlForces(2)/2);
			//	u(1) += (ctrlForces(2)/2);
			//}
			/// return u in four element manner
			return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
		}
	}

	Eigen::Vector4f scaledUnderactuatedAllocation::allocate_over(const Eigen::Vector3f& ctrlForces)
	{/*
	  *	ALL ANGLES HERE USE DEGREE REPRESENTATION
	  */

		bool sat_flag = false;

		// return vector
		Vector4f sysInputs;

		// extended thrust representation
		Vector4f f; //[Fxp Fyp Fxs Fys]

		float Tp;
		float alphap;
		float Ts;
		float alphas;

		// calculate using MP Inverse of transfo
		f = T_inv_over_*ctrlForces;

		// calculate using simple trigonometric functions
		Tp = sqrt( pow(f(0),2) + pow(f(1),2) );
		Ts = sqrt( pow(f(2),2) + pow(f(3),2) );

		alphap = RAD2DEG(atan2(f(1),f(0)));
		alphas = RAD2DEG(atan2(f(3),f(2)));

		double pos_rev_limit =  180.0 + RAD2DEG(act_min_alpha_);
		double neg_rev_limit = -180.0 + RAD2DEG(act_max_alpha_);

		// not within deadband continue
		if(alphap >= pos_rev_limit)
		{
			Tp *= -1;
			alphap -= 180.0;
		}
		else if(alphap <= neg_rev_limit)
		{
			// reverse port direction
			Tp *= -1;
			alphap += 180.0;
		}

		if(alphas >= pos_rev_limit)
		{
			Ts *= -1;
			alphas -= 180.0;
		}
		else if(alphas <= neg_rev_limit)
		{
			// reverse port direction
			Ts *= -1;
			alphas += 180.0;
		}

		// check for actuator saturation
		if(Tp > act_max_T_)
		{
			sat_flag = true;
			Tp = act_max_T_;
		}
		else if(Tp < act_min_T_)
		{
			sat_flag = true;
			Tp = act_min_T_;
		}

		if(Ts > act_max_T_)
		{
			sat_flag = true;
			Ts = act_max_T_;
		}
		else if(Ts < act_min_T_)
		{
			sat_flag = true;
			Ts = act_min_T_;
		}

		// load all act inputs into a vecotr
		sysInputs << Tp, alphap, Ts, alphas;
		ROS_INFO("sysInputs are %f %f %f %f",Tp, alphap, Ts, alphas);

		// if there is saturation scale scale SCALE!
		if(sat_flag)
		{
			// calculate new extended thrust representation
			Vector4f f_max, sigma;
			f_max = actState2ExtThrust(sysInputs);

			for(int i=0; i<3; ++i)
			{
				// deal with divide by zero errors
				if(f_max(i) == 0) sigma(i) = 0.0;
				else sigma(i) = f(i)/f_max(i);
			}

			// find maximum of sigma and divide
			f = f/sigma.lpNorm<Infinity>();

			// recalculate actuator inputs
			Tp = sqrt( pow(f(0),2) + pow(f(1),2) );
			Ts = sqrt( pow(f(2),2) + pow(f(3),2) );

			alphap = RAD2DEG(atan2(f(1),f(0)));
			alphas = RAD2DEG(atan2(f(3),f(2)));

			/////////////////////
			// Rerun Heuristic //
			/////////////////////

			// not within deadband continue
			if(alphap >= pos_rev_limit)
			{
				Tp *= -1;
				alphap -= 180.0;
			}
			else if(alphap <= neg_rev_limit)
			{
				// reverse port direction
				Tp *= -1;
				alphap += 180.0;
			}

			if(alphas >= pos_rev_limit)
			{
				Ts *= -1;
				alphas -= 180.0;
			}
			else if(alphas <= neg_rev_limit)
			{
				// reverse port direction
				Ts *= -1;
				alphas += 180.0;
			}

			// check for actuator saturation
			if(Tp > act_max_T_)
			{
				sat_flag = true;
				Tp = act_max_T_;
			}
			else if(Tp < act_min_T_)
			{
				sat_flag = true;
				Tp = act_min_T_;
			}

			if(Ts > act_max_T_)
			{
				sat_flag = true;
				Ts = act_max_T_;
			}
			else if(Ts < act_min_T_)
			{
				sat_flag = true;
				Ts = act_min_T_;
			}

		}

		// deadband heuristic
		if( (alphap > RAD2DEG(act_max_alpha_) && alphap < pos_rev_limit) || (alphap < RAD2DEG(act_min_alpha_) && alphap > neg_rev_limit))
			Tp = 0.0;

		if( (alphas > RAD2DEG(act_max_alpha_) && alphas < pos_rev_limit) || (alphas < RAD2DEG(act_min_alpha_) && alphas > neg_rev_limit))
			Ts = 0.0;

		// saturate 
		alphap = SAT(alphap,RAD2DEG(act_min_alpha_),RAD2DEG(act_max_alpha_));
		alphas = SAT(alphas,RAD2DEG(act_min_alpha_),RAD2DEG(act_max_alpha_));

		return (Vector4f() << Tp, DEG2RAD(alphap), Ts, DEG2RAD(alphas)).finished();
	}

	Eigen::Vector4f scaledUnderactuatedAllocation::actState2ExtThrust(const Vector4f& state)
	{
		// convert from actuators states [Tp alphap Ts alphas] to extended
		// thrust representation.
		// ALL ANGLES ARE IN DEGREES.

		Vector4f f;

		f << state(0)*cos(DEG2RAD(state(1))), state(0)*sin(DEG2RAD(state(1))),
				state(2)*cos(DEG2RAD(state(3))), state(2)*sin(DEG2RAD(state(3)));

		return f;
	}

	void scaledUnderactuatedAllocation::get_ros_params_(void)
	{
		// get actuator configuration parameters
		ros::param::get("ctrl/alloc/lx",lx_);
		ros::param::get("ctrl/alloc/ly",ly_);

		// forces induced on vehicle limit
		vector<double> v_tau;
		ros::param::get("ctrl/ctrlr/tau_max",v_tau);

		// store in tau max parameters
		tau_max_.X = v_tau[0];
		tau_max_.Y = v_tau[1];
		tau_max_.N = v_tau[2];

		ros::param::get("ctrl/ctrlr/tau_min",v_tau);

		// store in tau min parameters
		tau_min_.X = v_tau[0];
		tau_min_.Y = v_tau[1];
		tau_min_.N = v_tau[2];

		// actuator limits
		ros::param::get("ctrl/alloc/T_max",act_max_T_);
		ros::param::get("ctrl/alloc/T_min",act_min_T_);

		ros::param::get("ctrl/alloc/alpha_max",act_max_alpha_);
		ros::param::get("ctrl/alloc/alpha_min",act_min_alpha_);
  		
  		ros::param::get("pid/is_sim", is_sim_);
	}

	void scaledUnderactuatedAllocation::tau_callback_(const custom_messages_biggie::control_effort::ConstPtr& msg)
	{
		tau_flg = true;
		tau_msg.type = msg->type;
		tau_msg.tau = msg->tau;
	}

	void scaledUnderactuatedAllocation::publish_actuator_inputs_(const Vector4f& u)
	{
		//usv16_msgs::Usv16ActuatorInputs a;

		//a.timeNow = ros::Time::now();

		// timestamp
		//a.t 				 = msg->t;

		// actuator inputs
		//a.actuator_inputs[0] = u(0);
		//a.actuator_inputs[1] = u(1);
		//a.actuator_inputs[2] = u(2);
		//a.actuator_inputs[3] = u(3);

		//pub_->publish(a);
		if(is_sim_)
		{
			std_msgs::Float32 leftCmd;
			leftCmd.data=u(0)/210;
			sim_port_pub.publish(leftCmd);
			std_msgs::Float32 rightCmd;
			rightCmd.data=u(2)/210;
			sim_stbd_pub.publish(rightCmd);
		}
		else
		{
			std_msgs::Float32 leftCmd;
			leftCmd.data=u(0);
			sim_port_pub.publish(leftCmd);
			std_msgs::Float32 rightCmd;
			rightCmd.data=u(2);
			sim_stbd_pub.publish(rightCmd);
		}
	}

	int scaledUnderactuatedAllocation::run(void)
	{
		while(ros::ok())
		{
			if(tau_flg)
			{
				ROS_DEBUG("before Vector3f tau generated");
				// convert from message to Eigen vector
				Vector3f tau = (Vector3f() << tau_msg.tau[0].data, tau_msg.tau[1].data, tau_msg.tau[2].data).finished();

				ROS_DEBUG("before publisher");
				// allocate and publish to motors
				//check to see if the publishes is PID (underactuated) or SM_SK (overactuated)

				publish_actuator_inputs_(this->allocate_under(tau));

				// toggle message handle
				tau_flg = false;
			}
			ros::spinOnce();
		}
		return EXIT_FAILURE;
	}
}//end alloc namespace