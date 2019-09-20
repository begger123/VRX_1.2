#include "vehicle_allocation/scaled_underactuated_allocation_sim.h"

static bool tau_flg = false;

using namespace std;
using namespace Eigen;

namespace alloc {

	scaledUnderactuatedAllocationSim::scaledUnderactuatedAllocationSim(ros::NodeHandle &nh) : alloc_nh(&nh), loop_rate(4)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

		control_effort_sub = alloc_nh->subscribe("control_effort", 10, &alloc::scaledUnderactuatedAllocationSim::tau_callback_, this);
		
		//actuation_pub = alloc_nh->advertise<custom_messages_biggie::control_effort>("/ship/actuation", 10); //published TAU = {X,Y,Eta}
		sim_port_pub = alloc_nh->advertise<std_msgs::Float32>("right_thrust_cmd", 10);
		sim_stbd_pub = alloc_nh->advertise<std_msgs::Float32>("left_thrust_cmd", 10);

		get_ros_params_();

		// set up inverse of transformation matirix
		T_inv_ = ((Matrix2f() << 1.0, 1.0, -ly_,ly_).finished()).inverse();
	}

	scaledUnderactuatedAllocationSim::~scaledUnderactuatedAllocationSim()
	{
		
	}

	////////////////////
	// Public Methods //
	////////////////////
	Vector4f scaledUnderactuatedAllocationSim::allocate(const Vector3f& ctrlForces)
	{
		//checks to see if the error is large enough to warrant cutting speed and fixing heading
		if(ctrlForces(2)>150||ctrlForces(2)<-150)
		{
			Vector2f tau((Vector2f() << 0.0, ctrlForces(2)).finished());
			// calculate the tranformation and actuator inputs
			Vector2f u = T_inv_*tau;
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
			Vector2f u = T_inv_*tau;

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
			// return u in four element manner
			return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
		}
	}


	int scaledUnderactuatedAllocationSim::run(void)
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
				publish_actuator_inputs_(this->allocate(tau));

				// toggle message handle
				tau_flg = false;
			}
			ros::spinOnce();
		}
		return EXIT_FAILURE;
	}

	/////////////////////
	// Private Methods //
	/////////////////////

	void scaledUnderactuatedAllocationSim::get_ros_params_(void)
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
	}

	void scaledUnderactuatedAllocationSim::publish_actuator_inputs_(const Vector4f& u)
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

		std_msgs::Float32 leftCmd;
		leftCmd.data=u(0)/210;
		sim_port_pub.publish(leftCmd);
		std_msgs::Float32 rightCmd;
		rightCmd.data=u(2)/210;
		sim_stbd_pub.publish(rightCmd);
	}

	void scaledUnderactuatedAllocationSim::tau_callback_(const custom_messages_biggie::control_effort::ConstPtr& msg)
	{
		tau_flg = true;
		tau_msg.type = msg->type;
		tau_msg.tau = msg->tau;
	}
}//end alloc namespace
