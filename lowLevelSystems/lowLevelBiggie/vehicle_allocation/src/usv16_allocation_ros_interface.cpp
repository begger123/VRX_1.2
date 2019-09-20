#include <vehicle_allocation/usv16_allocation_ros_interface.h>

#include <Eigen/Core>

#include "vehicle_allocation/global.h"
#include "vehicle_allocation/usv16_allocation.h"
#include "std_msgs/String.h"

#define ROS_FILE_INFO __FILE__ << ':' << __LINE__ << ' '

static bool tau_flg_ = false;

using namespace std;
using namespace Eigen;


namespace ctrl {

	Usv16AllocationRosInterface::Usv16AllocationRosInterface(int argc, char** argv)
	{
		// initialize ros
		ros::init(argc,argv,"usv16_alloc");

		// initialize node handle
		nh_ = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

		// "tau" topic
		sub_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
		nh_->subscribe("control_effort", 10, &Usv16AllocationRosInterface::tau_callback_, this)));

		// set up publisher
		//pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
		//	nh_->advertise<usv16_msgs::Usv16ActuatorInputs>("ship/actuation",10)));

		// set up vmrc port publisher
		sim_port_pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
			nh_->advertise<std_msgs::Float32>("right_thrust_cmd",10)));

		// set up vmrc stbd publisher
		sim_stbd_pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
			nh_->advertise<std_msgs::Float32>("left_thrust_cmd",10)));

		// populate parameters
		//get_ros_params_();
	}

	Usv16AllocationRosInterface::~Usv16AllocationRosInterface()
	{
		delete alloc_;
	}

	////////////////////
	// Public Methods //
	////////////////////

	int Usv16AllocationRosInterface::run(void)
	{
		std_msgs::String prv_alloc_type; // allocation type

		while(ros::ok())
		{

			if(tau_flg_)
			{
				ROS_DEBUG("before string compare");
				if(strcmp(prv_alloc_type.data.c_str(),tau_msg.type.data.c_str())!=0)
				{
					ROS_DEBUG("inside strcmp");
					delete alloc_; alloc_ = nullptr;

					if(strcmp(tau_msg.type.data.c_str(),"PID")==0)
					{

						ROS_DEBUG("before alloc_ generated");
						alloc_ = new Usv16ScaledUnderactuatedAllocation(tau_max_, tau_min_, 
								act_max_T_, act_min_T_, act_max_alpha_, act_min_alpha_,ly_);  
					}
					// Evaluate and send
					prv_alloc_type = tau_msg.type;
				}
				
				ROS_DEBUG("before Vector3f tau generated");
				// convert from message to Eigen vector
				Vector3f tau = (Vector3f() << tau_msg.tau[0].data, tau_msg.tau[1].data, tau_msg.tau[2].data).finished();

				ROS_DEBUG("before publisher");
				// allocate and publish to motors
				publish_actuator_inputs_(alloc_->allocate(tau));

				// toggle message handle
				tau_flg_ = false;
			}

			/////////////////////////////////////
			// Do other things if necessary... //
			/////////////////////////////////////

			ros::spinOnce();
		}

		return EXIT_FAILURE;
	}

	/////////////////////
	// Private Methods //
	/////////////////////

	void Usv16AllocationRosInterface::get_ros_params_(void)
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


		ros::param::get("ctrl/alloc/ad_hoc/beta",beta_);
	}

	void Usv16AllocationRosInterface::publish_actuator_inputs_(const Vector4f& u)
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
		leftCmd.data=u(0);
		sim_port_pub_->publish(leftCmd);
		std_msgs::Float32 rightCmd;
		rightCmd.data=u(2);
		sim_stbd_pub_->publish(rightCmd);
	}

	void Usv16AllocationRosInterface::tau_callback_(const custom_messages_biggie::control_effort& msg)
	{
		ROS_DEBUG("Inside of tau callback");
		tau_flg_ = true;
		tau_msg.type = msg.type;
		tau_msg.tau = msg.tau;
	}



}
