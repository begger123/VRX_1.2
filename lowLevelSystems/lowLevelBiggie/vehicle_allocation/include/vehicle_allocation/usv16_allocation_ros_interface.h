#ifndef USV16_ALLOCATION_ROS_INTERFACE_HEADER_INCLUDED
#define USV16_ALLOCATION_ROS_INTERFACE_HEADER_INCLUDED

#include <ros/ros.h>
#include <Eigen/Core>

#include <vehicle_allocation/ctrl_types.h>
#include "custom_messages_biggie/control_effort.h"
//#include "usv16_ctrl/Usv16Tau.h"
//#include "usv16_msgs/Usv16ActuatorInputs.h"
#include "std_msgs/Float32.h"

namespace ctrl {

	//////////////////////////
	// Forward Declarations //
	//////////////////////////

	class BaseControlAllocation;

	class Usv16AllocationRosInterface
	{
	public:

		// control allocation types
		enum AllocationTypes {
			ERROR,							/**< Error occured when trying to parse ROS params */
			AD_HOC_UNDERACTUATED,			/**< Ad hoc underactuated solution (exponentially scaled) */
			SCALED_UNDERACTUATED,			/**< Scaled underactuated solution */
			EXTENDED_THRUST_OVERACTUATED	/**< Overactuated, extended thrust solution */
		};

		/////////////////
		// Constructor //
		/////////////////

		Usv16AllocationRosInterface(){};
		Usv16AllocationRosInterface(int argc, char** argv);
		
		////////////////////
		// Public Methods //
		////////////////////

		int run(void);

		////////////////
		// Destructor //
		////////////////

		~Usv16AllocationRosInterface();

	private:

		/////////////////////
		// Private Members //
		/////////////////////

		/**
		 * @brief      ROS Node Handle pointer
		 */
		std::unique_ptr<ros::NodeHandle> 	nh_ = nullptr;	

		/**
		 * @brief      ROS Subscriber pointer to "/tau" topic.
		 */
		std::unique_ptr<ros::Subscriber> 	sub_ = nullptr;	

		/**
		 * @brief      ROS Publisher pointer to "/ship/actuation" topic.
		 */
		std::unique_ptr<ros::Publisher> 	pub_ = nullptr;		

		/**
		 * @brief      ROS Publisher pointer to "/right_thrust_cmd" topic.
		 */
		std::unique_ptr<ros::Publisher> 	sim_port_pub_ = nullptr;		

		/**
		 * @brief      ROS Publisher pointer to "/left_thrust_cmd" topic.
		 */
		std::unique_ptr<ros::Publisher> 	sim_stbd_pub_ = nullptr;		


		BaseControlAllocation * alloc_ = nullptr;

		custom_messages_biggie::control_effort	tau_msg; // desired force message

		// allocation specific parameters
		tau_t tau_max_;		// maximum force produces by actuators on vehicle
		tau_t tau_min_;		// minimum force produced by actuators on vehicle

		double act_max_T_;	// maximum thrust from actuators
		double act_min_T_;	// maximum thrust from actuators

		double act_max_alpha_;	// maximum actuator angle from actuators
		double act_min_alpha_;	// maximum actuator angle from actuators

		double lx_;				// longitudinal separation of actuators from cg
		double ly_;				// lateral separation of actuators from cg

		double beta_;			// ad hoc uneractuated exponential scaling

		/////////////////////
		// Private Methods //
		/////////////////////

		void get_ros_params_(void);

		void publish_actuator_inputs_(const Eigen::Vector4f& u);

		// call back function for subscriber
		void tau_callback_(const custom_messages_biggie::control_effort& msg);


	};

}
#endif
