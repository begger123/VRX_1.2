#ifndef __CONTROL_ALLOCATION_HEADER_INCLUDED
#define __CONTROL_ALLOCATION_HEADER_INCLUDED

#include <Eigen/Core>
#include <vehicle_allocation/global.h>
#include <vehicle_allocation/ctrl_types.h>
#include <ros/ros.h>

namespace ctrl {

	/**************************
		
			Base Class

	***************************/

	class BaseControlAllocation
	{
	public:

		/////////////////
		// Constructor //
		/////////////////

		BaseControlAllocation(const tau_t& tau_max, const tau_t& tau_min, 
			const double act_max_T, const double act_min_T, const double act_max_alpha, 
			const double act_min_alpha) : 
			tau_max_(tau_max), tau_min_(tau_min), 
			act_max_T_(act_max_T), act_min_T_(act_min_T), 
			act_max_alpha_(act_max_alpha), act_min_alpha_(act_min_alpha)
			{};

		/////////////////////
		// Public Function //
		/////////////////////

		virtual Eigen::Vector4f allocate(const Eigen::Vector3f& ctrlForces) = 0;

		////////////////
		// Destructor //
		////////////////

		~BaseControlAllocation(void){};

	protected:

		/////////////////////
		// Private Members //
		/////////////////////

		tau_t tau_max_;		// maximum force produces by actuators on vehicle
		tau_t tau_min_;		// minimum force produced by actuators on vehicle

		double act_max_T_;	// maximum thrust from actuators
		double act_min_T_;	// maximum thrust from actuators

		double act_max_alpha_;	// maximum actuator angle from actuators
		double act_min_alpha_;	// maximum actuator angle from actuators
	};

	/*****************************

	   Underactuated Allocation 

	*****************************/

	class Usv16AdHocUnderactuatedAllocation : public BaseControlAllocation
	{

	public:

		/////////////////
		// Constructor //
		/////////////////

		Usv16AdHocUnderactuatedAllocation(const tau_t& tau_max, const tau_t& tau_min, 
			const double act_max_T, const double act_min_T, 
			const double act_max_alpha, const double act_min_alpha,
			const float& beta, const float& ly);

		//////////////////////
		// Public Functions //
		//////////////////////

		virtual Eigen::Vector4f allocate(const Eigen::Vector3f& ctrlForces) final;

		////////////////
		// Destructor //
		////////////////

		~Usv16AdHocUnderactuatedAllocation(){};

	private:

		float beta_;
		float ly_; // lateral separation from actuators to motor

		Eigen::Vector3f limControllerOutputs(const Eigen::Vector3f& unlimCtrlForces) const;
		Eigen::Vector4f limCtrlForces2Inputs(const Eigen::Vector3f& limCtrlForces) const;
	};

	/***********************************

	   Scaled Underactuated Allocation 

	************************************/

	class Usv16ScaledUnderactuatedAllocation : public BaseControlAllocation
	{

	public:

		/////////////////
		// Constructor //
		/////////////////

		Usv16ScaledUnderactuatedAllocation(const tau_t& tau_max, const tau_t& tau_min, 
			const double act_max_T, const double act_min_T, 
			const double act_max_alpha, const double act_min_alpha, const double ly);

		//////////////////////
		// Public Functions //
		//////////////////////

		virtual Eigen::Vector4f allocate(const Eigen::Vector3f& ctrlForces) final;

		////////////////
		// Destructor //
		////////////////

		~Usv16ScaledUnderactuatedAllocation(){};

	private:

		double ly_; // lateral separation of actuators

		Eigen::Matrix2f T_inv_;

		Eigen::Vector3f limControllerOutputs(const Eigen::Vector3f& unlimCtrlForces) const;
		Eigen::Vector4f limCtrlForces2Inputs(const Eigen::Vector3f& limCtrlForces) const;
	};

	/*******************************
	
		Overactuated Allocation

	*******************************/

	class Usv16OveractuatedAllocation : public BaseControlAllocation
	{
	public:

		////////////////
		// Construtor //
		////////////////

		Usv16OveractuatedAllocation(const tau_t& tau_max, const tau_t& tau_min, 
			const double act_max_T, const double act_min_T, 
			const double act_max_alpha, const double act_min_alpha,
			const float& lx, const float& ly);

		/////////////
		// Getters //
		/////////////

		Eigen::Matrix<float,4,3> getT_inv(void) const {return T_inv_;};

		//////////////////////
		// Public Functions //
		//////////////////////

		virtual Eigen::Vector4f allocate(const Eigen::Vector3f& ctrlForces) final;

		////////////////
		// Destructor //
		////////////////

		~Usv16OveractuatedAllocation(){};

	private:

		////////////////////
		// Private Fields //
		////////////////////

		Eigen::Matrix<float,4,3> T_inv_; // transformation matrix inverse

		/////////////////////
		// Private Methods //
		/////////////////////

		Eigen::Vector4f actState2ExtThrust(const Eigen::Vector4f& state);
	};
}

#ifdef _DEBUG
#undef _DEBUG
#endif

#endif
