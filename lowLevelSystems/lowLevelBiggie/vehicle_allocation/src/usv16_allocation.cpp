#include <vehicle_allocation/usv16_allocation.h>

#include <iostream>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace ctrl {

	/************************************

	  Scaled Underactuated Allocation

	*************************************/

	Usv16ScaledUnderactuatedAllocation::Usv16ScaledUnderactuatedAllocation(const tau_t& tau_max, const tau_t& tau_min, 
		const double act_max_T, const double act_min_T,
		const double act_max_alpha, const double act_min_alpha,
		const double ly) :
	BaseControlAllocation(tau_max,tau_min,act_max_T,act_min_T,act_max_alpha,act_min_alpha),
	ly_(ly)
	{
		// set up inverse of transformation matirix
		T_inv_ = ((Matrix2f() << 1.0, 1.0, ly_,-ly_).finished()).inverse();
	}

	Vector4f Usv16ScaledUnderactuatedAllocation::allocate(const Vector3f& ctrlForces)
	{
		//checks to see if the error is large enough to warrant cutting speed and fixing heading
		if(ctrlForces(2)>210||ctrlForces(2)<-210)
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
			// calculate the tranformation and actuator inputs
			Vector2f u = T_inv_*tau;

			// scaled desired control force/moment on vehicle
			if(tau(1)>0)
			{
				u(0) -= (ctrlForces(2)/2);
				u(1) -= (ctrlForces(2)/2);
			}
			else
			{
				u(0) += (ctrlForces(2)/2);
				u(1) += (ctrlForces(2)/2);
			}

			// return u in four element manner
			return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
		}
	}

	/*******************************

		Overactuated Allocation

	*******************************/

	//////////////////////
	// Public Functions //
	//////////////////////

	Usv16OveractuatedAllocation::Usv16OveractuatedAllocation(const tau_t& tau_max, const tau_t& tau_min, 
			const double act_max_T, const double act_min_T,
			const double act_max_alpha, const double act_min_alpha,
			const float& lx, const float& ly) :
	BaseControlAllocation(tau_max,tau_min,act_max_T,act_min_T,act_max_alpha,act_min_alpha)
	{	// set up MP pseudo inverse

		Matrix<float,3,4> T;

		// populate T
		T <<	1.0,	0.0,	1.0,	0.0,
				0.0,	1.0,	0.0,	1.0,
				ly,		-lx,	-ly,	-lx;

		// Moore-Penrose pseudoinverse
		T_inv_ = T.transpose()*((T*T.transpose()).inverse());
	};

	Vector4f Usv16OveractuatedAllocation::allocate(const Vector3f& ctrlForces)
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
		f = T_inv_*ctrlForces;

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

		return (Vector4f() << Tp, DEG2RAD(alphap),Ts, DEG2RAD(alphas)).finished();
	};

	/////////////////////
	// Private Methods //
	/////////////////////

	Vector4f Usv16OveractuatedAllocation::actState2ExtThrust(const Vector4f& state)
	{
		// convert from actuators states [Tp alphap Ts alphas] to extended
		// thrust representation.
		// ALL ANGLES ARE IN DEGREES.

		Vector4f f;

		f << state(0)*cos(DEG2RAD(state(1))), state(0)*sin(DEG2RAD(state(1))),
				state(2)*cos(DEG2RAD(state(3))), state(2)*sin(DEG2RAD(state(3)));

		return f;
	}
}
