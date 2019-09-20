#ifndef CTRL_TYPES_HEADER_INCLUDED
#define CTRL_TYPES_HEADER_INCLUDED

#include <cmath>

// desired position structure
/**
 * @brief	"Pose" data structure denoting position and orientation in 3DOF in NED. 
 * @details "Pose" data structure denoting position and orientation in three degrees of 
 * 			freedom in the North-East-Down coordinate system. All provided operations wrap
 * 			desired heading. Multiplication is used for scaling purposes in position only.
 * 
 */
struct pose_t
{
	/**@brief North position (m) */
	double x;

	/**@brief East position (m) */
	double y;

	/**@brief Orientation (yaw) (rad) */
	double psi;

	pose_t& operator +=(const pose_t& p)
	{
		x += p.x;
		y += p.y;
		psi += p.psi; // wrap this
		
		// wrap
		if 		(psi > M_PI)  psi-=2*M_PI; 
		else if (psi < -M_PI) psi+=2*M_PI; 

		return *this;
	}
	
	pose_t& operator -=(const pose_t& p)
	{
		x -= p.x;
		y -= p.y;
		psi -= p.psi; // wrap this

		// wrap
		if 		(psi > M_PI)  psi-=2*M_PI; 
		else if (psi < -M_PI) psi+=2*M_PI; 

		return *this;
	}

	pose_t operator +(const pose_t& p1) const
	{
		pose_t p2 = *this;

		p2.x += p1.x;
		p2.y += p1.y;
		p2.psi += p1.psi; // wrap this
		
		// wrap
		if 		(p2.psi > M_PI)  p2.psi-=2*M_PI; 
		else if (p2.psi < -M_PI) p2.psi+=2*M_PI; 

		return p2;
	}

	pose_t operator -(const pose_t& p1) const
	{
		pose_t p2 = *this;

		p2.x -= p1.x;
		p2.y -= p1.y;
		p2.psi -= p1.psi; // wrap this
		
		// wrap
		if 		(p2.psi > M_PI)  p2.psi-=2*M_PI; 
		else if (p2.psi < -M_PI) p2.psi+=2*M_PI; 

		return p2;
	}
	
	pose_t operator *(const double& lf) const
	{
		pose_t p = *this;
		p.x *= lf;
		p.y *= lf;

		return p;
	}

	pose_t operator /(const double& lf) const
	{
		pose_t p = *this;
		p.x /= lf;
		p.y /= lf;
		p.psi /= lf;

		return p;
	}

	// relational operators
	bool operator ==(const pose_t& p)
	{
		if(x == p.x
		&& y == p.y
		&& psi == p.psi)
			return true;
		else
			return false;
	}
};

// desired motion primitives structure
/**
 * @brief 	Motion primitive data structure composed of \p pose and an associated desired speed.
 * 
 */
struct motion_primitives_t {

	pose_t pose; 	/**< \p pose_t pose in NED ([x,y,psi]^T) */
	double u;		/**< Surge velocity in m/s */

	motion_primitives_t& operator +=(const motion_primitives_t& m)
	{
		pose += m.pose;
		return *this;
	}

	motion_primitives_t& operator -=(const motion_primitives_t& m)
	{
		pose -= m.pose;
		return *this;
	}

};

/**
 * @brief      State of the vehicle
 */
struct state_t {

	/**
	 * @brief 		3DOF pose as [x (North, m), y (East, m), psi (Orientation, rad)]^T
	 */
	pose_t pose;

	/**
	 * @brief      3DOF velocities as [u (Surge, m/s), v (Sway, m/s), r (Yaw, rad/s)]^T
	 */
	double vel[3];

	/**
	 * @brief      3DOF accelerations as [u (Surge, m/s^2), v (Sway, m/s^2), r (Yaw, rad/s^2)]^T
	 */		
	double acc[3];
};

/**
 * @brief      Output forces and torque from controller.
 */
struct tau_t {

	/**
	 * @brief      Surge thrust (N)
	 */		
	double X;

	/**
	 * @brief      Sway thrust (N)
	 */		
	double Y;

	/**
	 * @brief      Yaw torque (Nm)
	 */		
	double N;

	bool operator ==(const tau_t& tau)
	{
		if(X == tau.X 
		  && Y == tau.Y
		  && N == tau.N)
			return true;
		else
			return false;
	}

};


#endif
