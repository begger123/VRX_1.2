#include "vehicle_allocation/scaled_overactuated_allocation_sim.h"

static bool tau_flg = false;

using namespace std;
using namespace Eigen;

namespace alloc {

	scaledOveractuatedAllocationSim::scaledOveractuatedAllocationSim(ros::NodeHandle &nh) : alloc_nh(&nh), loop_rate(4)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

		control_effort_sub = alloc_nh->subscribe("control_effort_sk", 10, &alloc::scaledOveractuatedAllocationSim::tau_callback_, this);
		
		//actuation_pub = alloc_nh->advertise<custom_messages_biggie::control_effort>("/ship/actuation", 10); //published TAU = {X,Y,Eta}
		sim_port_pub = alloc_nh->advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);
		sim_port_ang_pub = alloc_nh->advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);
		sim_stbd_pub = alloc_nh->advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);
		sim_stbd_ang_pub = alloc_nh->advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);

		get_ros_params_();
        T_over_.resize(3,4);
        T_inv_over_.resize(3,4);

		// set up inverse of transformation matirix
		T_inv_ = ((Matrix2f() << 1.0, 1.0, -ly_,ly_).finished()).inverse();

        T_over_ <<  1.0,    0.0,    1.0,    0.0,
                    0.0,    1.0,    0.0,    1.0,
                    ly_,    -lx_,   -ly_,   -lx_;

        // Moore-Penrose pseudoinverse
        T_inv_over_ = T_over_.transpose()*((T_over_*T_over_.transpose()).inverse());
 
	}

	scaledOveractuatedAllocationSim::~scaledOveractuatedAllocationSim()
	{
		
	}

	////////////////////
	// Public Methods //
	////////////////////
/*	Vector4f scaledOveractuatedAllocationSim::allocate(const Vector3f& ctrlForces)
	{
        //checks to see if the heading error is large enough to warrant cutting speed and fixing heading
        if(sqrt(ctrlForces(2)*ctrlForces(2))>moment_scalar*sqrt(ctrlForces(0)*ctrlForces(0)))
        {
            ROS_WARN("Adjusting heading, error is too large");
            Vector2f tau((Vector2f() << 0.0, ctrlForces(2)).finished());
            // calculate the tranformation and actuator inputs
            Vector2f u = T_inv_over_*tau;
            // return u in four element manner
            return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
        }
		else
		{
		    Vector2f tau((Vector2f() << ctrlForces(0), ctrlForces(2)).finished());
            //Reminder: tau(0)=velocity, tau(1)=moment 

            //Both are out of bounds positive
            if(tau(0)>T_Max&&tau(1)>P_Max)
            {
                //velocity is more out of bounds than moment, so make velocity=max, and moment<max
                if(tau(0)>tau(1))
                {
                    //scale moment by (moment/velocity)*maxMoment
                    tau(1)=tau(1)*T_Max/tau(0);
                    //saturate velocity
                    tau(0)=T_Max;
                }
                //moment is more out of bounds than velocity, so make moment=max, and velocity<max
                else if(tau(0)<tau(1))
                {
                    //scale velocity by (velocity/moment)*maxVelocity
                    tau(0)=tau(0)*P_Max/tau(1);
                    //saturate velocity
                    tau(1)=P_Max;
                }
                //unlikely, but both are equally out of bounds, so just set both to max
                else
                {
                    tau(0)=T_Max;
                    tau(1)=P_Max;
                }
            }
            //Both are out of bounds negative
            else if(tau(0)<T_Min&&tau(1)<P_Min)
            {
                //velocity is more out of bounds than moment, so make velocity=max, and moment<max
                if(tau(0)<tau(1))
                {
                    //scale moment by (moment/velocity)*maxMoment
                    tau(1)=tau(1)*T_Min/tau(0);
                    //saturate velocity
                    tau(0)=T_Min;
                }
                //moment is more out of bounds than velocity, so make moment=max, and velocity<max
                else if(tau(0)>tau(1))
                {
                    //scale velocity by (velocity/moment)*maxVelocity
                    tau(0)=tau(0)*P_Min/tau(1);
                    //saturate velocity
                    tau(1)=P_Min;
                }
                //unlikely, but both are equally out of bounds, so just set both to max
                else
                {
                    tau(0)=T_Min;
                    tau(1)=P_Min;
                }
            }
            //velocity is out of bounds positive, moment is out of bounds negative
            else if(tau(0)>T_Max&&tau(1)<P_Min)
            {
                //velocity is more out of bounds than moment, so make velocity=max, and moment<max
                if(tau(0)>-tau(1))
                {
                    //scale moment by (moment/velocity)*maxMoment
                    tau(1)=tau(1)*T_Max/tau(0);
                    //saturate velocity
                    tau(0)=T_Max;
                }
                //moment is more out of bounds than velocity, so make moment=max, and velocity<max
                else if(tau(0)<-tau(1))
                {
                    //scale velocity by (velocity/moment)*maxVelocity
                    tau(0)=tau(0)*P_Min/tau(1);
                    //saturate velocity
                    tau(1)=P_Min;
                }
                //unlikely, but both are equally out of bounds, so just set both to max
                else
                {
                    tau(0)=T_Max;
                    tau(1)=P_Min;
                }
            }
            //velocity is out of bounds negative, moment is out of bounds positive
            else if(tau(0)<T_Min&&tau(1)>P_Max)
            {
                //velocity is more out of bounds than moment, so make velocity=max, and moment<max
                if(-tau(0)>tau(1))
                {
                    //scale moment by (moment/velocity)*maxMoment
                    tau(1)=tau(1)*T_Min/tau(0);
                    //saturate velocity
                    tau(0)=T_Min;
                }
                //moment is more out of bounds than velocity, so make moment=max, and velocity<max
                else if(-tau(0)<tau(1))
                {
                    //scale velocity by (velocity/moment)*maxVelocity
                    tau(0)=tau(0)*P_Max/tau(1);
                    //saturate velocity
                    tau(1)=P_Max;
                }
                //unlikely, but both are equally out of bounds, so just set both to max
                else
                {
                    tau(0)=T_Min;
                    tau(1)=P_Max;
                }
            }
            //velocity is out of bounds positive, but moment is in bounds
            else if(tau(0)>T_Max && tau(1)<P_Max && tau(1)>P_Min)
            {
                //scale moment by (moment/velocity)*maxMoment
                tau(1)=tau(1)*T_Max/tau(0);
                //saturate velocity
                tau(0)=T_Max;
            }
            //velocity is out of bounds negative, but moment is in bounds
            else if(tau(0)<T_Min && tau(1)<P_Max && tau(1)>P_Min)
            {
                //scale moment by (moment/velocity)*maxMoment
                tau(1)=tau(1)*T_Min/tau(0);
                //saturate velocity
                tau(0)=T_Min;
            }
            //moment is out of bounds negative, but velocity is in bounds
            else if(tau(0)<T_Max && tau(0)>T_Min && tau(1)>P_Max)
            {
                //scale velocity by (velocity/moment)*maxVelocity
                tau(0)=tau(0)*P_Max/tau(1);
                //saturate velocity
                tau(1)=P_Max;
            }
            //moment is out of bounds negative, but velocity is in bounds
            else if(tau(0)<T_Max && tau(0)>T_Min && tau(1)<P_Min)
            {
                //scale velocity by (velocity/moment)*maxVelocity
                tau(0)=tau(0)*P_Max/tau(1);
                //saturate moment
                tau(1)=P_Max;
            }
            else
            {
                ROS_WARN("Tau 0 is %f", tau(0));
                ROS_WARN("Tau 1 is %f", tau(1));
                ROS_DEBUG("No Saturation Required");
            }
            // calculate the tranformation and actuator inputs
            Vector2f u = T_inv_over_*tau;

            /// return u in four element manner
            return (Vector4f() << u(0), 0.0, u(1), 0.0).finished();
        }
	}
*/
   	Eigen::Vector4f scaledOveractuatedAllocationSim::allocate_over(const Eigen::Vector3f& ctrlForces)
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
    	
    	ROS_DEBUG("ctrlForces is %f %f %f",ctrlForces(0), ctrlForces(1), ctrlForces(2));
        ROS_DEBUG("extended thrust is %f %f %f %f",f(0), f(1), f(2), f(3));
    
    	// calculate using simple trigonometric functions
    	Tp = sqrt( pow(f(0),2) + pow(f(1),2) );
    	Ts = sqrt( pow(f(2),2) + pow(f(3),2) );
    
    	alphap = RAD2DEG(atan2(f(1),f(0)));
    	alphas = RAD2DEG(atan2(f(3),f(2)));
    
    	double pos_rev_limit =  180.0 + RAD2DEG(act_min_alpha_);
    	double neg_rev_limit = -180.0 + RAD2DEG(act_max_alpha_);

        ROS_WARN("pos_rev_limit is %f", pos_rev_limit);
        ROS_WARN("neg_rev_limit is %f", neg_rev_limit);
    
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
    	//ROS_DEBUG("sysInputs are %f %f %f %f",Tp, alphap, Ts, alphas);
    
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
    		//if(alphap >= pos_rev_limit)
    		//{
    		//	Tp *= -1;
    		//	alphap -= 180.0;
    		//}
    		//else if(alphap <= neg_rev_limit)
    		//{
    		//	// reverse port direction
    		//	Tp *= -1;
    		//	alphap += 180.0;
    		//}
    
    		//if(alphas >= pos_rev_limit)
    		//{
    		//	Ts *= -1;
    		//	alphas -= 180.0;
    		//}
    		//else if(alphas <= neg_rev_limit)
    		//{
    		//	// reverse port direction
    		//	Ts *= -1;
    		//	alphas += 180.0;
    		//}
    
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

    Eigen::Vector4f scaledOveractuatedAllocationSim::actState2ExtThrust(const Vector4f& state)
    {
    	// convert from actuators states [Tp alphap Ts alphas] to extended
    	// thrust representation.
    	// ALL ANGLES ARE IN DEGREES.
    
    	Vector4f f;
    
    	f << state(0)*cos(DEG2RAD(state(1))), state(0)*sin(DEG2RAD(state(1))),
    			state(2)*cos(DEG2RAD(state(3))), state(2)*sin(DEG2RAD(state(3)));
    
    	return f;
    }

	int scaledOveractuatedAllocationSim::run(void)
	{
		while(ros::ok())
		{
			if(tau_flg)
			{
				// convert from message to Eigen vector
				Vector3f tau = (Vector3f() << tau_msg.tau[0].data, tau_msg.tau[1].data, tau_msg.tau[2].data).finished();

				// allocate and publish to motors
				publish_actuator_inputs_(this->allocate_over(tau));

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

	void scaledOveractuatedAllocationSim::get_ros_params_(void)
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
        ros::param::get("ctrl/alloc/P_Max",P_Max);
        ros::param::get("ctrl/alloc/P_Min",P_Min);
        ros::param::get("ctrl/alloc/moment_scalar",moment_scalar);


		ros::param::get("ctrl/alloc/alpha_max",act_max_alpha_);
		ros::param::get("ctrl/alloc/alpha_min",act_min_alpha_);
	}

	void scaledOveractuatedAllocationSim::publish_actuator_inputs_(const Vector4f& u)
	{
		std_msgs::Float32 leftCmd;
        if(u(0)>=0)
        {
		    leftCmd.data=u(0)/act_max_T_;
        }
        else
        {
            leftCmd.data=u(0)/act_min_T_;
        }
        sim_port_pub.publish(leftCmd);
		
		std_msgs::Float32 leftAngCmd;
        if(u(1)>=0)
        {
		    //leftAngCmd.data=(u(1)-M_PI)/DEG2RAD(act_max_alpha_);
		    leftAngCmd.data=u(1);
        }
        else
        {
		    //leftAngCmd.data=-(u(1)+M_PI)/DEG2RAD(act_max_alpha_);
		    leftAngCmd.data=u(1);
        }
		sim_port_ang_pub.publish(leftAngCmd);
        
        //ROS_WARN("u(1), %f",u(1));
        //ROS_WARN("leftCmd.data, %f",leftAngCmd.data);
        
        
        std_msgs::Float32 rightCmd;
        if(u(2)>=0)
        {
		    rightCmd.data=u(2)/act_max_T_;
        }
        else
        {
		    rightCmd.data=u(2)/act_min_T_;
        }
		sim_stbd_pub.publish(rightCmd);

		std_msgs::Float32 rightAngCmd;
        if(u(3)>=0)
        {
		    //rightAngCmd.data=(u(3)-180)/DEG2RAD(act_max_alpha_);
		    rightAngCmd.data=u(3);
        }
        else
        {
		    //rightAngCmd.data=-(u(3)+180)/DEG2RAD(act_max_alpha_);
		    rightAngCmd.data=u(3);
        }
		sim_stbd_ang_pub.publish(rightAngCmd);
    	
        ROS_WARN("Finall Alloc outputs are %f %f %f %f",leftCmd.data, leftAngCmd.data, rightCmd.data, rightAngCmd.data);
	}

	void scaledOveractuatedAllocationSim::tau_callback_(const custom_messages_biggie::control_effort::ConstPtr& msg)
	{
		tau_flg = true;
		tau_msg.type = msg->type;
		tau_msg.tau = msg->tau;
	}
}//end alloc namespace
