#!/usr/bin/env python
# Program made by Armando J. Sinisterra for VRX Challenge
# Based on paper: "Constrained Nonlinear Control Allocation With Singularity Avoidance Using
# Sequential Quadratic Programming" from T. A. Johansen et al
# QP function based on the following expression:
#
#   minimize (1/2)x'Px + q'x
#      x
#           subject to:
#           Gx <= h
#           Ax  = b
############################################################################################

import numpy as np
import rospy
from custom_messages_biggie.msg import control_effort
from std_msgs.msg import Float32
from qpsolvers import solve_qp
from rosgraph_msgs.msg import Clock

class QP:

    def __init__(self, left_thrust, left_angle, right_thrust, right_angle, gradC1, gradC2):

        # Subscribers
        self.control_effort_sub = rospy.Subscriber("/control_effort", control_effort, self.tau_callback)

        # Publishers
        self.left_thrust_pub  = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size=10)
        self.left_angle_pub   = rospy.Publisher("/wamv/thrusters/left_thrust_angle", Float32, queue_size=10)
        self.right_thrust_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size=10)
        self.right_angle_pub  = rospy.Publisher("/wamv/thrusters/right_thrust_angle", Float32, queue_size=10)

        # Variables
        self.lastTime = rospy.get_time()
        self.command = 0.0
        self.left_thrust = left_thrust
        self.left_angle = left_angle
        self.left_command = 0
        self.right_command = 0
        self.max_thrust = 250
        self.min_thrust = -100
        self.right_thrust = right_thrust
        self.right_angle = right_angle
        self.n = 3                              # tau dimensionality (/control_effort)
        self.m = 2                              # u dimensionality (u is a vector with left and right thust values)
        self.x = np.zeros((self.n+2*self.m, 1)) # This is the vector to be optimized [deltaU_left, deltaU_right, Sx, Sy, Sz, d_left_angle, d_right_angle]
        fact =  10
        self.w = 0.018*fact                     # 0.018 - quadratic term coeff. of the quadratic fit of the power function
        self.q = 10                             # weights for Q>0 which penalize the error s between commanded and achieved forces
        self.omega = 180                        # 180 - weights for OMEGA>0 for tunning angle rate (delta_alpha)
        self.lx = 1.3                           # bf x-magnitude of the thrusters
        self.ly = 0.915                         # bf y-magnitude of the thrusters
        self.rho = 100                          # weighting parameter of the singularity avoidance term
        self.eps = 0.0001                       # small term to avoid division by zero in singularity avoidance term
        self.gradC1 = gradC1                    # gradient of singularity avoidance (SA) term (not yet implemented)                    
        self.gradC2 = gradC2                    # gradient of singularity avoidance (SA) term (not yet implemented)

        # Boundary constraints
        self.u_min = np.array([self.min_thrust, self.min_thrust])
        self.u_max = np.array([self.max_thrust, self.max_thrust])
        self.max_angle  = np.pi/2.0;
        self.min_angle  = -np.pi/2.0;
        self.alpha_min = np.array([self.min_angle, self.min_angle])
        self.alpha_max = np.array([self.max_angle, self.max_angle])
        self.d_alpha = 0.3;      # 0.3927 at 4 Hz according to elapsed time
        self.delta_alpha_min = np.array([-self.d_alpha, -self.d_alpha])
        self.delta_alpha_max = np.array([self.d_alpha, self.d_alpha])
        self.s_min = np.array([-np.inf, -np.inf, -np.inf])
        self.s_max = np.array([np.inf, np.inf, np.inf])

        # Quadratic terms matrix
        self.P = 2*np.array([ [self.w, 0, 0, 0, 0, 0, 0],
                              [0, self.w, 0, 0, 0, 0, 0],
                              [0, 0, self.q, 0, 0, 0, 0],
                              [0, 0, 0, self.q, 0, 0, 0],
                              [0, 0, 0, 0, self.q, 0, 0],
                              [0, 0, 0, 0, 0, self.omega, 0],
                              [0, 0, 0, 0, 0, 0, self.omega] ])


    def tau_callback(self, msg):

        # Set some variables
        tau = np.array([msg.tau[0].data, msg.tau[1].data, msg.tau[2].data])
        lx = self.lx
        ly = self.ly
        left_thrust = self.left_thrust
        left_angle = self.left_angle
        right_thrust = self.right_thrust
        right_angle = self.right_angle
        u_o = np.array([left_thrust, right_thrust])
        alpha_o = np.array([left_angle, right_angle])

        # Linear terms matrix
        f = np.array([2*self.w*left_thrust, 2*self.w*right_thrust, 0, 0, 0, self.gradC1, self.gradC2])

        # Linear constraint
        B_o = np.array([ [np.cos(left_angle), np.cos(right_angle)],
                         [np.sin(left_angle), np.sin(right_angle)],
                         [-lx*np.sin(left_angle) + ly*np.cos(left_angle), -lx*np.sin(right_angle) - ly*np.cos(right_angle)] ])

        dBuda_o = np.array([ [-left_thrust*np.sin(left_angle), -right_thrust*np.sin(right_angle)],
                             [ left_thrust*np.cos(left_angle),  right_thrust*np.cos(right_angle)],
                             [ left_thrust*(-lx*np.cos(left_angle) - ly*np.sin(left_angle)),
                               right_thrust*(-lx*np.cos(right_angle) + ly*np.sin(right_angle))] ])

        A = np.concatenate((B_o, np.identity(self.n), dBuda_o), axis=1)
        b = tau - np.dot(B_o, u_o)

        # Singularity avoidance (SA) term (not yet implemented)
        #  C = self.rho/(self.eps + np.linalg.det(np.dot(B_o, B_o.)))

        # Define delta_alpha_min and delta_alpha_max
        y1 = np.zeros((self.m, 1))
        y2 = np.zeros((self.m, 1))

        for i in range(self.m):
            if (alpha_o[i] > 0):
                if ((self.alpha_max[i] - alpha_o[i]) < self.delta_alpha_max[i]):
                    y1[i] = 1
                    y2[i] = 0
            elif (alpha_o[i] < 0):
                if ((self.alpha_min[i] - alpha_o[i]) > self.delta_alpha_min[i]):
                    y1[i] = 0
                    y2[i] = 1
            else:
                y1[i] = 0
                y2[i] = 0

        LB_delta_alpha_min = np.zeros((self.m, 1))
        UB_delta_alpha_max = np.zeros((self.m, 1))
        for i in range(self.m):
            LB_delta_alpha_min[i] = self.delta_alpha_min[i] - y2[i]*(self.delta_alpha_min[i] - (self.alpha_min[i] - alpha_o[i]))
            UB_delta_alpha_max[i] = self.delta_alpha_max[i] - y1[i]*(self.delta_alpha_max[i] - (self.alpha_max[i] - alpha_o[i]))

        # Define lower and upper boundaries
        LB = np.array([self.u_min[0] - u_o[0],
                       self.u_min[1] - u_o[1],
                       self.s_min[0],
                       self.s_min[1],
                       self.s_min[2],
                       LB_delta_alpha_min[0],
                       LB_delta_alpha_min[1]])

        UB = np.array([self.u_max[0] - u_o[0],
                       self.u_max[1] - u_o[1],
                       self.s_max[0],
                       self.s_max[1],
                       self.s_max[2],
                       UB_delta_alpha_max[0],
                       UB_delta_alpha_max[1]])

        # Convert lower and upper boundary constraints into the expression associated to the G matrix
        G = np.concatenate((np.identity(7), -np.identity(7)), axis=0)
        h = np.concatenate((UB, -LB), axis=0)

        x = solve_qp(self.P.astype(np.double), f.astype(np.double), G.astype(np.double), h.astype(np.double), A.astype(np.double), b.astype(np.double), solver="quadprog")
        print("QP solution: x = {}".format(x))

        # Store as previous values
        self.left_thrust  = x[0] + self.left_thrust
        self.right_thrust = x[1] + self.right_thrust
        self.left_angle   = x[5] + self.left_angle
        self.right_angle  = x[6] + self.right_angle

        curtime = rospy.get_time()
        difftime = curtime - self.lastTime

        #  curtime = rospy.Time.now()
        #  difftime = curtime.nsecs - self.lastTime.nsecs

        #  rospy.loginfo("tau = [%g, %g, %g]", tau[0], tau[1], tau[2])
        rospy.loginfo("[left_thrust, left_angle] = [%g, %g]", self.left_thrust, self.left_angle);
        rospy.loginfo("[right_thrust, right_angle] = [%g, %g]", self.right_thrust, self.right_angle);
        #  rospy.loginfo("difftime = %f", difftime)
        #  now = rospy.Time.now()
        #  rospy.loginfo("time_now = %i %i", now.secs, now.nsecs)

        if (self.left_thrust > 0):
            self.left_command = self.left_thrust/self.max_thrust
        elif (self.left_thrust < 0):
            self.left_command = self.left_thrust/self.min_thrust
        else:
            self.left_command = 0

        if (self.right_thrust > 0):
            self.right_command = self.right_thrust/self.max_thrust
        elif (self.right_thrust < 0):
            self.right_command = self.right_thrust/self.min_thrust
        else:
            self.right_command = 0

        # Create messages and send them
        left_thrust_msg  = Float32()
        left_angle_msg   = Float32()
        right_thrust_msg = Float32()
        right_angle_msg  = Float32()

        left_thrust_msg.data  = self.left_command
        right_thrust_msg.data = self.right_command
        left_angle_msg.data   = -self.left_angle                # the negative sign is to compensate for difference in thruster angle convention from gazebo
        right_angle_msg.data  = -self.right_angle               # the negative sign is to compensate for difference in thruster angle convention from gazebo

        self.left_thrust_pub.publish(left_thrust_msg)
        self.right_thrust_pub.publish(right_thrust_msg)
        self.left_angle_pub.publish(left_angle_msg)
        self.right_angle_pub.publish(right_angle_msg)

        self.lastTime = curtime


if __name__ == '__main__':
    rospy.init_node('qp_allocation', anonymous=True)
    qp_alloc = QP(0, 0, 0, 0, 0, 0)
    rospy.spin()
