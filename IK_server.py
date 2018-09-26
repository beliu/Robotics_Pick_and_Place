#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# These are helper functions for use in building transformation matrices
def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                    [0, cos(q), -sin(q)],
                    [0, sin(q), cos(q)]])
    return R_x

def rot_y(q):    
    R_y = Matrix([[cos(q), 0, sin(q)],
                    [0, 1, 0],
                    [-sin(q), 0, cos(q)]])
    return R_y           

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                    [sin(q), cos(q), 0],
                    [0, 0, 1]])
    return R_z

def t_x(a):
    t = Matrix([[a], [0], [0]])
    return t

def t_z(d):
    t = Matrix([[0], [0], [d]])
    return t

# This function makes a homogenous transform matrix out of a rotational
# and translational matrix
def make_htm(R, t):
    T = R.row_join(t)
    T = T.col_join(Matrix([[0, 0, 0, 1]]))
    
    return T

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        ### Your FK code here
        # Create symbols
    	# The Revolute Joints (theta's)
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')
        q_g = symbols('q_g')
        dtr = pi / 180
    	
        # The Twist Angles (alpha's)
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # The link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        # The link offsets
        d1, d2, d3, d4, d5, d6 = symbols('d1:7')
        d_g = symbols('d_g')

    	# Create Modified DH parameters
    	s = {alpha0:      0, a0:      0, d1:   0.75,
             alpha1 : -pi/2, a1:   0.35, d2:      0, q2: q2 - pi/2,
             alpha2:      0, a2:   1.25, d3:      0, 
             alpha3:  -pi/2, a3: -0.054, d4:   1.50,
             alpha4:   pi/2, a4:      0, d5:      0,
             alpha5:  -pi/2, a5:      0, d6:      0,
             alpha6:      0, a6:      0, d_g: 0.303, q_g: 0}

    	# Define Modified DH Transformation matrix
        # Perform all the transformations from joint 0 to 1
    	T0_1_x = make_htm(rot_x(alpha0), t_x(a0))
        T0_1_z = make_htm(rot_z(q1), t_z(d1))

        T1_2_x = make_htm(rot_x(alpha1), t_x(a1))
        T1_2_z = make_htm(rot_z(q2), t_z(d2))

        T2_3_x = make_htm(rot_x(alpha2), t_x(a2))
        T2_3_z = make_htm(rot_z(q3), t_z(d3))

        T3_4_x = make_htm(rot_x(alpha3), t_x(a3))
        T3_4_z = make_htm(rot_z(q4), t_z(d4))

        T4_5_x = make_htm(rot_x(alpha4), t_x(a4))
        T4_5_z = make_htm(rot_z(q5), t_z(d5))

        T5_6_x = make_htm(rot_x(alpha5), t_x(a5))
        T5_6_z = make_htm(rot_z(q6), t_z(d6))

        T6_g_x = make_htm(rot_x(alpha6), t_x(a6))
        T6_g_z = make_htm(rot_z(q_g), t_z(d_g))

    	# Create individual transformation matrices
    	T0_1 = T0_1_x * T0_1_z
        T0_1 = T0_1.subs(s)

        T1_2 = T1_2_x * T1_2_z
        T1_2 = T1_2.subs(s)

        T2_3 = T2_3_x * T2_3_z
        T2_3 = T2_3.subs(s)

        T3_4 = T3_4_x * T3_4_z
        T3_4 = T3_4.subs(s)

        T4_5 = T4_5_x * T4_5_z
        T4_5 = T4_5.subs(s)

        T5_6 = T5_6_x * T5_6_z
        T5_6 = T5_6.subs(s)

        T6_g = T6_g_x * T6_g_z
        T6_g = T6_g.subs(s)

    	# Extract rotation matrices from the transformation matrices
        T0_3 = T0_1 * T1_2 * T2_3
        R0_3 = T0_3[0:3, 0:3]
    	T0_6 = T0_3 * T3_4 * T4_5 * T5_6
        R0_6 = T0_6[0:3, 0:3]
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # print('End Effector Positions: ', px, py, pz)

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_corr = rot_x(pi) * rot_y(-pi/2)
            R_rpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
            R_rpy_corr =  R_rpy * R_corr
            d_l = 0.303 # The distance from the wrist center to the gripper joint

            # Calculate the wrist position w.r.t. base origin
            wx = px - d_l * R_rpy[0, 2]
            wy = py - d_l * R_rpy[1, 2]
            wz = pz - d_l * R_rpy[2, 2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx)  # Joint 1 angle
            
            C = 1.25    # Distance from J2 to J3
            A = sqrt(0.054**2 + (0.96 + 0.54)**2)   # Distance from J3 to WC
            d_2_wc_x = sqrt(wx**2 + wy**2) - 0.35   # Horizontal distance from J2 to WC
            d_2_wc_z = wz - 0.75    # Vertical distance from J2 to WC
            B = sqrt(d_2_wc_z**2 + d_2_wc_x**2)  # Distance from J2 to WC
            a = acos((B**2 + C**2 - A**2)/(2*B*C))  # Angle between C and B
            beta = atan2(d_2_wc_z, d_2_wc_x)    # Angle the vector from J2 to WC makes with the horizontal
            theta2 = pi/2 - a - beta    # Joint 2 angle

            b = acos((A**2 + C**2 - B**2)/(2*A*C))  # Angle between C and A
            gamma = atan2(0.054, 1.50)  # Angle the vector from J3 to WC makes with the horizontal
            theta3 = pi/2 - b - gamma   # Joint 3 angle

            ## Calculate the last 3 joint angles
            R0_3 = R0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv('LU') * R_rpy

            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0] )
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
