#!/usr/bin/env python

import rospy
from time import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# NN-learning speed parameters
CONST_V_FORWARD = 0.1 #0.08
CONST_W_FORWARD = 0.0
CONST_V_TURN = 0.08 #0.06
CONST_W_TURN = 0.4

k_ro = 2
k_alfa = 15
k_beta = -3
v_const = 0.1 # [m/s]



# Get theta in [radians]
def getRotation(odomMsg):
    orientation_q = odomMsg.pose.pose.orientation
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

# Get (x,y) coordinates in [m]
def getPosition(odomMsg):
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    return ( x , y)

# Create rosmsg Twist()
def createVelMsg(v,w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

# Feedback Control Algorithm
def robotFeedbackControl(velPub, x, y, theta, x_goal, y_goal, theta_goal):
    # theta goal normalization
    if theta_goal >= pi:
        theta_goal_norm = theta_goal - 2*pi
    else:
        theta_goal_norm = theta_goal

    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    lamda = atan2( y_goal - y , x_goal - x )

    alpha = (lamda -  theta + pi) % (2*pi) - pi
    beta = (theta_goal - lamda + pi) % (2*pi) - pi

    if ro < 0.05:
        v = 0
        w = 0
        v_scal = 0
        w_scal = 0
        robotStop(velPub)
        rospy.signal_shutdown('End of testing!')
    else:
        v = k_ro * ro
        w = k_alfa * alpha + k_beta * beta
        v_scal = v / abs(v) * v_const
        w_scal = w / abs(v) * v_const

    velMsg = createVelMsg(v_scal, w_scal)
    velPub.publish(velMsg)

# Go forward command
def robotGoForward(velPub):
    velMsg = createVelMsg(CONST_V_FORWARD,CONST_W_FORWARD)
    velPub.publish(velMsg)

# Turn left command
def robotTurnLeft(velPub):
    velMsg = createVelMsg(CONST_V_TURN,+CONST_W_TURN)
    velPub.publish(velMsg)

# Turn right command
def robotTurnRight(velPub):
    velMsg = createVelMsg(CONST_V_TURN,-CONST_W_TURN)
    velPub.publish(velMsg)

# Stop command
def robotStop(velPub):
    velMsg = createVelMsg(0.0,0.0)
    velPub.publish(velMsg)

# Set robot position and orientation
def robotSetPos(setPosPub, x, y, theta):
    checkpoint = ModelState()

    checkpoint.model_name = 'turtlebot3_burger'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,radians(theta))
    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return ( x , y , theta )

# Perform an action
def robotDoAction(velPub, action):
    if action == 0:
        robotGoForward(velPub)
    elif action == 1:
        robotTurnLeft(velPub)
    elif action == 2:
        robotTurnRight(velPub)
    else:
        robotGoForward(velPub)

