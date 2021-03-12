#! /usr/bin/env python

import rospy
from time import time
from time import sleep
from datetime import datetime

import collections

from Lidar import *
from Control import *
from NN import *


X_0 = -2.0
Y_0 = -0.5
THETA_0 = 0

X_GOAL = 2.5
Y_GOAL = 0.5
THETA_GOAL = 0


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)

        setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        f1 = open("/home/student/catkin_ws/src/projekat/src/scripts/w1.txt", "r")
        f2 = open("/home/student/catkin_ws/src/projekat/src/scripts/w2.txt", "r")
        f3 = open("/home/student/catkin_ws/src/projekat/src/scripts/w3.txt", "r")
        w1 = np.loadtxt(f1)
        w2 = np.loadtxt(f2)
        w3 = np.loadtxt(f3)

        nn = NeuralNetwork()
        nn.synaptic_weights1 = w1
        nn.synaptic_weights2 = w2
        nn.synaptic_weights3 = w3

       	print(nn.synaptic_weights1)
       	print(nn.synaptic_weights2)
       	print(nn.synaptic_weights3)

        # robot in initial position
        robot_in_pos = False

       	MEM_LENGTH = 40
       	qx = collections.deque(maxlen = MEM_LENGTH)
       	qy = collections.deque(maxlen = MEM_LENGTH)      
       	for i in range(MEM_LENGTH):
       		qx.append(100)
       		qy.append(100)
       	i = 0
       	# main loop
        while not rospy.is_shutdown():
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            odomMsg = rospy.wait_for_message('/odom', Odometry)

            if not robot_in_pos:
                robotStop(velPub)
                # init pos
                ( x_init , y_init , theta_init ) = robotSetPos(setPosPub, X_0, Y_0, THETA_0)
                odomMsg = rospy.wait_for_message('/odom', Odometry)
                ( x , y ) = getPosition(odomMsg)
                theta = degrees(getRotation(odomMsg))
                robot_in_pos = True
            else:
                # Get robot position and orientation
                ( x , y ) = getPosition(odomMsg)
                theta = getRotation(odomMsg)

      		qx.append(x)
      		qy.append(y)
      
      		x0 = qx[0]
      		y0 = qy[0]

    		loop = False

		distance_ = sqrt( pow( ( x0 - x ) , 2 ) + pow( ( y0 - y ) , 2) )
		if distance_ < 0.5:
			loop = True
		

                # Get lidar scan
                ( lidar, angles ) = lidarScan(msgScan)
                ( x1, x2 ,x3 ,x4 ) = scanDiscretization(lidar)

                # Check for objects nearby
                crash = checkCrash(lidar)
		object_nearby = checkObjectNearby(lidar)
                goal_near = checkGoalNear(x, y, X_GOAL, Y_GOAL)
       	       
                # Stop the simulation
                if crash:
	       	    print(x, y)
                    robotStop(velPub)
                    rospy.signal_shutdown('End of testing!')
                    print("CRASH!!!!!!!!!!!!!!!!")
		elif not loop and ( not object_nearby or goal_near ):
		     print("Feedback")                    
		     robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, THETA_GOAL)
                else:
	 	    print("Neuralna")
		    action = np.argmax(nn.forward_pass(np.array([x1, x2, x3, x4])))
                    robotDoAction(velPub, action)


    except rospy.ROSInterruptException:
        robotStop(velPub)
        print 'Simulation terminated!'
        pass

