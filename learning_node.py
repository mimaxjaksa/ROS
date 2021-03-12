#! /usr/bin/env python

import rospy
import math
from datetime import datetime
import time

from Lidar import *
from Control import *

from NN import *

#nase
gen = 0
n_gen = 30
pop_num = 40
MAX_STEPS_PER_GEN = 500 #400


# Episode parameters
MAX_EPISODES = 600
MAX_STEPS_PER_EPISODE = 400
MIN_TIME_BETWEEN_ACTIONS = 0.0


# Initial position
X_0 = -2.5 #-0.4
Y_INIT = -0.5 #-0.4
THETA_INIT = 0

X_GOAL = 1.5 #0.5
Y_GOAL = -3.5 #-0.5

RANDOM_INIT_POS = False



if __name__ == '__main__':
    try:
        rospy.init_node('learning_node', anonymous = False)
        rate = rospy.Rate(10)

        setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

	#inicijalizacija populacije
	population = createPopulation(pop_num)
	curr_nn_index = 0
	step_num = 0
	fitness = np.zeros(pop_num)
	goal = False
	crash = False 
	robot_in_pos = False
	while(not robot_in_pos):
		robotStop(velPub)
		rospy.sleep(2)

      		( x_init , y_init , theta_init ) = robotSetPos(setPosPub, X_0, Y_0, THETA_0)
		odomMsg = rospy.wait_for_message('/odom', Odometry)
                ( x , y ) = getPosition(odomMsg)
    	        theta = degrees(getRotation(odomMsg))
		if abs(x-X_0) < 0.01 and abs(y-Y_0) < 0.01 and abs(theta-THETA_0) < 0.8:
        		robot_in_pos = True
        	else:
        		robot_in_pos = False
	
        # main loop
        while not rospy.is_shutdown():
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            ( x , y ) = getPosition(odomMsg)
            theta = degrees(getRotation(odomMsg))

	    if not curr_nn_index == pop_num:
            	curr_nn = population[curr_nn_index]

            if gen < n_gen:
               	if curr_nn_index == pop_num:
            		#kraj jedne generacije
			print("Kraj gen: ", gen)
                	gen += 1
                	curr_nn_index = 0   
			pop1 = population
			pop_sorted = [x for _,x in sorted(zip(fitness, pop1))] 
           	     	best_nn = pop_sorted[-1]
			print(best_nn.synaptic_weights1)
			print(best_nn.synaptic_weights2)
			print(best_nn.synaptic_weights3)             		
			population = population_change(population, fitness)	                
			fitness = np.zeros(pop_num)
                	#mutacije i krosoveri
                	#reset fitnessa
                else:
	               	if (step_num < MAX_STEPS_PER_GEN) and not (crash or goal):
	               		step_num += 1
#				print("step_num:", step_num)
                		( lidar, angles ) = lidarScan(msgScan)
                        	( x1, x2 ,x3 ,x4 ) = scanDiscretization(lidar)
                        	crash = checkCrash(lidar)
                        	goal = checkGoalNear(x, y, X_GOAL, Y_GOAL)
				action = curr_nn.forward_pass(np.array([x1, x2, x3, x4]))
	                        action = np.argmax(action)
	                        robotDoAction(velPub, action)
                	else:
                		#kraj voznje jedinke
                		distance = (np.sqrt((x - X_GOAL)**2 + (y - Y_GOAL)**2))
				distance = distance**5
                		if crash:
                			curr_fitness = -100 - distance
                		elif goal: 
                			curr_fitness = 1000 - step_num
                		else:
                			curr_fitness = -distance        			
				print(curr_nn_index, curr_fitness)
                		fitness[curr_nn_index] = curr_fitness

                		curr_nn_index += 1
				robot_in_pos = False
				while(not robot_in_pos):
					robotStop(velPub)					
					rospy.sleep(0.5)
                			( x_init , y_init , theta_init ) = robotSetPos(setPosPub, X_0, Y_0, THETA_0)
					odomMsg = rospy.wait_for_message('/odom', Odometry)
                        		( x , y ) = getPosition(odomMsg)
	                        	theta = degrees(getRotation(odomMsg))
					if abs(x-X_0) < 0.01 and abs(y-Y_0) < 0.01 and abs(theta-THETA_0) < 0.1:
        	                        	robot_in_pos = True
        	                    	else:
        	                        	robot_in_pos = False
				crash = False
				goal = False
	                	step_num = 0
            else:
		print(fitness)
               	save1(best_nn)
                	       
    except rospy.ROSInterruptException:
        robotStop(velPub)
        print 'Simulation terminated!'
        pass
