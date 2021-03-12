#! /usr/bin/env python

import numpy as np
from math import *
from sensor_msgs.msg import LaserScan

lidar_dist = 1.0
crash_dist = 0.14 # LaserScan.range_min = 0.1199999
close_dist = 0.55

zone_x_dist = 0.3
zone_0_dist = 0.4
zone_1_dist = 0.7

max_ugao = 359
min_ugao = 0
vidokrug_w = 75

# LasecScan msg u array
def lidarScan(msgScan):
    
    udaljenosti = np.array([])
    uglovi = np.array([])

    for i in range(len(msgScan.ranges)):
        ugao = degrees(i * msgScan.angle_increment)
        if ( msgScan.ranges[i] > lidar_dist ):
            udaljenost = lidar_dist
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            udaljenost = msgScan.range_min
        else:
            udaljenost = msgScan.ranges[i]

        udaljenosti = np.append(udaljenosti, udaljenost)
        uglovi = np.append(uglovi, ugao)

    # udaljenosti u m, uglovi u stepenima
    return ( udaljenosti, uglovi )

def checkCrash(lidar):
    lidar_horizont = np.concatenate((lidar[(min_ugao + vidokrug_w):(min_ugao):-1],lidar[(max_ugao):(max_ugao - vidokrug_w):-1]))
    W = np.linspace(1.2, 1, len(lidar_horizont) // 2)
    W = np.append(W, np.linspace(1, 1.2, len(lidar_horizont) // 2))
    if np.min( W * lidar_horizont ) < crash_dist:
        return True
    else:
        return False


def checkObjectNearby(lidar):
    vidokrug_w1 = 50
    lidar_horizont = np.concatenate((lidar[(min_ugao + vidokrug_w1):(min_ugao):-1],lidar[(max_ugao):(max_ugao - vidokrug_w1):-1]))
    W = np.linspace(1.4, 1, len(lidar_horizont) // 2)
    W = np.append(W, np.linspace(1, 1.4, len(lidar_horizont) // 2))
    if np.min( W * lidar_horizont ) < close_dist:
        return True
    else:
        return False


def checkGoalNear(x, y, x_goal, y_goal):
    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if ro < 0.1:
        return True
    else:
        return False

def checkGoalNear1(x, y, x_goal, y_goal):
    if x > 1.5:
        return True
    else:
        return False



def scanDiscretization(lidar):
    x1 = 3 # leva zona
    x2 = 3 # desna zona
    x3 = 3 # levi sektor
    x4 = 3 # desni sektor

    # Find the left side lidar values of the vehicle
    lidar_left = min(lidar[(min_ugao):(min_ugao + vidokrug_w)])
    if zone_1_dist > lidar_left > zone_0_dist:
        x1 = 2 # zone 1
    elif zone_0_dist > lidar_left > zone_x_dist:
	    x1 = 1
    elif lidar_left <= zone_x_dist:
        x1 = 0 # zone 0

    # Find the right side lidar values of the vehicle
    lidar_right = min(lidar[(max_ugao - vidokrug_w):(max_ugao)])
    if zone_1_dist > lidar_right > zone_0_dist:
        x2 = 2 # zone 1
    elif zone_0_dist > lidar_right > zone_x_dist:
	    x2 = 1
    elif lidar_right <= zone_x_dist:
        x2 = 0 # zone 0

    # Detection of object in front of the robot
    if ( min(lidar[(max_ugao - vidokrug_w // 3):(max_ugao)]) < 1.0 ) or ( min(lidar[(min_ugao):(min_ugao + vidokrug_w // 3)]) < 1.0 ):
        object_front = True
    else:
        object_front = False

    # Detection of object on the left side of the robot
    if min(lidar[(min_ugao):(min_ugao + 2 * vidokrug_w // 3)]) < 1.0:
        object_left = True
    else:
        object_left = False

    # Detection of object on the right side of the robot
    if min(lidar[(max_ugao - 2 * vidokrug_w // 3):(max_ugao)]) < 1.0:
        object_right = True
    else:
        object_right = False

    # Detection of object on the far left side of the robot
    if min(lidar[(min_ugao + vidokrug_w // 3):(min_ugao + vidokrug_w)]) < 1.0:
        object_far_left = True
    else:
        object_far_left = False

    # Detection of object on the far right side of the robot
    if min(lidar[(max_ugao - vidokrug_w):(max_ugao - vidokrug_w // 3)]) < 1.0:
        object_far_right = True
    else:
        object_far_right = False

    # The left sector of the vehicle
    if ( object_front and object_left ) and ( not object_far_left ):
        x3 = 0 # sector 0
    elif ( object_left and object_far_left ) and ( not object_front ):
        x3 = 1 # sector 1
    elif object_front and object_left and object_far_left:
        x3 = 2 # sector 2

    if ( object_front and object_right ) and ( not object_far_right ):
        x4 = 0 # sector 0
    elif ( object_right and object_far_right ) and ( not object_front ):
        x4 = 1 # sector 1
    elif object_front and object_right and object_far_right:
        x4 = 2 # sector 2

    # Find the state space index of (x1,x2,x3,x4) in Q table
    return ( x1, x2, x3 , x4 )




