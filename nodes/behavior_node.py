# -*- coding: utf-8 -*-
"""
Created on Sun Nov 24 13:44:04 2019

@author: Marc Friis Torkelund
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from math import pi
from math import cos
from math import sin
from math import atan2
from math import sqrt

def goToBlock(xyz,startPosition):
    path = [
            [startPosition[0],startPosition[1],startPosition[2],startPosition[3],startPosition[4]],
            [startPosition[0],startPosition[1],15,startPosition[3],startPosition[4]],
            [xyz[0],xyz[1],15,xyz[3],-pi/2],
            [xyz[0],xyz[1],xyz[2],xyz[3],-pi/2],
            [xyz[0],xyz[1],xyz[2],xyz[3],xyz[4]]
            ]
    return path

def moveBlock(xyz,startPosition):
    path = [
            [startPosition[0],startPosition[1],startPosition[2],startPosition[3],startPosition[4]],
            [startPosition[0],startPosition[1],15,startPosition[3],startPosition[4]],
            [xyz[0],xyz[1],15, xyz[3],startPosition[4]],
            [xyz[0],xyz[1],15,xyz[3],startPosition[4]],
            [xyz[0],xyz[1],xyz[2],xyz[3],-pi/2]
            ]
    return path

def moveArm(xyz,startPosition):
    path = [
            [startPosition[0],startPosition[1],startPosition[2],startPosition[3],startPosition[4]],
            [startPosition[0],startPosition[1],15,startPosition[3],startPosition[4]],
            [xyz[0],xyz[1],xyz[2],xyz[3],xyz[4]]
            ]
    return path

def resetArm(startPosition):
    path = [
            [startPosition[0],startPosition[1],startPosition[2],startPosition[3],startPosition[4]],
            [startPosition[0],startPosition[1],15,startPosition[3],startPosition[4]],
            [0,0,55.5,0,0]
            ]
    return path
