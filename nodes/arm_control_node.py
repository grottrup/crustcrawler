#!/usr/bin/env python


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from math import pi
from math import cos
from math import sin
from math import atan2
from math import sqrt

def invkin(xyzrg):
	d1 = 13.0; # cm (height of 2nd joint)
	a1 = 0.0; # (distance along "y-axis" to 2nd joint)
	a2 = 17.0; # (distance between 2nd and 3rd joints)
	d4 = 25.5; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	xc = xyzrg[0]
	yc = xyzrg[1]
	zc = xyzrg[2]
	rot = xyzrg[3]
	grip = xyzrg[4]

	q1 = atan2(yc, xc)

	r2 = (xc - a1*cos(q1))**2 + (yc - a1*sin(q1))**2
	s = (zc - d1)
	D = (r2 + s**2 - a2**2 - d4**2)/(2*a2*d4)

	q3 = atan2(-sqrt(1-D**2), D) + 0
	q2 = atan2(s, sqrt(r2)) - atan2(d4*sin(q3), a2 + d4*cos(q3)) - pi/2

	q4 = rot
	q5 = grip

	return q1,q2,q3,q4,q5

class ActionExampleNode:

	N_JOINTS = 5
	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

		self.joint_positions = []
		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4",
                "gripper"
				]
		# the list of xyz points we want to plan
		xyzrg_positions = [
        # [25, -20, 5, 0, 0],
        # [25, -15, 5, 0, 0],
        # [25, -10, 5, 0, 0],
        # [25, -5, 5, 0, 0],
        # [25, 0, 5, 0, 0],
        # [25, 5, 5, 0, 0],
        # [25, 10, 5, 0, 0],
        # [25, 15, 5, 0, 0],
        # [25, 20, 5, 0, 0]

        # [20, -20, 5, 0, 0],
        # [20, 20, 5, 0, 0]


        # [30, 0, 30, 0, 0],
        # [30, 0, 25, 0, 0],
        # [30, 0, 20, 0, 0],
        # [30, 0, 15, 0, 0],
        # [30, 0, 10, 0, 0],
        # [30, 0, 5, 0, 0],
        # [30, 0, 0, 0, 0]

        # [30, 0, 30, 0, -pi/2],
        [30, 0, 15, 0, -pi/2],
        [30, 0, 0, 0, -pi/2],
        [30, 0, 0, 0, pi/4],
        [30, 0, 15, 0, pi/4],
        # [30, 0, 0, 0, pi/4],
        # [30, 0, 0, 0, -pi/2],
        [30, -13, 15, 0, pi/4],
        [30, -13, 5, 0, pi/4],
        [30, -13, 5, 0, -pi/2],
        [30, -13, 15, 0, -pi/2],
        # [0, 0, 55, 0, 0]
		]
		# initial duration
		dur = rospy.Duration(2)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in xyzrg_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def send_command(self):
		self.client.wait_for_server()
		print(self.goal)
		self.client.send_goal(self.goal)

		self.client.wait_for_result()
		print(self.client.get_result())

if __name__ == "__main__":
 	rospy.init_node("arm_control")

 	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")

 	node.send_command()
