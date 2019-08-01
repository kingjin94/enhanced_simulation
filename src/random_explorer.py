#!/usr/bin/python
# following to begin with: http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Needs apt install ros-melodic-moveit-commander

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import random

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

while True:
	# Produce random goal
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = 0.7 * random.random()
	pose_goal.position.y = 0.7 * random.random()
	pose_goal.position.z = 0.5 * random.random() +0.4

	move_group.set_pose_target(pose_goal)

	# Go to random goal
	plan = move_group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	move_group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	move_group.clear_pose_targets()

	joint_goal = move_group.get_current_joint_values() # Get current config
	# Scan about current config
	for i in np.linspace(-170+10,170-10,num=5):
		for j in np.linspace(-5+10,219-10,num=5):
			joint_goal[4] = i / 180 * pi
			joint_goal[5] = j / 180 * pi
			
			move_group.go(joint_goal, wait=True)
			move_group.stop()
			rospy.sleep(1)
