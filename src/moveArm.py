#!/usr/bin python
# Closes and opens the gripper claws
# Tested
import rospy
rospy.init_node('test')
import actionlib
import math
from control_msgs.msg import FollowJointTrajectoryAction
client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
from control_msgs.msg import FollowJointTrajectoryGoal
goal = FollowJointTrajectoryGoal()
from trajectory_msgs.msg import JointTrajectory
joint_traj = JointTrajectory()
joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

from trajectory_msgs.msg import JointTrajectoryPoint
point1 = JointTrajectoryPoint()
point2 = JointTrajectoryPoint()
point1.positions = [0.33877080806309046, -0.3623406480725775, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795, 0.06015034910227879]
point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
#point2.positions = [0.33877080806309046, -0.3623406480725775, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795 - math.pi/2, 0.06015034910227879]
point2.positions = [0.33877080806309046, -1.5, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795 - math.pi/2, 0.06015034910227879]
point2.velocities = [0., 0., 0., 0., 0., 0., 0.]
point1.time_from_start = rospy.Duration(2.)
point2.time_from_start = rospy.Duration(4.)

from copy import deepcopy
joint_traj.points = [point1, point2, deepcopy(point1), deepcopy(point2)]
joint_traj.points[2].time_from_start += rospy.Duration(4.)
joint_traj.points[3].time_from_start += rospy.Duration(4.)
goal.trajectory = joint_traj
joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
client.send_goal_and_wait(goal)

