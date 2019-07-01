#!/usr/bin python
# Closes and opens the gripper claws
# May have to kill movegroup before execution or link to same version
# Tested
import rospy
rospy.init_node('test')
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
client = actionlib.SimpleActionClient('/panda_hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
from control_msgs.msg import FollowJointTrajectoryGoal
goal = FollowJointTrajectoryGoal()
from trajectory_msgs.msg import JointTrajectory
joint_traj = JointTrajectory()
joint_traj.joint_names = ["panda_finger_joint1","panda_finger_joint2"]

from trajectory_msgs.msg import JointTrajectoryPoint
point1 = JointTrajectoryPoint()
point2 = JointTrajectoryPoint()
point1.positions = [0., 0.]
point1.velocities = [0., 0.]
point2.positions = [0.02, 0.02]
point2.velocities = [0., 0.]
point1.time_from_start = rospy.Duration(2.)
point2.time_from_start = rospy.Duration(4.)

joint_traj.points = [point1, point2]
goal.trajectory = joint_traj
joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
client.send_goal_and_wait(goal)

