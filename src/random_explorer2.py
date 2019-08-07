import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import random_quaternion, euler_from_quaternion, quaternion_from_euler
import numpy as np
import random

MIN_DIST_LOOKOUTS = 0.4

rospy.init_node('random_explorer', anonymous=True)

def distance_pos_list(prev_positions, pos):
	dist = []
	for p in prev_positions:
		dist.append(sqrt((p.x- pos.x)**2 + (p.y- pos.y)**2 + (p.z- pos.z)**2))
	if len(dist) == 0:
		return [np.inf]
	else:
		return dist
		

# Look around randomly
def random_walk(move_group):
	scanned_positions = []
	while True:
		move_group.set_max_velocity_scaling_factor(0.5) # Allow 10 % of the set maximum joint velocities
		move_group.set_max_acceleration_scaling_factor(0.5)
		
		if min(distance_pos_list(scanned_positions, move_group.get_current_pose().pose.position)) > MIN_DIST_LOOKOUTS:
			print("Looking around")
			joint_goal = np.asarray(robot.get_current_state().joint_state.position[:7])
			for c, j4 in enumerate(np.linspace(-170+10,170-10,num=7)):
				for d, j5 in enumerate((c%2)*np.linspace(-5+15,219-10,num=7) + ((c+1)%2)*np.linspace(219-10,-5+10,num=7)):
					joint_goal[4] = j4 / 180 * pi
					joint_goal[5] = j5 / 180 * pi
					joint_goal[6] = pi/4
		
					move_group.go(joint_goal, wait=True)
					move_group.stop()
					rospy.sleep(1)
			scanned_positions.append(move_group.get_current_pose().pose.position)
		else:
			print("Was here before")
				
		move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
		move_group.set_max_acceleration_scaling_factor(0.1)

		
		print("Moving to new position")
		# Find new goal
		while True:
			pose_goal = geometry_msgs.msg.Pose()
			while True: # do ... while
				pose_goal.position.x = 1.4 * (random.random()-0.5) # -0.7 till 0.7 m
				pose_goal.position.y = 1.4 * (random.random()-0.5) # -0.7 till 0.7 m
				pose_goal.position.z = 0.5 * random.random() +0.4 # 0.4 till 0.9 m
				if min(distance_pos_list(scanned_positions, pose_goal.position)) > MIN_DIST_LOOKOUTS:
					break
					
			ori = random_quaternion()
			pose_goal.orientation.x = ori[0]
			pose_goal.orientation.y = ori[1]
			pose_goal.orientation.z = ori[2]
			pose_goal.orientation.w = ori[3]
			(plan, fraction) = move_group.compute_cartesian_path([pose_goal], 0.01, 0.0)
			exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
			print("Path length: {}; Exec time: {} s".format(len(plan.joint_trajectory.points), exec_time))
			if plan:
				print("Valid plan found")
				if not (0.1 < exec_time < 4.0) \
					or not (1 < len(plan.joint_trajectory.points) < 100):
					print("To long")
					continue
				print("executing ...")
				try:
					ret = move_group.execute(plan, wait=False) # Move to goal
					rospy.sleep(exec_time+1) # Wait till arrived		
					move_group.stop()	
					break	
				except KeyboardInterrupt:
					return
				except Exception as e:
					print(e)
			
		else:
			print("No plan found; new goal ...")
				
		# move_group.set_pose_target(pose_goal)
		# try:
			# move_group.plan()
			# print("Going")
			# move_group.go(wait=True)
		# except Exception as e:
			# print("No valid path found")
			# print(e)

# For initial move
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Move to better starting position
print("Moving to start position")
client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
goal = FollowJointTrajectoryGoal()
joint_traj = JointTrajectory()
joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
point1 = JointTrajectoryPoint()
#point1.positions = [0.33877080806309046, -0.3623406480725775, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795, 0.06015034910227879]
point1.positions = [0.33877080806309046, -1.5, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795 - 1.51, 0.06015034910227879]
point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
point1.time_from_start = rospy.Duration(2.)
joint_traj.points = [point1]
goal.trajectory = joint_traj
joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
client.send_goal_and_wait(goal)

# Start Motion Planner

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
move_group.set_max_acceleration_scaling_factor(0.1)


try:
	print("Start exploring")
	random_walk(move_group)
except KeyboardInterrupt:
	rospy.loginfo("random_explorer shutting down")


