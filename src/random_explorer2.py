import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sqrt
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
from tf.transformations import random_quaternion, euler_from_quaternion, quaternion_from_euler
import numpy as np
import random
from filter_octomap.msg import table
import touchTable2

# For initial move
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MIN_DIST_LOOKOUTS = 0.4

rospy.init_node('random_explorer', anonymous=True)

def force_home():
	# Move to better starting position
	print("Moving to start position")
	client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	client.wait_for_server()
	goal = FollowJointTrajectoryGoal()
	joint_traj = JointTrajectory()
	joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
	point1 = JointTrajectoryPoint()
	# point1.positions = [0.33877080806309046, -1.5, -1.2750252749223279, -1.8702679826187074, 2.4926642728445163, 2.873639813867795 - 1.51, 0.06015034910227879]
	point1.positions = [0.0, -1.5, 0.0, -1.8702679826187074, 0, 2.873639813867795 - 1.51, 0.06015034910227879]
	point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
	point1.time_from_start = rospy.Duration(2.)
	joint_traj.points = [point1]
	goal.trajectory = joint_traj
	joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
	client.send_goal_and_wait(goal)

def distance_pos_list(prev_positions, pos):
	dist = []
	for p in prev_positions:
		dist.append(sqrt((p.x- pos.x)**2 + (p.y- pos.y)**2 + (p.z- pos.z)**2))
	if len(dist) == 0:
		return [np.inf]
	else:
		return dist
		
def look_around(move_group, robot, scanned_positions):
	move_group.stop()
	move_group.set_max_velocity_scaling_factor(0.5) # Allow 50 % of the set maximum joint velocities, are moving only short distances
	move_group.set_max_acceleration_scaling_factor(0.5)
	failures = 0
	print("Looking around")
	joint_goal = np.asarray(robot.get_current_state().joint_state.position[:7])
	for c, j4 in enumerate(np.linspace(-170+10,170-10,num=7)):
		for d, j5 in enumerate((c%2)*np.linspace(-5+15,219-10,num=7) + ((c+1)%2)*np.linspace(219-10,-5+10,num=7)):
			joint_goal[4] = j4 / 180 * pi
			joint_goal[5] = j5 / 180 * pi
			joint_goal[6] = pi/4
			
			try: 
				move_group.go(joint_goal, wait=True)
			except Exception as e:
				failures += 1
				print("Impossible orientation")
				print(e)
			move_group.stop()
			rospy.sleep(1.)
			
	if failures < 10:
		scanned_positions.append(move_group.get_current_pose().pose.position)
	move_group.stop()
	rospy.sleep(1.)
	
	move_group.set_max_velocity_scaling_factor(0.2) # Allow 10 % of the set maximum joint velocities
	move_group.set_max_acceleration_scaling_factor(0.1)
	
def get_new_random_direction():
	position = move_group.get_current_pose().pose.position
	# move_direction = np.random.uniform(low=-1., high=1., size=(3))
	move_direction = np.zeros((3))
	# Sample such that tendency to move to center
	move_direction[0] = np.random.uniform(low=-0.5*(position.x+1.), high=.5*abs(position.x-1.), size=(1))
	move_direction[1] = np.random.uniform(low=-0.5*(position.y+1.), high=.5*abs(position.y-1.), size=(1))
	move_direction[2] = np.random.uniform(low=-0.5*(position.z+1.), high=.5*abs(position.z-1.), size=(1))
	
	move_direction = move_direction / np.linalg.norm(move_direction)
	return 0.3 * move_direction

# Look around randomly
def random_walk(move_group, robot):
	scanned_positions = []
	move_direction = get_new_random_direction()
	pose_goal = geometry_msgs.msg.Pose()
	
	while True:		
		if min(distance_pos_list(scanned_positions, move_group.get_current_pose().pose.position)) > MIN_DIST_LOOKOUTS:
			look_around(move_group, robot, scanned_positions)
			#print("Skipped looking around")
		else:
			print("Was here before")
		
		print("Moving to new position")
		move_group.set_max_velocity_scaling_factor(0.2) # Allow 10 % of the set maximum joint velocities
		move_group.set_max_acceleration_scaling_factor(0.1)
		# Find new positions and go to them
		num_replan = 0
		while True:
			move_group.stop()	
			if num_replan > 100:
				print("Still nothing found .. forcing home")
				force_home()
				num_replan = 0
				move_direction = get_new_random_direction()
				
			move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
			move_group.set_max_acceleration_scaling_factor(0.05)
			
			pose_goal.position = move_group.get_current_pose().pose.position
			print("I'm here:")
			print(pose_goal.position)
			print("Move towards:")
			print(move_direction)
			pose_goal.position.x = move_direction[0] + pose_goal.position.x
			pose_goal.position.y = move_direction[1] + pose_goal.position.y
			pose_goal.position.z = move_direction[2] + pose_goal.position.z
					
			ori = random_quaternion()
			pose_goal.orientation.x = ori[0]
			pose_goal.orientation.y = ori[1]
			pose_goal.orientation.z = ori[2]
			pose_goal.orientation.w = ori[3]
			print("New goal:")
			print(pose_goal.position)
			if pose_goal.position.x**2+pose_goal.position.y**2 + pose_goal.position.z**2 > 1.:
				move_direction = get_new_random_direction()
				print("Goal to far out; new direction")
				continue
				
			tmp_tolerance = move_group.get_goal_orientation_tolerance()
			move_group.set_goal_orientation_tolerance(20.) # Eef orientation not important, look around anyway
			(plan, fraction) = move_group.compute_cartesian_path([pose_goal], 0.01, 1.0)
			move_group.set_goal_orientation_tolerance(tmp_tolerance)
			exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
			print("Path length: {}; Exec time: {} s".format(len(plan.joint_trajectory.points), exec_time))
			if plan:
				print("Valid plan found")
				if fraction < 0.5 or not (.1 < exec_time < 10.0) \
					or not (1 < len(plan.joint_trajectory.points) < 100):
					print("To long / short")
					num_replan += 1
					continue
				print("executing ...")
				num_replan = 0
				try:
					ret = move_group.execute(plan, wait=False) # Move to goal
					rospy.sleep(exec_time+1) # Wait till arrived		
					move_group.stop()	
					rospy.sleep(1.)
					break	
				except KeyboardInterrupt:
					return
				except Exception as e:
					print(e)
			else:
				move_direction = get_new_random_direction()
				num_replan += 1
				print("No plan found; new goal ...")			
				
		# evaluate if done
		try:
			mapEntropyMsg = rospy.wait_for_message("/octomap_new/entropy", Float64,  timeout=2.)
			tableMsg = rospy.wait_for_message("/octomap_new/table", table,  timeout=2.)
			
			if mapEntropyMsg.data < -0.5 and tableMsg.score > 900000:
				print("Table found and map good enough")
				print("Moving on to touching the table")
				return True
		except rospy.exceptions.ROSException:
			print("No feedback on progress received")
		
		

force_home()

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
	random_walk(move_group, robot)
	refiner = touchTable2.tableRefiner()
	refiner.touch_and_refine_table()
except KeyboardInterrupt:
	rospy.loginfo("random_explorer shutting down")


