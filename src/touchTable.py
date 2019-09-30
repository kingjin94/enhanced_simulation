from filter_octomap.msg import table
import numpy as np
import random
import rospy
import moveit_commander
import geometry_msgs.msg
import sys
from copy import deepcopy

# For sending joint level controlls
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# For constraining planning
from moveit_msgs.msg import Constraints, JointConstraint

# For kinematic calculations
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import tf

# For bumper
from enhanced_sim.msg import CollisionState
from gazebo_msgs.msg import ContactsState

def go_to_q(q):
	client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	print("Waiting for action server ...")
	client.wait_for_server()
	goal = FollowJointTrajectoryGoal()
	joint_traj = JointTrajectory()
	joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
	point1 = JointTrajectoryPoint()
	if type(q)==np.ndarray:
		point1.positions = q.tolist()
	else:
		point1.positions = q
	point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
	point1.time_from_start = rospy.Duration(1.)
	joint_traj.points = [point1]
	goal.trajectory = joint_traj
	joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
	client.send_goal_and_wait(goal)
	
def go_home():
	go_to_q(np.asarray([0.0, -1.5, 0.0, -1.8702679826187074, 0, 2.873639813867795 - 1.51, 0.06015034910227879])) # go home configuration
	
def transl_distance_pose(pose_a, pose_b):
	position_a = np.asarray((pose_a.position.x, pose_a.position.y, pose_a.position.z))
	position_b = np.asarray((pose_b.position.x, pose_b.position.y, pose_b.position.z))
	
	return np.linalg.norm(position_a - position_b)
	
def plan_and_go(move_group, pose_goal, substeps=10):
	arrived = False
	for i in range(substeps):
		(plan, fraction) = move_group.compute_cartesian_path([pose_goal], 0.01, 1.0)
		exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
		if plan:
			if fraction < 0.5:
				print("Sub plan not complete ({})".format(fraction))
				continue
				
			if not (.1 < exec_time < 10.0):
				print("Sub p exec time to long / short ({})".format(exec_time))
				continue
				
			if not (1 < len(plan.joint_trajectory.points) < 100):
				print("Sub p with to many steps ({})".format(len(plan.joint_trajectory.points)))
				continue
				
			print("executing ...")
			num_replan=0
			try:
				plan.joint_trajectory.points.append(deepcopy(plan.joint_trajectory.points[-1]))
				plan.joint_trajectory.points[-1].velocities = [0, 0, 0, 0, 0, 0, 0]
				plan.joint_trajectory.points[-1].accelerations = [0, 0, 0, 0, 0, 0, 0]
				plan.joint_trajectory.points[-1].time_from_start += rospy.rostime.Duration(0.5)
				print(plan)
				ret = move_group.execute(plan, wait=False) # Move to goal
				rospy.sleep(exec_time+0.7) # Wait till arrived	
				#go_to_q(plan.joint_trajectory.points[-1].positions)
				move_group.stop()	
				rospy.sleep(1.)
				#TODO : Check if at intended position
				c_pose = move_group.get_current_pose().pose
				if transl_distance_pose(c_pose, pose_goal) < 0.1 and c_pose.orientation.x > 0.95:
					return True
				else:
					print("Not yet there")
					continue	
			except KeyboardInterrupt:
				return False
			except Exception as e:
				print(e)
				return False
	return False

def go_to_random_point_over_table(move_group, table_msg, max_reach=0.8):
	go_home()
	arrived = False
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.position.z = table_msg.centroid_position.z + 0.3
	pose_goal.orientation.x = 1 # tool pointing down
	pose_goal.orientation.y = 0
	pose_goal.orientation.z = 0
	pose_goal.orientation.w = 0

	# select random point above table and go there
	# if not successfull try another point
	num_replan = 0
	while not arrived and not rospy.is_shutdown():
		if num_replan > 100:
			num_replan = 0
			go_home()
			
		pose_goal.position.x = random.uniform(table_msg.min.x+.1, table_msg.max.x-.1)
		pose_goal.position.y = random.uniform(table_msg.min.y+.1, table_msg.max.y-.1)
		
		if pose_goal.position.x**2+pose_goal.position.y**2 > max_reach**2:
			print("Goal probably to far")
			continue
		
		print("Trying: {}", pose_goal.position)
		arrived = plan_and_go(move_group, pose_goal)
		if not arrived:
			num_replan += 1
	
	print("Arrived successfully")
	return pose_goal

def touch_table_straight_on(move_group, robot, kdl_kin, pose_goal):
	# move towards the table in a straight line (along end-effector's z-axis)
	rot_d = np.asarray((1,0,0,0))
	pos_d = np.asarray((pose_goal.position.x,pose_goal.position.y,pose_goal.position.z))
	old_e = np.zeros(6)
	sum_e = np.zeros(6)
	touchSensorMsg = None
	while True and not rospy.is_shutdown():  # while not touched later!!!
		pose_c = move_group.get_current_pose()
		pos_c = np.asarray((pose_c.pose.position.x, pose_c.pose.position.y, pose_c.pose.position.z))
		rot_c = np.asarray((pose_c.pose.orientation.x, pose_c.pose.orientation.y, pose_c.pose.orientation.z, pose_c.pose.orientation.w))
		if rot_c[0] < 0:
			rot_c = -1.*rot_c
		
		pos_d[2] -= 0.005
		
		print("Desired:")
		print(np.concatenate([pos_d, rot_d]))
		print("Actual:")
		print(np.concatenate([pos_c, rot_c]))
		
		pos_e = pos_d - pos_c
		rot_e = rot_d - rot_c
		rot_e = np.asarray(tf.transformations.euler_from_quaternion(rot_e)) / 1000 #(get into same ballpark pi of rot to 0.001 m of the position)
		e = np.concatenate([pos_e, rot_e])
		de = e-old_e
		old_e = e
		sum_e += e
		
		print("Error:")
		print(e)
		print("dError:")
		print(de)
		print("dsumError:")
		print(sum_e)
		
		q = np.asarray(robot.get_current_state().joint_state.position[:7])
		J = np.asarray(kdl_kin.jacobian(q))

		q_new = q+J.T.dot(e+0.1*de+0.01*sum_e) #[0,0,-1,0,0,0]
		go_to_q(q_new)
		move_group.stop()
		
		touchSensorMsg = rospy.wait_for_message("/panda/bumper/colliding", CollisionState)
		if touchSensorMsg.colliding:
			#print(touchSensorMsg)
			move_group.stop()
			break
			
	return touchSensorMsg

def retract_from_table_straight(move_group, robot, kdl_kin):
	q = np.asarray(robot.get_current_state().joint_state.position[:7])
	J = np.asarray(kdl_kin.jacobian(q))

	q_new = q+J.T.dot([0,0,0.05,0,0,0])
	go_to_q(q_new)
	#move_group.shift_pose_target(2, +0.05)
	ret = move_group.go()
	move_group.stop()
	print("Retracted")

def fit_table(touchPoses):
	# fit table surface with new information from the touched points and display info
	#print("Touched surface at the following poses:")
	#print(touchPoses)
	print("Improved surface estimate:")
	touch_pts = np.zeros((3,10))
	touch_pts[0,:] = [touchPoses[i].states[0].contact_positions[0].x for i in range(10)]
	touch_pts[1,:] = [touchPoses[i].states[0].contact_positions[0].y for i in range(10)]
	touch_pts[2,:] = [touchPoses[i].states[0].contact_positions[0].z for i in range(10)]

	print("Height: {}".format(np.mean(touch_pts[2,:])))
	# Fit plane - https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
	touch_pts_mean = np.mean(touch_pts, axis=1, keepdims=True)
	print("Mean of touched points: {}".format(touch_pts_mean))
	touch_pts = touch_pts - touch_pts_mean # Make mean free
	xx = np.sum(touch_pts[0,:]**2)
	yy = np.sum(touch_pts[1,:]**2)
	zz = np.sum(touch_pts[2,:]**2)
	xy = np.sum(touch_pts[0,:]*touch_pts[1,:])
	xz = np.sum(touch_pts[0,:]*touch_pts[2,:])
	yz = np.sum(touch_pts[1,:]*touch_pts[2,:])

	det_x = yy*zz - yz**2
	det_y = xx*zz - xz**2
	det_z = xx*yy - xy**2

	n = np.asarray((xy*yz - xz*yy,xy*xz - yz*xx,det_z))
	n = n/np.linalg.norm(n)
	print("Normal of touch surface: {}".format(n))

def touch_and_refine_table(robot, scene, move_group):
	urdf_robot = URDF.from_parameter_server()
	kdl_kin = KDLKinematics(urdf_robot, urdf_robot.links[1].name, urdf_robot.links[8].name)
	
	move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
	move_group.set_max_acceleration_scaling_factor(0.05)
	
	# Receive table message
	go_home()
	print("Waiting for table message ...")
	the_table = rospy.wait_for_message("/octomap_new/table", table)

	touchPoses = []

	for i in range(10):
		pose_goal = go_to_random_point_over_table(move_group, the_table)
		
		touchSensorMsg = touch_table_straight_on(move_group, robot, kdl_kin, pose_goal)
			
		print("Touched the surface")

		# Check if usable collision
		if '/panda/bumper/panda_probe_ball' in touchSensorMsg.collidingLinks:
			rospy.sleep(2.)
			ballSensorMsg = ContactsState()
			while ballSensorMsg.states == []:
				ballSensorMsg = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)
			#print(ballSensorMsg)
			touchPoses.append(ballSensorMsg)
		else:
			print("Collided with wrong part; ignored")
			i-=1

		print("Recording done, retracting ...")
		
		retract_from_table_straight(move_group, robot, kdl_kin)

		# note eef position when collided, e.g. by listening to /panda/panda/colliding; in real probably ask libfranka
		
	fit_table(touchPoses)

# Notes on getting jacobian etc.
# Install pykdl_utils by checking out git into catkin_ws and catkin_make_isolated


if __name__ == '__main__':
	rospy.init_node('touch_table', anonymous=True)

	moveit_commander.roscpp_initialize(sys.argv)

	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group_name = "panda_arm"    
	move_group = moveit_commander.MoveGroupCommander(group_name)

	path_constr = Constraints()
	path_constr.name = "arm_constr"
	joint_constraint = JointConstraint()
	joint_constraint.position = 0.8
	joint_constraint.tolerance_above = 3.14
	joint_constraint.tolerance_below = 0.1
	joint_constraint.weight = 1
	joint_constraint.joint_name = "panda_joint2"
	path_constr.joint_constraints.append(joint_constraint)
	move_group.set_path_constraints(path_constr)

	try:
		touch_and_refine_table(robot, scene, move_group)
	except KeyboardInterrupt:
		print("Shutting down")
