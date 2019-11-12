from filter_octomap.msg import table
import numpy as np
import random
import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import subprocess
from copy import deepcopy
import math

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

class TactileRefiner(object):
	def __init__(self,  group_name = "panda_arm" ):
		# Moveit init
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()

		self.scene = moveit_commander.PlanningSceneInterface()
		   
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.max_vel = 1.0
		self.max_acc = 0.1
		self.move_group.set_max_velocity_scaling_factor(self.max_vel) # Allow 10 % of the set maximum joint velocities
		self.move_group.set_max_acceleration_scaling_factor(self.max_acc)
		
		# URDF and kinematics init
		self.urdf_robot = URDF.from_parameter_server()
		self.kdl_kin = KDLKinematics(self.urdf_robot, self.urdf_robot.links[1].name, self.urdf_robot.links[9].name)
		
		# Start working
		self.go_home()
	
	def wait_for_joint_arrival(self, q, repeat_time = 1.):
		new = True
		start = rospy.Time.now()
		while not rospy.is_shutdown() and \
			(np.linalg.norm(q-self.robot.get_current_state().joint_state.position[:7]) > 0.1 or \
			np.linalg.norm(self.robot.get_current_state().joint_state.velocity[:7]) > 0.1):
			if new:
				print("Waiting for arrival")
				new = False
				
			if start + rospy.Duration.from_sec(repeat_time) < rospy.Time.now():
				print("Reissue")
				self.go_to_q(q, wait=False)
				start = rospy.Time.now()

	def go_to_q(self, q, dt=1., wait=True, exact=False):
		"""
		dt deprecated; NEW: scale sucht that maximum joint velocity is 1/4 rad/s
		"""
		q = np.asarray(q)
		if not exact and np.linalg.norm(q-np.asarray(self.move_group.get_current_joint_values())) < 0.1:
			print("Close enough already")
			return
		client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		#print("Waiting for action server ...")
		client.wait_for_server()
		goal = FollowJointTrajectoryGoal()
		joint_traj = JointTrajectory()
		joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
		point1 = JointTrajectoryPoint()
		point1.positions = q.tolist()
		point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
		delta_joints = np.abs(np.asarray(point1.positions) - np.asarray(self.move_group.get_current_joint_values()))
		point1.time_from_start = rospy.Duration(max(0.5, np.max(delta_joints) / 0.25))
		joint_traj.points = [point1]
		goal.trajectory = joint_traj
		joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
		client.send_goal_and_wait(goal)
		if wait:
			self.wait_for_joint_arrival(q)
	
	def go_home(self):
		print("Homing ...")
		self.go_to_q(np.asarray([0.0, -1.5, 0.0, -1.8702679826187074, 0, 2.873639813867795 - 1.51, 0.06015034910227879])) # home configuration
	
	def get_current_pose(self):	
		return self.move_group.get_current_pose().pose
	
	def position_msg_to_numpy(self, position_msg):
		return np.asarray((position_msg.x, position_msg.y, position_msg.z))
	
	def orientation_msg_to_numpy(self, orientation_msg):
		return np.asarray((orientation_msg.x, orientation_msg.y, \
			orientation_msg.z, orientation_msg.w))
	
	def numpy_to_position(self, arr):
		pos = geometry_msgs.msg.Point()
		pos.x = arr[0]
		pos.y = arr[1]
		pos.z = arr[2]
		return pos
	
	def numpy_to_orientation(self, arr):
		ori = geometry_msgs.msg.Quaternion()
		ori.x = arr[0]
		ori.y = arr[1]
		ori.z = arr[2]
		ori.w = arr[3]
		return ori
	
	def get_current_position(self):
		return self.position_msg_to_numpy(self.get_current_pose().position)
	
	def get_current_rotation(self):
		return self.orientation_msg_to_numpy(self.get_current_pose().orientation)
	
	def get_distance_to_position(self, pos):
		if type(pos) == geometry_msgs.msg.Point:
			pos = self.position_msg_to_numpy(pos)
		return np.linalg.norm(self.get_current_position() - pos)
	
	def get_distance_to_rot(self, rot):
		if type(rot) == geometry_msgs.msg.Quaternion:
			rot = self.orientation_msg_to_numpy(rot)
		return np.linalg.norm(self.get_current_rotation() - rot)
	
	def go_to_pose(self, pose):
		q = np.asarray(self.robot.get_current_state().joint_state.position[:7])
		q_ik = self.kdl_kin.inverse(pose, q)
		if q_ik is None:
			print("Not even IK works")
			return False
		self.move_group.set_max_velocity_scaling_factor(self.max_vel) # Allow 10 % of the set maximum joint velocities
		self.move_group.set_max_acceleration_scaling_factor(self.max_acc)
		self.move_group.set_planning_time(2.0)
		# (plan, fraction) = self.move_group.compute_cartesian_path([pose], 0.05, 1.0)
		self.move_group.set_start_state_to_current_state()
		self.move_group.set_pose_target(pose, self.move_group.get_end_effector_link())
		plan = self.move_group.plan()
		if not plan or len(plan.joint_trajectory.points) <= 1:
			return False
		#print(plan)
		exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
		if not (exec_time < 15.0) \
			or not (1 < len(plan.joint_trajectory.points) < 100):
			#fraction < 0.2 or 
			return False
		try:
			ret = self.move_group.execute(plan, wait=False) # Move to goal
			rospy.sleep(exec_time) # Wait till arrived		
			# print(plan.joint_trajectory.points[-1])
			self.go_to_q(plan.joint_trajectory.points[-1].positions, dt=0.2)
			# self.move_group.stop()
			rospy.sleep(1.)
			return True	
		except KeyboardInterrupt:
			return False
		except Exception as e:
			print(e)
			return False
	
	def p_step(self, pos_d=None, ori_d=None, pose_d=None):
		""" Go to desired pos and ori with single step ik (should be close to current pos) """
		assert (pos_d is None and ori_d is None) != (pose_d is None), "Either pose or pos and ori"
		if pose_d is None:
			pose_d = geometry_msgs.msg.Pose()
			pose_d.position = self.numpy_to_position(pos_d)
			pose_d.orientation = self.numpy_to_orientation(ori_d)
		
		q = np.asarray(self.robot.get_current_state().joint_state.position[:7])
		q_ik = self.kdl_kin.inverse(pose_d, q)
		
		if q_ik is not None:
			self.go_to_q(q_ik, wait=True, exact=True)
			return 0
		else:
			print("IK not successful")
			return -1
	
	def go_straight_till_coll(self, x_step=0, y_step=0, z_step=0):
		""" Moves along step direction till a collision is sensed; if probe_ball collided returns collisionstate else None """
		pos_to_keep = self.get_current_position()
		ori_to_keep = self.get_current_rotation()
		IK_retries = 10
		while IK_retries>0:
			pos_to_keep[0] += x_step
			pos_to_keep[1] += y_step
			pos_to_keep[2] += z_step
			IK_retries += self.p_step(pos_to_keep, ori_to_keep)
			rospy.sleep(0.8)
			touchSensorMsg = rospy.wait_for_message("/panda/bumper/colliding", CollisionState)
			if touchSensorMsg.colliding:
				print("Detected Collision")
				if '/panda/bumper/panda_probe_ball' in touchSensorMsg.collidingLinks: # Collided with sensing part -> wait for details
					ballSensorMsg = ContactsState()
					start = rospy.Time.now()
					while ballSensorMsg.states == []:
						try:
							ballSensorMsg = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState, timeout=0.5)
							break
						except:
							print("Time out on collision")
						if rospy.Time.now() > start + rospy.Duration.from_sec(3): # Don't wait for ever
							print("False alarm")
							break
					print(ballSensorMsg)
					if len(ballSensorMsg.states) > 0:
						print("Touched with state: \n{}".format(ballSensorMsg))
						return ballSensorMsg
					else:
						continue
				else: # Collided with wrong part -> ignore this collision in later evaluation
					return None

	def n_steps_dir(self, n, x_step=0., y_step=0., z_step=0.):
		pos_to_keep = self.get_current_position()
		ori_to_keep = self.get_current_rotation()
		
		for i in range(n):
			pos_to_keep[0] += x_step
			pos_to_keep[1] += y_step
			pos_to_keep[2] += z_step
			self.p_step(pos_to_keep, ori_to_keep)
			
	def rotate_quart_around_axis(self, quart, axis, angle):
		turn = tf.transformations.quaternion_about_axis(angle, axis)
		quart = self.orientation_msg_to_numpy(quart)
		
		return self.numpy_to_orientation(tf.transformations.quaternion_multiply(quart, turn))
		
	def rotate_quart_around_z(self, quart, yaw_angle=0.1):
		""" rotates pose encoded by quart by yaw_angle about its z axis """
		return self.rotate_quart_around_axis(quart, (0,0,1), yaw_angle)
	
	def try_to_go_to_pose(self, pose_goal, arrival_test= lambda _1,_2: True, modifier = lambda _1,_2: _2, timeout = 10):
		""" Tries to go to a random position over the table; gives up after timeout planning attempts; return True if close to the desired point """
		print("Trying to reach")
		print(pose_goal)
		while timeout > 0:
			self.go_to_pose(pose_goal)
			if arrival_test(self, pose_goal):
				print("Arrived")
				return True
			else:
				timeout -= 1
				pose_goal = modifier(self, pose_goal)
				print("Try new pose: {}".format(pose_goal))
				continue
				
		return False
		
	def pose_pub(self, name, pose):
		pose_pub = rospy.Publisher(name, geometry_msgs.msg.PoseStamped, queue_size=1, latch=True)
		pose_goal_st = geometry_msgs.msg.PoseStamped()
		pose_goal_st.pose = pose
		pose_goal_st.header.stamp = rospy.Time.now()
		pose_goal_st.header.frame_id = "world"
		pose_pub.publish(pose_goal_st)

class TableRefiner(TactileRefiner):
	def __init__(self, group_name = "panda_arm"):
		super(TableRefiner, self).__init__(group_name)
		
		# Publisher for refined table
		self.table_publisher = rospy.Publisher("/octomap_new/table_touched", table, queue_size=1, latch=True)
		
		print("Waiting for table message ...")
		self.table_msg = rospy.wait_for_message("/octomap_new/table", table)
		
		self.touched_points = []
		
		self.max_reach = 0.9 # Maximum distance from 0,0,0 the robot might be able to reach
	
	def sample_pose_over_table(self, ox = 1, oy = 0, oz = 0, ow = 0):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.z = self.table_msg.centroid_position.z + 0.3
		pose_goal.orientation.x = ox # tool pointing down
		pose_goal.orientation.y = oy
		pose_goal.orientation.z = oz
		pose_goal.orientation.w = ow
		
		# sample postions over table till one is reachable
		while not rospy.is_shutdown():
			pose_goal.position.x = random.uniform(self.table_msg.min.x+.1, self.table_msg.max.x-.1)
			pose_goal.position.y = random.uniform(self.table_msg.min.y+.1, self.table_msg.max.y-.1)
			
			if pose_goal.position.x**2+pose_goal.position.y**2+pose_goal.position.z**2 < self.max_reach**2:
				return pose_goal
		
	def over_table(self):
		pos = self.get_current_position()
		
		return (self.table_msg.min.x+0.05 < pos[0] < self.table_msg.max.x-0.05) \
			and (self.table_msg.min.y+0.05 < pos[1] < self.table_msg.max.y-0.05)
		
	def go_to_random_point_over_table(self, timeout = 100):
		""" Tries to go to a random position over the table; gives up after timeout planning attempts; return True if close to the desired point """
		def arrived_over_table(self, pose_goal):
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.get_current_rotation())
			while roll < 0:
				roll += 2*math.pi # need roll in 0 to 2 pi
				
			print("Errors: dist: {}, rpy: {}".format(self.get_distance_to_position(pose_goal.position), (roll, pitch, yaw)))
			return (self.over_table() and \
				self.get_distance_to_position(pose_goal.position) < 0.1 and \
				roll > 2.967 and roll < 3.316 and \
				abs(pitch) < 0.175) # x about downwards <=> roll = 180+-10 pitch = 0+-10
		
		def modify_over_table(self, pose_goal):
			pose_goal.orientation = self.rotate_quart_around_z(pose_goal.orientation, yaw_angle=random.uniform(0, 3.141))
			return pose_goal
		
		
		goal_pose = self.sample_pose_over_table()
		return self.try_to_go_to_pose(goal_pose, arrival_test=arrived_over_table, modifier=modify_over_table)
		
	def approach_table_straight(self):
		""" Approaches the table by moving along the long axis of the probing tool till a collision is sensed """
		return self.go_straight_till_coll(z_step=-0.002)

	def depart_from_table(self):
		""" More or less safely moves away from table """
		self.n_steps_dir(5, z_step=0.03)
		
	def fit_table(self):
		# fit table surface with new information from the touched points and display info
		#print("Touched surface at the following poses:")
		#print(touchPoses)
		print("Improved surface estimate:")
		touch_pts = np.zeros((3,len(self.touched_points)))
		touch_pts[0,:] = [self.touched_points[i].states[0].contact_positions[0].x for i in range(len(self.touched_points))]
		touch_pts[1,:] = [self.touched_points[i].states[0].contact_positions[0].y for i in range(len(self.touched_points))]
		touch_pts[2,:] = [self.touched_points[i].states[0].contact_positions[0].z for i in range(len(self.touched_points))]

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
		if(n[2] < 0): # Table always pointing up
			n = -1.*n
		print("Normal of touch surface: {}".format(n))
		
		new_table = deepcopy(self.table_msg)
		new_table.normal.x = n[0]
		new_table.normal.y = n[1]
		new_table.normal.z = n[2]
		if "/franka_state_controller/F_ext" in [x for x,y in rospy.get_published_topics()]: # Indicator for real robot; hacky
			new_table.max.z = touch_pts_mean[2,:] 
		else:
			new_table.max.z = touch_pts_mean[2,:] - 0.5 # Offset between world in gazebo and in ros
		new_table.header.stamp = rospy.Time.now()
		
		print(new_table)
		self.table_publisher.publish(new_table)
	
	def touch_and_refine_table(self, touchPts=5):
		self.go_home()
		
		while(len(self.touched_points) < touchPts):
			self.go_home()
			arrived_over_probe_point = self.go_to_random_point_over_table()
			if not arrived_over_probe_point:
				continue
			touch_point = self.approach_table_straight()
			if touch_point:
				self.touched_points.append(touch_point)
				self.depart_from_table()
		self.fit_table()

if __name__ == '__main__':
	rospy.init_node('touch_table', anonymous=True)
	refiner = TableRefiner()

	try:
		refiner.touch_and_refine_table(touchPts=3) # 3 is min for normal fit ...
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
