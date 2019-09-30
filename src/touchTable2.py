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

class tableRefiner:
	def __init__(self, node_name='touch_table', group_name = "panda_arm" ):
		rospy.init_node(node_name, anonymous=True)
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()

		self.scene = moveit_commander.PlanningSceneInterface()
		   
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
		self.move_group.set_max_acceleration_scaling_factor(0.05)
		
		self.urdf_robot = URDF.from_parameter_server()
		self.kdl_kin = KDLKinematics(self.urdf_robot, self.urdf_robot.links[1].name, self.urdf_robot.links[8].name)

		
		self.go_home()
		
		print("Waiting for table message ...")
		self.table_msg = rospy.wait_for_message("/octomap_new/table", table)
		
		self.touched_points = []
		
		self.max_reach = 1.0 # Maximum distance from 0,0,0 the robot might be able to reach
	
	def wait_for_joint_arrival(self, q):
		new = True
		while not rospy.is_shutdown() and \
			(np.linalg.norm(q-self.robot.get_current_state().joint_state.position[:7]) > 0.1 or \
			np.linalg.norm(self.robot.get_current_state().joint_state.velocity[:7]) > 0.1):
			if new:
				print("Waiting for arrival")
				new = False

			
	def go_to_q(self, q, dt=1., wait=True):
		q = np.asarray(q)
		client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		#print("Waiting for action server ...")
		client.wait_for_server()
		goal = FollowJointTrajectoryGoal()
		joint_traj = JointTrajectory()
		joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
		point1 = JointTrajectoryPoint()
		point1.positions = q.tolist()
		point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
		point1.time_from_start = rospy.Duration(dt)
		joint_traj.points = [point1]
		goal.trajectory = joint_traj
		joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
		client.send_goal_and_wait(goal)
		if wait:
			self.wait_for_joint_arrival(q)
		
	def go_home(self):
		print("Homing ...")
		self.go_to_q(np.asarray([0.0, -1.5, 0.0, -1.8702679826187074, 0, 2.873639813867795 - 1.51, 0.06015034910227879])) # home configuration
		
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
		(plan, fraction) = self.move_group.compute_cartesian_path([pose], 0.01, 1.0)
		exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
		if fraction < 0.2 or not (exec_time < 10.0) \
			or not (1 < len(plan.joint_trajectory.points) < 100):
			pose.orientation = self.rotate_quart_around_z(pose.orientation, yaw=random.uniform(0, 3.141))
			return False
		try:
			ret = self.move_group.execute(plan, wait=False) # Move to goal
			rospy.sleep(exec_time) # Wait till arrived		
			# print(plan.joint_trajectory.points[-1])
			self.go_to_q(plan.joint_trajectory.points[-1].positions, dt=0.2)
			self.move_group.stop()
			rospy.sleep(1.)
			return True	
		except KeyboardInterrupt:
			return False
		except Exception as e:
			print(e)
			return False
			
	def rotate_quart_around_z(self, quart, yaw=0.1):
		turn = tf.transformations.quaternion_about_axis(yaw, (0,0,1))
		quart = self.orientation_msg_to_numpy(quart)
		
		return self.numpy_to_orientation(tf.transformations.quaternion_multiply(quart, turn))
		
	def over_table(self):
		pos = self.get_current_position()
		
		return (self.table_msg.min.x+0.05 < pos[0] < self.table_msg.max.x-0.05) \
			and (self.table_msg.min.y+0.05 < pos[1] < self.table_msg.max.y-0.05)
			
	def ori_geq_zero(self, a):

		return a
	
	def go_to_random_point_over_table(self, call_count=0):
		timeout = 100
		goal_pose = self.sample_pose_over_table()
		print("Trying to reach")
		print(goal_pose)
		while timeout > 0:
			self.go_to_pose(goal_pose)
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.get_current_rotation())
			while roll < 0:
				roll += 2*math.pi # need roll in 0 to 2 pi
			if not self.over_table() or self.get_distance_to_position(goal_pose.position) > 0.1 or \
				roll < 2.967 or roll > 3.316 or abs(pitch) > 0.175:  # x about downwards <=> roll = 180+-10, pitch = 0+-10
				print("Errors: dist: {}, rpy: {}".format(self.get_distance_to_position(goal_pose.position), (roll, pitch, yaw)))
				timeout -= 1
				print("Try new ori: {}".format(goal_pose.orientation))
				continue
			else:
				print("Arrived")
				return True
				
		return False
		
	def p_step(self, pos_d, ori_d):
		""" Make a step proportional to an error """
		pos_e = pos_d - self.get_current_position()
		rot_e = ori_d - self.get_current_rotation()
		
		rot_e = np.asarray(tf.transformations.euler_from_quaternion(rot_e)) / 300
		e = np.concatenate([pos_e, rot_e])
		print("Error: {}".format(e))
		
		q = np.asarray(self.robot.get_current_state().joint_state.position[:7])
		J = np.asarray(self.kdl_kin.jacobian(q))
		
		q_new = q+J.T.dot(e)
		self.go_to_q(q_new, wait=False)
		
	def approach_table_straight(self):
		""" Approaches the table by moving along the long axis of the probing tool till a collision is sensed """
		pos_to_keep = self.get_current_position()
		ori_to_keep = self.get_current_rotation()
		while True:
			pos_to_keep[2] -= 0.01
			self.p_step(pos_to_keep, ori_to_keep)
			touchSensorMsg = rospy.wait_for_message("/panda/bumper/colliding", CollisionState)
			if touchSensorMsg.colliding:
				if '/panda/bumper/panda_probe_ball' in touchSensorMsg.collidingLinks: # Collided with sensing part -> wait for details
					ballSensorMsg = ContactsState()
					while ballSensorMsg.states == []:
						ballSensorMsg = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)
					print("Touched table with state: \n{}".format(ballSensorMsg))
					return ballSensorMsg
				else: # Collided with wrong part -> ignore this collision in later evaluation
					return None
	
	def depart_from_table(self):
		""" More or less safely moves away from table """
		pos_to_keep = self.get_current_position()
		ori_to_keep = self.get_current_rotation()
		
		for i in range(5):
			pos_to_keep[2] += 0.03
			self.p_step(pos_to_keep, ori_to_keep)
		
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
		print("Normal of touch surface: {}".format(n))
	
	def touch_and_refine_table(self, touchPts=10):
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
	refiner = tableRefiner()

	try:
		refiner.touch_and_refine_table()
	except KeyboardInterrupt:
		print("Shutting down")
