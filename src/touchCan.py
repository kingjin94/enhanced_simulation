#!/usr/bin/python

from filter_octomap.msg import table, cans, can
import rospy
import numpy as np
import geometry_msgs
import random
import tf
import math
from scipy import optimize

import touchTable2

class CanRefiner(touchTable2.TactileRefiner):
	def __init__(self, group_name = "panda_arm"):
		super(CanRefiner, self).__init__(group_name=group_name)
		
		# Publisher for refined table
		self.can_publisher = rospy.Publisher("/octomap_new/cans_touched", cans, queue_size=1, latch=True)
		
		print("Waiting for can message ...")
		self.cans_msg = rospy.wait_for_message("/octomap_new/cans", cans)
		print("Waiting for table touched message ...")
		self.table_msg = rospy.wait_for_message("/octomap_new/table_touched", table)
		
		# Select highest scoring can to refine
		scores = [can.score for can in self.cans_msg.cans]
		self.can_msg = self.cans_msg.cans[np.argmax(scores)]
		print("Concentrating on can: \n{}".format(self.can_msg))
		
		self.touched_points_mantle = []
		self.touched_points_top = []
		
	def sample_point_on_top(self):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.z = self.table_msg.max.z + self.can_msg.height + 0.25 # 20 cm above can
		pose_goal.orientation.x = 1. # tool pointing down
		pose_goal.orientation.y = 0.
		pose_goal.orientation.z = 0.
		pose_goal.orientation.w = 0.
		
		# sample postions over can till one is reachable
		while not rospy.is_shutdown():
			# Uniform sample on top of can --> sample in surounding square, filter those in the circle, add offset to centroid
			pose_goal.position.x = random.uniform(-1.*self.can_msg.radius, 1.*self.can_msg.radius)
			pose_goal.position.y = random.uniform(-1.*self.can_msg.radius, 1.*self.can_msg.radius)
			
			if pose_goal.position.x**2+pose_goal.position.y**2 < self.can_msg.radius**2:
				pose_goal.position.x += self.can_msg.centroid_position.x
				pose_goal.position.y += self.can_msg.centroid_position.y 
				
				if self.pose_reachable(pose_goal):
					return pose_goal
		
	def touch_and_refine_can_top(self, touch_pts=3):
		""" Touches the top of the can at three points to refine the cans height and position """
		
		def arrived_over_can(self, pose_goal):
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.get_current_rotation())
			while roll < 0:
				roll += 2*math.pi # need roll in 0 to 2 pi
				
			print("Errors: dist: {}, rpy: {}".format(self.get_distance_to_position(pose_goal.position), (roll, pitch, yaw)))
			return (self.get_distance_to_position(pose_goal.position) < 0.02 and \
				roll > 2.967 and roll < 3.316 and \
				abs(pitch) < 0.175) # x about downwards <=> roll = 180+-10 pitch = 0+-10
		
		def modify_over_can(self, pose_goal):
			pose_goal.orientation = self.rotate_quart_around_z(pose_goal.orientation, yaw_angle=random.uniform(0, 3.141))
			return pose_goal
		
		while len(self.touched_points_top) < touch_pts and not rospy.is_shutdown():
			self.go_home()
			# Sample random point on top
			pose_goal = self.sample_point_on_top()
			# Try to touch; on fail continue, else add to touched_points_top
			if self.try_to_go_to_pose(pose_goal, arrival_test=arrived_over_can, modifier=modify_over_can):
				# Arrived successfully --> touch
				coll_state = self.go_straight_till_coll(z_step=-0.01)
				self.n_steps_dir(5, z_step=0.03) # retreat
				if coll_state:
					self.touched_points_top.append(coll_state)
				else:
					print("Collided with not sensing part")
					continue # Collided with other part -> no usable info gained so retry
			else:
				print("Did not arrive at goal pose")
				continue
	
	def sample_point_on_mantle(self):			
		pose_goal = geometry_msgs.msg.Pose()
		while not rospy.is_shutdown():
			pose_goal.position.z = self.table_msg.max.z + 0.03 + random.uniform(0, self.can_msg.height-0.03) # Can not touch can near table surface
			alpha = random.uniform(0, 2*math.pi)
			pose_goal.position.x = math.cos(alpha) * (self.can_msg.radius+0.25) + self.can_msg.centroid_position.x
			pose_goal.position.y = math.sin(alpha) * (self.can_msg.radius+0.25) + self.can_msg.centroid_position.y 
			
			# Find orientation s.t. z_hand points into the can and y_hand is parallel to the world's z axis
			x_hand = np.asarray((-math.sin(alpha), math.cos(alpha), 0))
			y_hand = np.asarray((0, 0, -1))
			z_hand = np.asarray((-math.cos(alpha), -math.sin(alpha), 0))
			T = np.zeros((4,4))
			T[3,3] = 1.
			T[0,:3] = x_hand
			T[1,:3] = y_hand
			T[2,:3] = z_hand
			#print(T.T)
			pose_goal.orientation = self.numpy_to_orientation(tf.transformations.quaternion_from_matrix(T.T))
			if self.pose_reachable(pose_goal):
				self.pose_pub("/debug/can_mantle/pose", pose_goal)
				return pose_goal
	
	def mantle_normal_at_pose(self, pose):
		""" Calculates the the normal of the mantle at the closest point to the given pose"""
		p = self.position_msg_to_numpy(pose.position) # world coordinates
		p -= self.position_msg_to_numpy(self.can_msg.centroid_position) # coordinates relative to can centroid
		p_proj = np.asarray((0,0,1)) * p[2] # project on can axis (= z axis)
		n_mantle = p-p_proj # normal of mantle is the part of p not along the cans axis
		return n_mantle / np.linalg.norm(n_mantle)
	
	def touch_and_refine_can_mantle(self, touch_pts=5):
		""" Touches the mantle of the can at multiple points to refine the cans radius and position """
		
		def arrived_before_can(self, pose_goal):
			# Requirements: z aligned with cylinder mantle
			T = tf.transformations.quaternion_matrix(self.get_current_rotation())
			z_axis = T[:3, 2]
			z_wanted = -1. * self.mantle_normal_at_pose(pose_goal) # tool should point into mantle
			print("Wanted z: {}".format(z_wanted))
				
			print("Errors: dist: {}, z: {}".format(self.get_distance_to_position(pose_goal.position), z_axis))
			return (self.get_distance_to_position(pose_goal.position) < 0.02 and \
				np.inner(z_axis, z_wanted) > 0.99) # <a,b> = cos(angle(a,b)) for a, b unit vectors; cos(g) = 0.99 <=> g = 8 deg
		
		def modify_before_can(self, pose_goal):
			pose_goal.orientation = self.rotate_quart_around_z(pose_goal.orientation, yaw_angle=random.uniform(0, 2*math.pi))
			self.pose_pub("/debug/can_mantle/pose_altered", pose_goal)
			return pose_goal
		
		while len(self.touched_points_mantle) < touch_pts and not rospy.is_shutdown():
			self.go_home()
			# Sample random point on top
			pose_goal = self.sample_point_on_mantle()
			# Try to touch; on fail continue, else add to touched_points_top
			if self.try_to_go_to_pose(pose_goal, arrival_test=arrived_before_can, modifier=modify_before_can):
				# Arrived successfully --> go exactly to goal then touch
				self.p_step(pose_d=pose_goal)
				axis = -0.01 * self.mantle_normal_at_pose(pose_goal)
				coll_state = self.go_straight_till_coll(x_step = axis[0], y_step = axis[1]) 
				self.n_steps_dir(5, x_step = -2.*axis[0], y_step = -2.*axis[1]) # retreat
				if coll_state:
					self.touched_points_mantle.append(coll_state)
				else:
					print("Collided with not sensing part")
					continue # Collided with other part -> no usable info gained so retry
			else:
				print("Did not arrive at goal pose")
				continue
	
	def fit_can_and_publish(self):
		assert len(self.touched_points_top) > 0, "To few points on top to refine, need atleast 1"
		assert len(self.touched_points_mantle) >= 3, "To few points on mantle, need atleast 3 have {}".format(len(self.touched_points_mantle))
			
		new_can = can()
		# Height from np.mean(self.touched_points_top.position.z) - self.table_msg.max.z
		# Centroid.z = Height/2 + self.table_msg.max.z
		# Radius & centroid x/y from least squares fit of circle to x,y of points on mantle
		z_can_top = np.mean([coll.states[0].contact_positions[0].z - 0.5 for coll in self.touched_points_top])  # Offset gazebo <-> ros world
		new_can.height = z_can_top - self.table_msg.max.z
		new_can.centroid_position.z = self.table_msg.max.z + new_can.height/2
		
		# find radius and x/y via circle fit -- https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
		def calc_R(x,y, xc, yc):
			""" calculate the distance of each 2D points from the center (xc, yc) """
			return np.sqrt((x-xc)**2 + (y-yc)**2)

		def Error(c, x, y):
			""" calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
			Ri = calc_R(x, y, *c)
			return Ri - Ri.mean()
		
		x_i = self.can_msg.centroid_position.x
		y_i = self.can_msg.centroid_position.y
		x_mantle = np.asarray([coll.states[0].contact_positions[0].x for coll in self.touched_points_mantle])
		y_mantle = np.asarray([coll.states[0].contact_positions[0].y for coll in self.touched_points_mantle])

		center, ier = optimize.leastsq(Error, (x_i, y_i), args=(x_mantle,y_mantle))
		new_can.centroid_position.x = center[0]
		new_can.centroid_position.y = center[1]
		new_can.radius = calc_R(x_mantle, y_mantle, *center).mean()
		
		new_can.score = self.can_msg.score * 10 # Way better estimate as touched ...
		new_cans = cans()
		new_cans.cans.append(new_can)
		new_cans.header = self.cans_msg.header
		new_cans.count = 1
		new_cans.header.stamp = rospy.Time.now()
		
		self.can_publisher.publish(new_cans)
		
	def touch_and_refine_can(self):
		self.touch_and_refine_can_top() 
		self.touch_and_refine_can_mantle()
		
		self.fit_can_and_publish()

if __name__ == '__main__':
	rospy.init_node('touch_can', anonymous=True)
	try:
		table_msg = rospy.wait_for_message("/octomap_new/table_touched", table,  timeout=5.)
	except rospy.exceptions.ROSException as e:
		print("Table not touched; publishing own")
		new_table = table()
		new_table.normal.x = 0.
		new_table.normal.y = 0.
		new_table.normal.z = 1.
		new_table.max.x = 1.26
		new_table.max.y = 0.46
		new_table.max.z = 0.375
		new_table.min.x = 0.34
		new_table.min.y = -0.46
		new_table.min.z = 0.36
		new_table.centroid_position.x = 0.800836920738
		new_table.centroid_position.y = -0.00100824853871
		new_table.centroid_position.z = 0.359990298748
		new_table.score = 913584.590061
		new_table.header.stamp = rospy.Time.now()
		new_table.header.seq = 1
		new_table.header.frame_id = "world"
		
		table_publisher = rospy.Publisher("/octomap_new/table_touched", table, queue_size=1, latch=True)
		table_publisher.publish(new_table)
	
	refiner = CanRefiner()
	print("Refine started")
	refiner.touch_and_refine_can()
	rospy.spin()
