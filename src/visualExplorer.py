import touchTable2
from std_msgs.msg import Float64
from filter_octomap.msg import table
import rospy
import numpy as np
from math import pi, sqrt
import geometry_msgs.msg
from tf.transformations import random_quaternion
from sensor_msgs.msg import JointState

def distance_pos_list(prev_positions, pos):	
	dist = []
	for p in prev_positions:
		dist.append(sqrt((p.x- pos.x)**2 + (p.y- pos.y)**2 + (p.z- pos.z)**2))
	if len(dist) == 0:
		return [np.inf]
	else:
		return dist

class VisualExplorer(touchTable2.TactileRefiner):
	def __init__(self, entropy_threshold, table_threshold, min_dist_lookouts, group_name = "panda_arm"):
		super(VisualExplorer, self).__init__(group_name)
		
		self.max_reach = 0.9
		self.entropy_threshold = entropy_threshold
		self.table_threshold = table_threshold
		self.scanned_positions = []
		self.min_dist_lookouts = min_dist_lookouts
		self.walk_direction = None
		
		# To know when done --> velocity observer
		rospy.Subscriber("/joint_states", JointState, 
			callback=self.velocity_observer)
		self.robot.total_joint_velocity = np.nan # Single Writer is Callback
		
	def velocity_observer(self, msg):
		self.robot.eef_velocity = np.linalg.norm(msg.velocity[:7])
	
	def go_to_q_safe(self, q):	
		#print("Goal: {}".format(q))
		try: 
			self.move_group.set_max_velocity_scaling_factor(self.max_vel) # Allow 10 % of the set maximum joint velocities
			self.move_group.set_max_acceleration_scaling_factor(self.max_acc)
			self.move_group.set_planning_time(2.0)
			self.move_group.set_goal_joint_tolerance(0.01)
			self.move_group.set_start_state_to_current_state()
			self.move_group.set_joint_value_target(q)
			plan = self.move_group.plan()
			#print("Plan: {}".format(plan))
			if not plan or len(plan.joint_trajectory.points) <= 1:
				print("No plan found (go_to_q_safe)")
				return False
			exec_time = plan.joint_trajectory.points[-1].time_from_start.to_sec()
			ret = self.move_group.execute(plan, wait=False) # Move to goal
			rospy.sleep(exec_time) # Wait till arrived	
		except Exception as e:
			print("Error:")
			print(e)
			return False
		# self.move_group.stop()
		return True
		
	def wait_for_move_end(self):
		print("Wait for end of move")
		while not rospy.is_shutdown():
			#print(state)
			#print(state.velocity[:7])
			#print(np.linalg.norm(state.velocity[:7]))
			if self.robot.eef_velocity < 0.001:
				return
			rospy.sleep(0.01)
		
	def look_around(self):
		# self.move_group.stop()
		failures = 0
		print("Looking around")
		joint_goal = np.asarray(self.robot.get_current_state().joint_state.position[:7])
		for c, j4 in enumerate(np.linspace(self.robot.get_joint('panda_joint5').min_bound()*180/pi+10, self.robot.get_joint('panda_joint5').max_bound()*180/pi-10,num=5)): # Was 7 for full scan. Range: 299 for FOV of 74
			for d, j5 in enumerate((c%2)*np.linspace(self.robot.get_joint('panda_joint6').min_bound()*180/pi+10, self.robot.get_joint('panda_joint6').max_bound()*180/pi-10,num=5) \
									+ ((c+1)%2)*np.linspace(self.robot.get_joint('panda_joint6').max_bound()*180/pi-10, self.robot.get_joint('panda_joint6').min_bound()*180/pi+10,num=5)): # Range of 188 for FOV of 60
				joint_goal[4] = j4 / 180 * pi
				joint_goal[5] = j5 / 180 * pi
				joint_goal[6] = pi/4
				print("Go to {}".format(joint_goal))
				
				if not self.go_to_q_safe(joint_goal):
					failures +=1
					print("Cannot reach {}".format(joint_goal))
				else:
					self.wait_for_move_end()
					rospy.sleep(0.5)
			
		if failures < 10:
			self.scanned_positions.append(self.move_group.get_current_pose().pose.position)
		# self.move_group.stop()
		
	def get_new_random_direction(self):
		position = self.move_group.get_current_pose().pose.position
		move_direction = np.zeros((3))
		# Sample such that tendency to move to center
		move_direction[0] = np.random.uniform(low=-0.5*(position.x+1.), high=.5*abs(position.x-1.), size=(1))
		move_direction[1] = np.random.uniform(low=-0.5*(position.y+1.), high=.5*abs(position.y-1.), size=(1))
		move_direction[2] = np.random.uniform(low=-0.5*(position.z+1.), high=.5*abs(position.z-1.), size=(1))
		
		move_direction = move_direction / np.linalg.norm(move_direction)
		return 0.2 * move_direction
		
	def random_walk(self):
		old_pos = self.move_group.get_current_pose().pose.position
		
		if self.walk_direction is None:
			self.walk_direction = self.get_new_random_direction()
			
		for i in range(10):
			pose_goal = geometry_msgs.msg.Pose()
			pose_goal.position = self.move_group.get_current_pose().pose.position
			print("I'm here:")
			print(pose_goal.position)
			print("Move towards:")
			print(self.walk_direction)
			pose_goal.position.x = self.walk_direction[0] + pose_goal.position.x
			pose_goal.position.y = self.walk_direction[1] + pose_goal.position.y
			pose_goal.position.z = max(0.33, self.walk_direction[2] + pose_goal.position.z) # Make sure he stays over the table and shoulder such that less knotted
					
			# ori = random_quaternion()
			# pose_goal.orientation.x = ori[0]
			# pose_goal.orientation.y = ori[1]
			# pose_goal.orientation.z = ori[2]
			# pose_goal.orientation.w = ori[3]
			pose_goal.orientation = self.move_group.get_current_pose().pose.orientation
			
			print("Goal: {}".format(pose_goal))
			
			tmp_tolerance = self.move_group.get_goal_orientation_tolerance()
			self.move_group.set_goal_orientation_tolerance(1.) # Eef orientation not important, look around anyway
			self.go_to_pose(pose_goal)
			self.move_group.set_goal_orientation_tolerance(tmp_tolerance) 
			
			new_pos = self.move_group.get_current_pose().pose.position
			print("Post move I'm:")
			print(new_pos)
			if((old_pos.x - new_pos.x)**2 + (old_pos.y - new_pos.y)**2 + (old_pos.z - new_pos.z)**2 > 0.1**2): # Have gone 10 cm, maybe look around?
				return
			else: # Have not gone far, try a bit farther
				self.walk_direction *= 1.05
				
		self.walk_direction = None # Did not manage to go 10 cm --> forget this direction and choose new next time
		
	def run(self):
		while not rospy.is_shutdown():
			try:
				mapEntropyMsg = rospy.wait_for_message("/octomap_new/entropy", Float64,  timeout=2.)
				tableMsg = rospy.wait_for_message("/octomap_new/table", table,  timeout=2.)
				
				if mapEntropyMsg.data < self.entropy_threshold and tableMsg.score > self.table_threshold:
					print("Table found and map good enough")
					print("Moving on to touching the table")
					return True
			except rospy.exceptions.ROSException:
				print("No feedback on progress received")
				
			if min(distance_pos_list(self.scanned_positions, self.move_group.get_current_pose().pose.position)) > self.min_dist_lookouts and \
				self.move_group.get_current_pose().pose.position.z > 0.2:  # Min distance from table ...
				self.look_around()
			else:
				print("Been here before; move on")
				
			self.random_walk()
