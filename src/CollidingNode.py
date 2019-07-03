#!/usr/bin/python
import rospy
from gazebo_msgs.msg import ContactsState
from enhanced_sim.msg import CollisionState

"""
Spec:
Look at all /panda/bumper* topics, get which are colliding and 
publish a boolean variable (per link) saying whether in collision
"""

class CollidingNode(object):
	def __init__(self, collision_state_update_freq = 10):
		self.update_rate = rospy.Rate(collision_state_update_freq)
		self.pub = rospy.Publisher('/panda/bumper/colliding', CollisionState, queue_size=10)

		# find all bumper topics and subscribe
		topics = rospy.get_published_topics()
		self.observedLinks = [] # List of all links under observance
		self.last_collision_times = {} # Last collision time for each link
		for name, msgType in topics:
			if msgType == "gazebo_msgs/ContactsState" and "/panda/bumper/panda_" in name:
				rospy.Subscriber(name, ContactsState, 
					callback=self.collision_listener, callback_args=name)
				self.observedLinks.append(name)
				self.last_collision_times[name] = rospy.get_rostime()

	def collision_listener(self, msg, link_name):
		if any(msg.states): # atleast one valid collision point
			if msg.header.stamp > self.last_collision_times[link_name]:
				self.last_collision_times[link_name] = msg.header.stamp	
	
	def start(self):
		while not rospy.is_shutdown():
			# Have less than 10 ms passed since last collision?
			msg = CollisionState()
			msg.colliding = False
			for k, v in self.last_collision_times.items():
				if rospy.get_rostime() <= rospy.Duration(1, 10000000) + v:
					msg.colliding = True
					msg.collidingLinks.append(k)
			self.pub.publish(msg)
			self.update_rate.sleep() # Wait till next update needs to be sent

if __name__ == '__main__':
	rospy.init_node("SimpleCollStateGenerator", anonymous=True)
	thisNode = CollidingNode()
	thisNode.start()

