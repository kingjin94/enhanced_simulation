#!/usr/bin/python
import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool

"""
Spec:
Look at all /panda/bumper* topics, get which are colliding and 
publish a boolean variable (per link) saying whether in collision
"""

class CollidingNode(object):
	def __init__(self, collision_state_update_freq = 10):
		self.update_rate = rospy.Rate(collision_state_update_freq)
		self.pub = rospy.Publisher('/panda/bumper/colliding', Bool, queue_size=10)
		self.last_collision_time = rospy.get_rostime() # Save last collision time for cooldown

		# find all bumper topics and subscribe
		topics = rospy.get_published_topics()
		for name, msgType in topics:
			if msgType == "gazebo_msgs/ContactsState" and "/panda/bumper/panda_" in name:
				rospy.Subscriber(name, ContactsState, self.collision_listener)

	def collision_listener(self, msg):
		if any(msg.states): # atleast one valid collision point
			if msg.header.stamp > self.last_collision_time:
				self.last_collision_time = msg.header.stamp	
	
	def start(self):
		while not rospy.is_shutdown():
			# Have less than 10 ms passed since last collision?
			self.pub.publish(Bool(rospy.get_rostime() <= rospy.Duration(1, 10000000) + self.last_collision_time))
			self.update_rate.sleep() # Wait till next update needs to be sent

if __name__ == '__main__':
	rospy.init_node("BoolCollGenerator", anonymous=True)
	thisNode = CollidingNode()
	thisNode.start()

