
import geometry_msgs.msg

test = geometry_msgs.msg.PointStamped()

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import ContactsState
rospy.init_node("Test", anonymous=True)


msg  = rospy.wait_for_message("/panda/ft/tool", WrenchStamped)
F = np.asarray((msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z))
M = np.asarray((msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z))
# get ground truth, normal is published in world frame it seems
msg_contact = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)
while len(msg_contact.states) == 0:
	msg_contact = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)

test.header = msg_contact.header

test.point = msg_contact.states[0].contact_positions[0]

import tf

transf = tf.TransformListener()

now = rospy.get_rostime()
transf.waitForTransform("/world", '/panda_probe_cyl', now, rospy.Duration(3))
test.header.stamp = now
test_cyl_frame = transf.transformPoint('/panda_probe_cyl', test)

#test_ball_frame = test_cyl_frame
#test_ball_frame.point.z -= 0.07
#test_ball_frame.header.frame_id="/panda_probe_ball"
