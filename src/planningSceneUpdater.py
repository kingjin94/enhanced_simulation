#!/usr/bin/python
# Following: https://answers.ros.org/question/253132/discrepancy-between-octomap-visualization-and-actual-octomap-in-moveit/

import rospy
import sys
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from octomap_msgs.msg import Octomap

class PlanningSceneUpdater:
	def __init__(self):
		# self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=5)
		self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=5)
		self.octomap_sub = rospy.Subscriber("/octomap_binary", Octomap, self.callback)
		rospy.loginfo("PlanningSceneUpdater started")
		
	def callback(self, msg):
		rospy.loginfo("PlanningSceneUpdater received map update")
		scene_update = PlanningScene()
		scene_update.is_diff = True
		scene_update.world.octomap.octomap = msg
		scene_update.world.octomap.header.stamp = rospy.Time.now()
		scene_update.world.octomap.header.frame_id = "world"
		
		
		self.scene_pub.publish(scene_update)
		rospy.loginfo("PlanningSceneUpdater published map update")
		
		
def main(args):
	rospy.init_node('planningSceneUpdater', anonymous=True)
	planningSceneUpdater = PlanningSceneUpdater()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("PlanningSceneUpdater shutting down")

if __name__ == '__main__':
	main(sys.argv)
