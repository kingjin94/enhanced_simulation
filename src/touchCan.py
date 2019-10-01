from filter_octomap.msg import table, cans
import rospy
import numpy as np

import touchTable2

class CanRefiner(touchTable2.TactileRefiner):
	def __init__(self, group_name = "panda_arm"):
		super(CanRefiner, self).__init__(group_name)
		
		# Publisher for refined table
		self.can_publisher = rospy.Publisher("/octomap_new/cans_touched", cans, queue_size=1, latch=True)
		
		print("Waiting for can message ...")
		self.cans_msg = rospy.wait_for_message("/octomap_new/cans", cans)
		
		# Select highest scoring can to refine
		scores = [can.score for can in self.cans_msg.cans]
		self.can_msg = self.cans_msg.cans[np.argmax(scores)]
		print("Concentrating on can: \n{}".format(self.can_msg))
		
		self.touched_points_mantle = []
		self.touched_points_top = []
		
		self.max_reach = 1.0 # Maximum distance from 0,0,0 the robot might be able to reach

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
	rospy.spin()
