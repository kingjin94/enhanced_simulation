#!/usr/bin/python

import rospy
import touchTable2
import touchCan
import visualExplorer

rospy.init_node('random_explorer', anonymous=True)

try:
	print("Start exploring")
	#random_walk(move_group, robot)
	visExp = visualExplorer.VisualExplorer(entropy_threshold=-0.5, table_threshold=500000, min_dist_lookouts=0.4)
	visExp.run()
	print("Visual done")
	# Kill all vision nodes; make sure important topics are latched, esp. table
	
	# Refine table
	refinerTable = touchTable2.TableRefiner()
	refinerTable.touch_and_refine_table(touchPts=4)
	print("Table done")
	
	# Refine cans
	refinerCans = touchCan.CanRefiner()
	refinerCans.touch_and_refine_can()
	print("Can done")
	
	rospy.spin() # Keep all the messages alive
except KeyboardInterrupt:
	rospy.loginfo("random_explorer shutting down")


