from filter_octomap.msg import table
import numpy as np
import random
import rospy
import moveit_commander
import geometry_msgs.msg
import sys

# For sending joint level controlls
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# For kinematic calculations
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import tf

# For bumper
from enhanced_sim.msg import CollisionState
from gazebo_msgs.msg import ContactsState

def go_to_q(q):
	client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	client.wait_for_server()
	goal = FollowJointTrajectoryGoal()
	joint_traj = JointTrajectory()
	joint_traj.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
	point1 = JointTrajectoryPoint()
	point1.positions = q.tolist()
	point1.velocities = [0., 0., 0., 0., 0., 0., 0.]
	point1.time_from_start = rospy.Duration(1.)
	joint_traj.points = [point1]
	goal.trajectory = joint_traj
	joint_traj.header.stamp = rospy.Time.now()+rospy.Duration(1.0)
	client.send_goal_and_wait(goal)

def touch_and_refine_table(robot, scene, move_group):
	urdf_robot = URDF.from_parameter_server()
	kdl_kin = KDLKinematics(urdf_robot, urdf_robot.links[1].name, urdf_robot.links[8].name)

	
	move_group.set_max_velocity_scaling_factor(0.1) # Allow 10 % of the set maximum joint velocities
	move_group.set_max_acceleration_scaling_factor(0.1)
	
	# Receive table message
	the_table = rospy.wait_for_message("/octomap_new/table", table)

	touchPoses = []

	for i in range(10):
		ret = False
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.z = the_table.centroid_position.z + 0.25
		pose_goal.orientation.x = 1 # tool pointing down
		pose_goal.orientation.y = 0
		pose_goal.orientation.z = 0
		pose_goal.orientation.w = 0

		# select random point above table and go there
		# if not successfull try another point
		while not ret:
			pose_goal.position.x = random.uniform(the_table.min.x+.1, the_table.max.x-.1)
			pose_goal.position.y = random.uniform(the_table.min.y+.1, the_table.max.y-.1)
			
			print("Trying: {}", pose_goal.position)
			
			if pose_goal.position.x**2+pose_goal.position.y**2 < 0.7**2:
				move_group.set_pose_target(pose_goal)
				ret = move_group.go()
			else:
				print("Probably to far")
		
		print("Arrived successfully")
		# move towards the table in a straight line (along end-effector's z-axis)
		ret = True
		rot_d = np.asarray((1,0,0,0))
		pos_d = np.asarray((pose_goal.position.x,pose_goal.position.y,pose_goal.position.z))
		old_e = np.zeros(6)
		sum_e = np.zeros(6)
		while True:  # while not touched later!!!
			pose_c = move_group.get_current_pose()
			pos_c = np.asarray((pose_c.pose.position.x, pose_c.pose.position.y, pose_c.pose.position.z))
			rot_c = np.asarray((pose_c.pose.orientation.x, pose_c.pose.orientation.y, pose_c.pose.orientation.z, pose_c.pose.orientation.w))
			if rot_c[0] < 0:
				rot_c = -1.*rot_c
			
			pos_d[2] -= 0.005
			
			print("Desired:")
			print(np.concatenate([pos_d, rot_d]))
			print("Actual:")
			print(np.concatenate([pos_c, rot_c]))
			
			pos_e = pos_d - pos_c
			rot_e = rot_d - rot_c
			rot_e = np.asarray(tf.transformations.euler_from_quaternion(rot_e)) / 1000 #(get into same ballpark pi of rot to 0.001 m of the position)
			e = np.concatenate([pos_e, rot_e])
			de = e-old_e
			old_e = e
			sum_e += e
			
			print("Error:")
			print(e)
			print("dError:")
			print(de)
			print("dsumError:")
			print(sum_e)
			
			q = np.asarray(robot.get_current_state().joint_state.position[:7])
			J = np.asarray(kdl_kin.jacobian(q))

			q_new = q+J.T.dot(e+0.1*de+0.01*sum_e) #[0,0,-1,0,0,0]
			go_to_q(q_new)
			
			touchSensorMsg = rospy.wait_for_message("/panda/bumper/colliding", CollisionState)
			if touchSensorMsg.colliding:
				#print(touchSensorMsg)
				break
			
		print("Touched the surface")
		rospy.sleep(2.)
		ballSensorMsg = ContactsState()
		while ballSensorMsg.states == []:
			ballSensorMsg = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)
		#print(ballSensorMsg)
		touchPoses.append(ballSensorMsg)
		
		print("Recording done, retracting ...")
		
		q = np.asarray(robot.get_current_state().joint_state.position[:7])
		J = np.asarray(kdl_kin.jacobian(q))

		q_new = q+J.T.dot([0,0,0.05,0,0,0])
		go_to_q(q_new)
		#move_group.shift_pose_target(2, +0.05)
		ret = move_group.go()
		print("Retracted")

		# note eef position when collided, e.g. by listening to /panda/panda/colliding; in real probably ask libfranka
		
		
	# fit table surface with new information from the touched points and display info
	#print("Touched surface at the following poses:")
	#print(touchPoses)
	print("Improved surface estimate:")
	touch_pts = np.zeros((3,10))
	touch_pts[0,:] = [touchPoses[i].states[0].contact_positions[0].x for i in range(10)]
	touch_pts[1,:] = [touchPoses[i].states[0].contact_positions[0].y for i in range(10)]
	touch_pts[2,:] = [touchPoses[i].states[0].contact_positions[0].z for i in range(10)]

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

# Notes on getting jacobian etc.
# Install pykdl_utils by checking out git into catkin_ws and catkin_make_isolated


if __name__ == '__main__':
	rospy.init_node('touch_table', anonymous=True)

	moveit_commander.roscpp_initialize(sys.argv)

	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group_name = "panda_arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)

	touch_and_refine_table(robot, scene, move_group)
