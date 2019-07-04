An enhanced 3D simulation of franka emika's panda robot
=========
Based on the work of Erdal Pakel (https://github.com/erdalpekel/panda_simulation)

Install
----------

1. Docker
    1. Have a PC with a Nvidia GPU, docker and nvidia-docker2 installed
    2. Build docker image with install/build_all.sh
    3. Use commands in "install/commands_for_docker" to get the container running with X11 passthrough
2. Install ros melodic full desktop, than follow steps from Dockerfile in install/

First start-up
----------

1. Clone this repository to your catkin workspace's src (in Docker /home/catkin_ws/src)
2. Make catkin workspace
3. source devel/setup.bash
4. Launch simulation with `roslaunch enhanced_sim camera_and_panda.launch`

Available topics
-------

* Outputs:
    * /joint_states --> q, q_dot, torque
    * /gazebo/link_states --> pose, twist of all links
    * /tf --> transforamtions along kinematic chain
    * /panda_[arm|hand]_controller/state --> actual, desired and error for q, qd, qdd, torque
    * /camera_bot/camera_bot/image_raw --> color image stream from camera bot
    * /camera_bot/depth_camera/[depth_image|points] --> depth image and point cloud
    * /panda/bumper/colliding --> overview if and where collisions occur
    * /panda/bumper/panda_* --> detailed collision state per linke (where, wrench, normal, counterpart)

* Inputs:
    * /panda_[arm|hand]_controller/follow_joint_trajectory --> Interface for a FollowJointTrajectory action client (example in src/gripper_close_open.py)
    * /move_group --> moveit interface

How to modify the simulation
-------
* Panda dynamics: franka_ros/franka_description/robots/
    * panda_arm.xacro: Mass, Inertia properties of each link (sofar best guesses by Erdal Pakel in https://erdalpekel.de/?p=55)
    * panda.transmission.xacro: The joint transmissions and interfaces
* Change the camera in enhanced_sim
    * model/camera_bot.gazebo.xacro: Camera intrinsics, topic names
    * launch/camera_and_panda.launch: Spawn position for the camera (node spawn_cam_urdf, args -x/y/z)
    * Position and orientation via roslaunch arguments, e.g. roslaunch enhanced_sim camera_and_panda.launch x:=-0.5 y:=0 height:=2 Y:=0 P:=1.51
	    * x,y,height: Position
	    * Y: Yaw angle, orientation in ground plane in rad
	    * P: Look down (+) or up (-), in rad
	   
	   
Open issues with workaround
--------
* Gazebo or RViz may not open every time one tries roslaunch; retry roslaunch
