An enhanced 3D simulation of franka emika's panda robot
=========
Based on the work of Erdal Pakel (https://github.com/erdalpekel/panda_simulation)

Install
----------

1. Docker
    1. Have a PC with a Nvidia GPU, docker and nvidia-docker2 installed
    2. Change $HOME/.ssh/id_rsa to your private ssh key for github
    3. Build docker image with install/build_all.sh
    4. Use commands in "install/commands_for_docker" to get the container running with X11 passthrough
2. Install ros melodic full desktop, than follow the steps from the Dockerfile in install/

First start-up
----------

1. Clone this repository to your catkin workspace's src (in Docker /home/catkin_ws/src)
2. Make catkin workspace
3. source devel/setup.bash
4. Launch simulation with `roslaunch enhanced_sim camera_and_panda.launch

Run with the real robot
----------
One needs the Panda robot by Franka Emika with a FCI license and a Intel RealSense D435 to run this demo. Drivers for the Intel camera must be installed on the PC the camera is connected to. FCI is assumed to be running on a dedicated second PC running a real-time capable Linux.

Now build the docker container under install/realRobot/ or install all the packages and their dependencies in your main PC's catkin_ws. If FCI runs on a second PC it needs panda_moveit_control_only installed locally and the ROS network must be established between the main PC and the FCI host (run the docker container with --net=host and --priviledged to interface the other PC and realsense camera via USB). Then the demo should be startable with the lauchscripts real_robot_explorer/setupExplorer.launch and real_robot_explorer/randomExplorer.launch. You may want to callibrate the camera <-> hand offset with real_robot_explorer/calibrate.launch beforehand. Also make sure that the main PC can ssh into the one running FCI such that the local controller from panda_moveit_control_only can be restarted remotely.

Related Packages
------
For the pure simulation of the robot the following packages were adapted; refer to the individual repositories to find where they were forked from:

* https://github.com/kingjin94/gazebo_ros_pkgs
* https://github.com/kingjin94/panda_simulation
* https://github.com/kingjin94/octomap_mapping
* https://github.com/kingjin94/panda_moveit_config
* https://github.com/kingjin94/mask_rcnn_ros
* https://github.com/kingjin94/franka_ros
* https://github.com/kingjin94/realtime_urdf_filter
* https://github.com/kingjin94/filter_octomap

Additionaly, for use with the real Panda robot the following packages are needed

* https://github.com/kingjin94/real_robot_explorer (to be installed on the PC that rund the simulation)
* https://github.com/kingjin94/panda_moveit_control_only, based on panda_moveit_config by Franka Emika (to be installed on the PC interacting with Panda via FCI)

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
