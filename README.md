An enhanced 3D simulation of franka emika's panda robot
=========
Based on the work of Erdal Pakel (https://github.com/erdalpekel/panda_simulation)

Install
----------

1. Docker
    1. Have a PC with a Nvidia GPU, docker and nvidia-docker2 installed
    2. Pull docker image from repository
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
    * /camera_bot/camera_bot/camera1/image_raw --> image stream from camera bot

* Inputs:
    * /panda_[arm|hand]_controller/follow_joint_trajectory --> Interface for a FollowJointTrajectory action client (example in src/gripper_close_open.py)
    * /move_group --> moveit interface

How to modify the simulation
-------
* Panda dynamics: franka_ros/franka_description/robots/
    * panda_arm.xacro: Mass, Inertia properties of each link (sofar best guesses by Erdal Pakel in https://erdalpekel.de/?p=55)
    * panda.transmission.xacro: The joint transmissions and interfaces
* Change the camera in enahnced_sim
    * model/camera_bot.gazebo.xacro: Camera intrinsics, topic name
    * model/camera_bot.xacro: Camera bots size, camera orientation on bot
    * launch/camera_and_panda.launch: Spawn position for the camera (node spawn_cam_urdf, args -x/y/z)