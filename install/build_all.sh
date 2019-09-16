cd docker_images/ros/melodic/ubuntu/bionic
cd ros-core
sudo docker build -t ros:melodic-ros-core-bionic .
cd ../ros-base
sudo docker build -t ros:melodic-ros-base-bionic .
cd ../robot
sudo docker build -t ros:melodic-robot-bionic .
cd ../desktop
sudo docker build -t osrf/ros:melodic-desktop-bionic .
cd ../desktop-full
sudo docker build -t ros:melodic-desktop-full .
cd ../../../../../../
sudo docker build --build-arg SSH_PRIVATE_KEY="$(cat $HOME/.ssh/id_rsa)" -t panda_sim .
