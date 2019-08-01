FROM ros:melodic-desktop-full

# Steps for franka sim by https://erdalpekel.de/?p=55
# Install libfranka from source https://frankaemika.github.io/docs/install_linux.html
RUN cd /home && apt update && \
	apt install build-essential cmake git libpoco-dev libeigen3-dev && \
	git clone --recursive https://github.com/frankaemika/libfranka && \
	cd libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && \
	cmake --build . -- -j16 # allow make to multithread
	
RUN apt install -y ssh
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/ && \
    echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
	chmod 600 /root/.ssh/id_rsa && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts


# Add to end of /root/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc


# panda simulation
RUN cd /home && mkdir -p catkin_ws/src && cd catkin_ws/src && \
	git clone ssh://git@github.com/kingjin94/enhanced_simulation.git && \
	git clone ssh://git@github.com/kingjin94/panda_simulation.git && \
	git clone ssh://git@github.com/kingjin94/panda_moveit_config.git && \
	git clone --branch simulation ssh://git@github.com/kingjin94/franka_ros && \
	# Not yet ready for primetime git clone --branch melodic-devel ssh://git@github.com/kingjin94/gazebo_ros_pkgs.git && \
	cd /home/catkin_ws && apt install libboost-filesystem-dev && \
	rosdep install --from-paths src --ignore-src -y --skip-keys libfranka --skip-keys moveit_perception

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /home/catkin_ws/ && \
	catkin_make_isolated -j16 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/libfranka/build"
	
# urdf-filter
RUN cd /home/catkin_ws/src && git clone git@github.com:kingjin94/realtime_urdf_filter.git && \
	apt-get install -y glew-utils mesa-utils python-pip && cd .. && pip install scipy scikit-image && \
	/bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /home/catkin_ws/ && catkin_make_isolated -j16"

# get octomap
RUN apt-get update && apt-get install -y ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-server

# Additional moveit stuff
RUN apt-get install -y ros-melodic-moveit

# Fix /root/.panda_simulation does not exist error
RUN mkdir /root/.panda_simulation
