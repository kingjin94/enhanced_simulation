FROM ros:melodic-desktop-full

# Add to end of /root/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Steps for franka sim by https://erdalpekel.de/?p=55
# Install libfranka from source https://frankaemika.github.io/docs/install_linux.html
RUN cd /home && apt update && \
	apt install build-essential cmake git libpoco-dev libeigen3-dev && \
	git clone --recursive https://github.com/frankaemika/libfranka && \
	cd libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && \
	cmake --build . -- -j16 # allow make to multithread


# panda simulation
RUN cd /home && mkdir -p catkin_ws/src && cd catkin_ws/src && \
	git clone https://github.com/erdalpekel/panda_simulation.git && \
	git clone https://github.com/kingjin94/panda_moveit_config.git && \
	git clone --branch simulation https://github.com/kingjin94/franka_ros && \
	cd /home/catkin_ws && apt install libboost-filesystem-dev && \
	rosdep install --from-paths src --ignore-src -y --skip-keys libfranka --skip-keys moveit_perception

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /home/catkin_ws/ && \
	catkin_make -j16 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/libfranka/build"
