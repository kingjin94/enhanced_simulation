FROM panda_sim5

RUN apt-get install -y iputils-ping net-tools usbutils # For network & usb debug

# Have kernel 5.0.0-31-generic --> need to compile realsense sdk from scratch
# Source: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
# Make sure own hostname is in /etc/hosts s.t. copied into the container!
RUN cd /home && \
	apt-get update && \
	apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
	apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev sudo && \ 
	#apt-get install -y linux-headers-$(uname -r) kmod linux-image-$(uname -r) && \
	git clone https://github.com/IntelRealSense/librealsense.git && \
	cd librealsense && \
	#./scripts/setup_udev_rules.sh && \
	#./scripts/patch-realsense-ubuntu-lts.sh && \
	mkdir build && cd build && \
	cmake -DCMAKE_BUILD_TYPE=Release ../ && \
	make uninstall && make clean && make -j 16 && make install

# Install ROS part 
RUN cd /home/catkin_ws/src && \
	git clone https://github.com/IntelRealSense/realsense-ros.git && \
	git clone git@github.com:pal-robotics/ddynamic_reconfigure.git && \
	cd realsense-ros/ && \
	git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1` && \
	cd /home/catkin_ws && \
	catkin_make_isolated
