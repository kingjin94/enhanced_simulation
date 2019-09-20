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
RUN apt-get update && apt-get install -y ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-server ros-melodic-octomap-rviz-plugins

# Additional moveit stuff
RUN apt-get install -y ros-melodic-moveit

RUN cd /home/catkin_ws/src && git clone git@github.com:kingjin94/filter_octomap.git && \
	git clone https://github.com/gt-ros-pkg/hrl-kdl.git && \
	cd /home/catkin_ws && /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /home/catkin_ws/ && catkin_make_isolated -j16 --force-cmake && \
	/home/catkin_ws/src/filter_octomap/makeMaps.sh"
	
## CUDA Stuff - base
# stacked base-runtime-devel-cudnn: https://gitlab.com/nvidia/container-images/cuda/tree/master/dist/ubuntu18.04/10.0
RUN apt-get update && apt-get install -y --no-install-recommends \
gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    #apt-get purge --autoremove -y curl && \
	rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION \
cuda-compat-10-0 && \
ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics,compat32 #added graphics for OpenGL in Gazebo etc. https://gitlab.com/nvidia/container-images/opengl/blob/ubuntu18.04/base/Dockerfile
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=410,driver<411"

# runtime
ENV NCCL_VERSION 2.4.8

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-$CUDA_PKG_VERSION \
cuda-nvtx-$CUDA_PKG_VERSION \
libnccl2=$NCCL_VERSION-1+cuda10.0 && \
    apt-mark hold libnccl2 && \
    rm -rf /var/lib/apt/lists/*

# devel
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
cuda-libraries-dev-$CUDA_PKG_VERSION \
        cuda-minimal-build-$CUDA_PKG_VERSION \
        libnccl-dev=$NCCL_VERSION-1+cuda10.0 \
&& \
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

# libcudnn
ENV CUDNN_VERSION 7.6.3.30
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 \
&& \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

# END CUDA stuff

# Get Mask R-CNN
RUN pip install ipython keras==2.0.8 numpy scikit-image scipy pillow cython h5py tensorflow-gpu && \
	cd /home/catkin_ws/src/ && git clone git@github.com:kingjin94/mask_rcnn_ros.git
# END Mask R-CNN

# Fix /root/.panda_simulation does not exist error
RUN mkdir /root/.panda_simulation

# For display and ros source
RUN echo "# For ROS" >> /root/.bashrc && \
echo "source /home/catkin_ws/devel_isolated/setup.bash" >> /root/.bashrc && \
echo "# Fix Display" >> /root/.bashrc && \
echo "export DISPLAY=\":0\"" >> /root/.bashrc

RUN git config --global user.name "Matthias Mayer" && git config --global user.email "matthias.mayer@tum.de"

# Update all gits make them
RUN sh -c "cd /home/catkin_ws/src && find . -mindepth 1 -maxdepth 1 -type d -print -exec git -C {} pull \;" && \
	/bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /home/catkin_ws/ && catkin_make_isolated -j16 --force-cmake"

# Fix "[REST.cc:205] Error in REST request" with https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo
# RUN cd /root/.ignition/fuel/ && sed -i -e 's/url: https:\/\/api.ignitionfuel.org/url: https:\/\/api.ignitionrobotics.org/g' config.yaml
