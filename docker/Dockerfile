FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-c"]

# Basic apt-get
RUN apt-get update
RUN apt-get install -y git \
    && apt-get install -y python3 \
    && apt-get install -y python3-pip

# Clone vibro_tactile_toolbox repo
RUN git clone --branch docker-image https://github.com/cmu-mfi/vibro_tactile_toolbox.git \
    && cd /vibro_tactile_toolbox \
    && git pull

# Install pip dependencies
RUN cd /vibro_tactile_toolbox/docker \
    && pip install -r requirements.txt

# Add external packages into worskpace

## Audio (https://github.com/firephinx/sounddevice_ros)
RUN apt-get install -y libportaudio2 \
    && apt-get install -y libasound-dev \
    && cd /vibro_tactile_toolbox/ros1_ws/src \ 
    && git clone https://github.com/firephinx/sounddevice_ros.git

## Robot controller

## Cameras
RUN apt-get install -y libgflags-dev \
    && apt-get install -y ros-$ROS_DISTRO-image-geometry \
    && apt-get install -y ros-$ROS_DISTRO-camera-info-manager \
    && apt-get install -y ros-$ROS_DISTRO-image-transport \
    && apt-get install -y ros-$ROS_DISTRO-image-publisher \
    && apt-get install -y libgoogle-glog-dev \
    && apt-get install -y libusb-1.0-0-dev \
    && apt-get install -y libeigen3-dev  \
    && cd /vibro_tactile_toolbox/ros1_ws/src \ 
    && git clone https://github.com/orbbec/OrbbecSDK_ROS1.git

## FTS

# Build and install the ROS workspace
RUN source /opt/ros/noetic/setup.bash \
 && cd /vibro_tactile_toolbox/ros1_ws \
 && catkin_make install

# Add sourcing to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /vibro_tactile_toolbox/ros1_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /vibro_tactile_toolbox/ros1_ws/install/setup.bash" >> ~/.bashrc

RUN echo "All Done "
