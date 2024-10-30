FROM ros:noetic
ENV TZ=ET
SHELL ["/bin/bash", "-c"]

# Basic apt-get
RUN apt-get update
RUN apt-get install -y git vim curl \
    && apt-get install -y python3 \
    && apt-get install -y python3-pip

# Install Pyserial
RUN pip install pyserial

# Install ROS Noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y ros-noetic-desktop-full \
    && apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep update


# Create a Ros1 ws
RUN cd / && mkdir -p ros1_ws/src

# Install catkin tools
RUN apt-get -y install python3-catkin-tools

# Clone robotiq_mm_ros repo
RUN cd /ros1_ws/src \
    && git clone https://github.com/cmu-mfi/robotiq_mm_ros.git

RUN source /opt/ros/noetic/setup.bash \
 && cd /ros1_ws \
 && catkin build

# Add sourcing to bashrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ros1_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "All Done "