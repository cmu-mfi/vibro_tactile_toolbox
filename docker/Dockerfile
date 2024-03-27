FROM nvidia/cuda:12.3.2-cudnn9-devel-ubuntu20.04
ENV TZ=ET
SHELL ["/bin/bash", "-c"]

# Basic apt-get
RUN apt-get update
RUN apt-get install -y git vim curl \
    && apt-get install -y python3 \
    && apt-get install -y python3-pip

# Install Pytorch
RUN pip install torch torchvision torchaudio

# Install ROS Noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y ros-noetic-desktop-full \
    && apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init && rosdep update

# Install Other Dependencies in Documents folder
RUN mkdir Documents

# Clone detectron2
RUN cd /Documents \
    && git clone https://github.com/facebookresearch/detectron2.git \
    && cd detectron2 \
    && pip install -e .

# Clone Segment Anything 
RUN cd /Documents \
    && git clone https://github.com/facebookresearch/segment-anything.git \
    && cd segment-anything \
    && pip install -e .

# Clone Segment Anything Labeling Tool
RUN cd /Documents \
    && git clone https://github.com/cmu-mfi/salt.git \
    && cp salt/helpers/* segment-anything/

# Create a Ros1 ws
RUN cd / && mkdir -p ros1_ws/src

# Install catkin tools
RUN apt-get -y install python3-catkin-tools

# Clone vibro_tactile_toolbox repo
RUN cd /ros1_ws/src \
    && git clone https://github.com/cmu-mfi/vibro_tactile_toolbox.git \
    && cd vibro_tactile_toolbox \
    && pip install -r requirements.txt

# Audio (https://github.com/firephinx/sounddevice_ros)
RUN cd /ros1_ws/src \ 
    && git clone https://github.com/firephinx/sounddevice_ros.git

RUN source /opt/ros/noetic/setup.bash \
 && cd /ros1_ws \
 && catkin build

# Add sourcing to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /vibro_tactile_toolbox/ros1_ws/install/setup.bash" >> ~/.bashrc

RUN echo "All Done "