FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-c"]

# Basic apt-get
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip

# Clone vibro_tactile_toolbox repo
RUN cd ~/ \
    && mkdir -p /ros1_ws 

COPY ros1_ws /ros1_ws

# Build and install the ROS workspace
RUN source /opt/ros/noetic/setup.bash \
 && cd /ros1_ws \
 && catkin_make install

# Add sourcing to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ros1_ws/install/setup.bash" >> ~/.bashrc

# RUN pip install -r requirements.txt

RUN echo "All Done "
