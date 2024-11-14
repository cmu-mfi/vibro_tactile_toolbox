docker build -t nvidia_ros:noetic -f Dockerfile.nvidia_ros .
docker build -t ros_desktop_full:noetic -f Dockerfile.ros_desktop_full .
docker build -t realsense_ros:noetic -f Dockerfile.realsense_ros .
docker build -t sounddevice_ros:noetic -f Dockerfile.sounddevice_ros .
docker build -t orbbec_ros:noetic -f Dockerfile.orbbec_ros .
docker build -t robotiq_mm_ros:noetic -f Dockerfile.robotiq_mm_ros .
docker build -t vibro_tactile_toolbox_prereq:noetic -f Dockerfile.vibro_tactile_toolbox_prereq .