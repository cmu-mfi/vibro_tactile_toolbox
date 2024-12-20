cmake_minimum_required(VERSION 3.0.2)
project(vibro_tactile_toolbox)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
) 

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AudioOutcomeRepub.msg
  BoundingBox.msg
  SkillParams.msg
  TerminationConfig.msg
  TerminationSignal.msg
  VisionOutcomeRepub.msg
  FtsOutcomeRepub.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  AudioOutcome.srv
  FTSOutcome.srv
  LegoOutcome.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES vibro_tactile_toolbox
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
scripts/audio_detector.py
  scripts/audio_outcome_republisher.py
  scripts/collect_lego_audio_data.py
  scripts/collect_nist_audio_data.py
  scripts/compressed_image_cropped_republisher.py
  scripts/compressed_lego_detector.py
  scripts/determine_camera_parameters.py
  scripts/fts_detector.py
  scripts/image_cropped_republisher.py
  scripts/lego_detector.py
  scripts/pose_stamped_publisher.py
  scripts/reset_joints.py
  scripts/save_lego_pose.py
  scripts/save_hande_pose.py
  scripts/termination_handler_node.py
  scripts/terminator_node.py
  scripts/test_lego_audio_outcome.py
  scripts/test_nist_audio_outcome.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
