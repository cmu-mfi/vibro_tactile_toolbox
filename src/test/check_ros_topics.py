import rospy
from sensor_msgs.msg import CompressedImage, JointState
from sounddevice_ros.msg import AudioData, AudioInfo
from geometry_msgs.msg import WrenchStamped

def check_ros_topics(rostopic_list):
	for rostopic in rostopic_list:
		if 'detector' in rostopic:
			pass
		elif 'compressed' in rostopic:
			rospy.wait_for_message(rostopic, CompressedImage, timeout=5)
		elif 'audio_info' in rostopic:
			rospy.wait_for_message(rostopic, AudioInfo, timeout=5)
		elif 'audio' in rostopic:
			rospy.wait_for_message(rostopic, AudioData, timeout=5)
		elif 'joint_states' in rostopic:
			rospy.wait_for_message(rostopic, JointState, timeout=5)
		elif 'fts' in rostopic:
			rospy.wait_for_message(rostopic, WrenchStamped, timeout=5)


def check_ros_services(rosservice_list):
	for rosservice in rosservice_list:
		rospy.wait_for_service(rosservice, timeout=5)