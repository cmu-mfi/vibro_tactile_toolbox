import rospy
from sensor_msgs.msg import CompressedImage, JointState
from sounddevice_ros.msg import AudioData, AudioInfo
from geometry_msgs.msg import WrenchStamped

def check_ros_topics(rostopic_list):
	all_ros_topics_running = True

	for rostopic in rostopic_list:
		if 'detector' in rostopic:
			pass
		elif 'terminator' in rostopic:
			pass
		elif 'compressed' in rostopic:
			try:
				rospy.wait_for_message(rostopic, CompressedImage, timeout=1)
			except:
				print(f'{rostopic} is not currently publishing.')
				all_ros_topics_running = False
		elif 'audio_info' in rostopic:
			try:
				rospy.wait_for_message(rostopic, AudioInfo, timeout=1)
			except:
				print(f'{rostopic} is not currently publishing.')
				all_ros_topics_running = False
		elif 'audio' in rostopic:
			try:
				rospy.wait_for_message(rostopic, AudioData, timeout=1)
			except:
				print(f'{rostopic} is not currently publishing.')
				all_ros_topics_running = False
		elif 'joint_states' in rostopic:
			try:
				rospy.wait_for_message(rostopic, JointState, timeout=1)
			except:
				print(f'{rostopic} is not currently publishing.')
				all_ros_topics_running = False
		elif 'fts' in rostopic:
			try:
				rospy.wait_for_message(rostopic, WrenchStamped, timeout=1)
			except:
				print(f'{rostopic} is not currently publishing.')
				all_ros_topics_running = False
	return all_ros_topics_running


def check_ros_services(rosservice_list):
	all_ros_services_running = True
	for rosservice in rosservice_list:
		try:
			rospy.wait_for_service(rosservice, timeout=1)
		except:
			print(f'{rosservice} is not currently running.')
			all_ros_services_running = False
	return all_ros_services_running