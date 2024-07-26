import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/side_camera/color/image_cropped"
    # Set up your subscriber and define its callback
    img_msg = rospy.wait_for_message(image_topic, Image)
    cv2_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv2.imwrite('camera_image.jpg', cv2_image)

if __name__ == '__main__':
    main()