#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageCroppedRepublisher:

  def __init__(self):

    self.namespace = rospy.get_namespace()
    self.image_sub_topic_name = rospy.get_param('image_cropped_republisher/image_sub_topic_name')
    self.image_pub_topic_name = rospy.get_param('image_cropped_republisher/image_pub_topic_name')

    self.x_offset = rospy.get_param('image_cropped_republisher/x_offset')
    self.y_offset = rospy.get_param('image_cropped_republisher/y_offset')
    self.x_size = rospy.get_param('image_cropped_republisher/x_size')
    self.y_size = rospy.get_param('image_cropped_republisher/y_size')

    self.image_pub = rospy.Publisher("/" + self.namespace + self.image_pub_topic_name,Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/" + self.namespace + self.image_sub_topic_name,Image,self.callback, queue_size=10)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cropped_cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)[self.y_offset:self.y_offset+self.y_size,self.x_offset:self.x_offset+self.x_size]

    try:
      image_msg = self.bridge.cv2_to_imgmsg(cropped_cv_image, "bgr8")
      image_msg.header = data.header
      self.image_pub.publish(image_msg)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_cropped_republisher')
  ic = ImageCroppedRepublisher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
