#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

VERBOSE=False

class CompressedImageCroppedRepublisher:

  def __init__(self):

    self.namespace = rospy.get_namespace()
    self.compressed_image_sub_topic_name = rospy.get_param('compressed_image_cropped_republisher/compressed_image_sub_topic_name')
    self.compressed_image_pub_topic_name = rospy.get_param('compressed_image_cropped_republisher/compressed_image_pub_topic_name')

    self.x_offset = rospy.get_param('compressed_image_cropped_republisher/x_offset')
    self.y_offset = rospy.get_param('compressed_image_cropped_republisher/y_offset')
    self.x_size = rospy.get_param('compressed_image_cropped_republisher/x_size')
    self.y_size = rospy.get_param('compressed_image_cropped_republisher/y_size')
    self.rotation_direction = rospy.get_param('compressed_image_cropped_republisher/rotation_direction')

    self.compressed_image_pub = rospy.Publisher(self.namespace + self.compressed_image_pub_topic_name,CompressedImage, queue_size=1)

    self.compressed_image_sub = rospy.Subscriber(self.namespace + self.compressed_image_sub_topic_name,CompressedImage,self.callback, queue_size=1)
    if VERBOSE :
            print("subscribed to "+ self.namespace + self.compressed_image_sub_topic_name)

  def callback(self,ros_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    if VERBOSE :
        print('received image of type: ' + ros_data.format)

    #### direct conversion to CV2 ####
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    if self.rotation_direction == 'clockwise':
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)[self.y_offset:self.y_offset+self.y_size,self.x_offset:self.x_offset+self.x_size]
    elif self.rotation_direction == 'counter_clockwise':
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_90_COUNTERCLOCKWISE)[self.y_offset:self.y_offset+self.y_size,self.x_offset:self.x_offset+self.x_size]
    elif self.rotation_direction == '180':
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_180)[self.y_offset:self.y_offset+self.y_size,self.x_offset:self.x_offset+self.x_size]
    else:
      cropped_cv_image = image_np[self.y_offset:self.y_offset+self.y_size,self.x_offset:self.x_offset+self.x_size]


    compressed_image_msg = CompressedImage()
    compressed_image_msg.header = ros_data.header
    compressed_image_msg.format = "jpeg"
    compressed_image_msg.data = np.array(cv2.imencode('.jpg', cropped_cv_image)[1]).tobytes()
    # Publish new image
    self.compressed_image_pub.publish(compressed_image_msg)

def main(args):
  rospy.init_node('compressed_image_cropped_republisher')
  cicr = CompressedImageCroppedRepublisher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)