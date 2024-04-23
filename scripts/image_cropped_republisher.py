import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

x_offset = 200
y_offset = 1950


class ImageCroppedRepublisher:

  def __init__(self):
    self.image_pub = rospy.Publisher("/side_camera/color/image_cropped",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/side_camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cropped_cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)[y_offset:y_offset+720,x_offset:x_offset+1280]

    try:
      image_msg = self.bridge.cv2_to_imgmsg(cropped_cv_image, "bgr8")
      image_msg.header = data.header
      self.image_pub.publish(image_msg)
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = ImageCroppedRepublisher()
  rospy.init_node('image_cropped_republisher', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
