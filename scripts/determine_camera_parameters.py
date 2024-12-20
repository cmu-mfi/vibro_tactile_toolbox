import cv2

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

ROTATION = 0

center_x, center_y = 0,0

original_size = (0,0)
current_size = (1280.720)

class Button:
    def __init__(self, text, x, y, width, height, command=None, command2=None):
        self.text = text
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.command = command
        self.command2 = command2


    def handle_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.x <= x <= self.x + self.width and self.y <= y <= self.y + self.height:
                if self.command is not None:
                    self.command()
            elif self.command2 is not None:
                self.command2(x,y)

    def draw(self, frame):
        cv2.rectangle(frame, (self.x, self.y), (self.x + self.width, self.y + self.height), (255, 0, 0), 2)
        cv2.putText(frame, self.text, (self.x + 10, self.y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def on_button_click():
    global ROTATION
    ROTATION = (ROTATION + 90) % 360

def on_button_click2(x,y):
    global center_x, center_y
    global original_size
    global current_size
    global VERBOSE
    x_ratio = original_size[0] / current_size[1]
    y_ratio = original_size[1] / current_size[0]
   
    center_x, center_y = round(x_ratio * x), round(y_ratio * y)
    if VERBOSE:
      print(original_size)
      print(current_size)
      print(x_ratio)
      print(y_ratio)
      print(center_x, center_y)

def main(args):
  global ROTATION
  global center_x, center_y
  global original_size
  global current_size
  # Create a button
  button = Button("Rotate", 100, 100, 100, 50, on_button_click, on_button_click2)

  # Create a window
  cv2.namedWindow("RAW IMAGE")

  # Create a window
  cv2.namedWindow("CROPPED IMAGE")

  # Set mouse callback
  cv2.setMouseCallback("RAW IMAGE", button.handle_event)
  image_msg = rospy.wait_for_message(args[1] + '/side_camera/color/image_raw/compressed',CompressedImage)

  np_arr = np.frombuffer(image_msg.data, np.uint8)
  image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  image_size = image_np.shape[:2]
  center_x = int(image_size[1]/2)
  center_y = int(image_size[0]/2)

  while not rospy.is_shutdown():
    
    y_offset = center_y-360
    y_size = 720
    x_offset = center_x-640
    x_size = 1280

    if ROTATION == 90:
      original_size = (image_size[1], image_size[0])
      current_size = (720,1280)
      vis_image = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)[y_offset:y_offset+y_size,x_offset:x_offset+x_size]
    elif ROTATION == 180:
      original_size = (image_size[0], image_size[1])
      current_size = (1280,720)
      vis_image = cv2.rotate(image_np, cv2.ROTATE_180)
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_180)[y_offset:y_offset+y_size,x_offset:x_offset+x_size]
    elif ROTATION == 270:
      original_size = (image_size[1], image_size[0])
      current_size = (720,1280)
      vis_image = cv2.rotate(image_np, cv2.ROTATE_90_COUNTERCLOCKWISE)
      cropped_cv_image = cv2.rotate(image_np, cv2.ROTATE_90_COUNTERCLOCKWISE)[y_offset:y_offset+y_size,x_offset:x_offset+x_size]
    else:
      original_size = (image_size[0], image_size[1])
      current_size = (1280,720)
      vis_image = image_np
      cropped_cv_image = image_np[y_offset:y_offset+y_size,x_offset:x_offset+x_size]

    small_image = cv2.resize(vis_image, current_size)

    button.draw(small_image)
    cv2.imshow("RAW IMAGE", small_image)

    cv2.imshow("CROPPED IMAGE", cropped_cv_image)

    if cv2.waitKey(1) == ord('q'):
        print(f'<param name="x_offset" value="{str(x_offset)}"/>')
        print(f'<param name="y_offset" value="{str(y_offset)}"/>')
        print(f'<param name="x_size" value="{str(x_size)}"/>')
        print(f'<param name="y_size" value="{str(y_size)}"/>')
        print(f'<param name="rotation" value="{str(ROTATION)}"/>')
        break
    
  cv2.destroyAllWindows()

if __name__ == '__main__':
    
    rospy.init_node('camera_parameters')
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)


