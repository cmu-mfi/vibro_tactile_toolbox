#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped, Wrench
from rospy.timer import TimerEvent

class TerminatorNode:
  """
  Node that listens to all termination handlers and will terminate the skill based on the termination config
  """
  def __init__(self):
    self.termination_config = None

    self.termination_pub = rospy.Publisher("/terminator/skill_termination_signal", Bool)

    self.termination_config_sub = rospy.Subscriber("/terminator/termination_config", None, None)

    self.fts_handler_sub = rospy.Subscriber("/terminator/FTS_termination_signal",WrenchStamped,self.fts_callback)
    self.timeout_handler_sub = None
    self.robotstate_handler_sub = None
    self.audio_handler_sub = None
    self.vision_handler_sub = None

  def fts_handler_cb(self, fts_termination_signal: TerminationSignal):
    pass


def main(args):
  rospy.init_node('terminator_node', anonymous=True)
  terminator_node = TerminatorNode()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)