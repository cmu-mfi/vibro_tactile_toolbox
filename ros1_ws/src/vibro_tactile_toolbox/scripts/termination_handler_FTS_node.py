#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped, Wrench
from rospy.timer import TimerEvent

from terminator.terminator import Terminator

TERMINATOR_TIMER_PERIOD_NS = 1000 # nanoseconds
TERMINATOR_WRENCH_THRESH = Wrench()
TERMINATOR_WRENCH_THRESH.force.x = 30
TERMINATOR_WRENCH_THRESH.force.y = 30
TERMINATOR_WRENCH_THRESH.force.z = 30
TERMINATOR_WRENCH_THRESH.torque.x = 2
TERMINATOR_WRENCH_THRESH.torque.y = 2
TERMINATOR_WRENCH_THRESH.torque.z = 2




class TerminatorNode:

  def __init__(self):
    self.terminator = Terminator(TERMINATOR_WRENCH_THRESH)
    self.fts_wrench = None

    self.terminate_pub = rospy.Publisher("/terminator/FTS_termination_signal", Bool)

    self.fts_sub = rospy.Subscriber("/fts",WrenchStamped,self.fts_callback)

    self.termination_timer = rospy.Timer(rospy.Duration(0, TERMINATOR_TIMER_PERIOD_NS), self.termination_callback)

  def fts_callback(self, fts_wrench: WrenchStamped):
    self.fts_wrench = fts_wrench

  def termination_callback(self, event: TimerEvent):
    self.terminate_pub.publish(
      self.terminator.get_termination_signal(
        self.fts_wrench
      )
    )

def main(args):
  rospy.init_node('terminator_node', anonymous=True)
  terminator_node = TerminatorNode()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)