#!/usr/bin/env python3

import sys
import rospy

from vibro_tactile_toolbox.msg import TerminationSignal, TerminatorConfig

class TerminatorNode:
  """
  Node that listens to all termination handlers and will terminate the skill based on the termination config
  """
  def __init__(self):
    self.use_fts_handler = True
    self.use_timeout_handler = False
    self.use_robotstate_handler = False
    self.use_audio_handler = False
    self.use_vision_handler = False

    self.termination_pub = rospy.Publisher(
      "/terminator/skill_termination_signal", 
      TerminationSignal
    )

    self.termination_config_sub = rospy.Subscriber(
      "/terminator/termination_config", 
      TerminatorConfig,
      self.terminator_config_cb,
      queue_size=1
    )

    self.fts_handler_sub = rospy.Subscriber("/terminator/FTS_termination_signal", TerminationSignal, self.fts_handler_cb)
    self.timeout_handler_sub = None
    self.robotstate_handler_sub = None
    self.audio_handler_sub = None
    self.vision_handler_sub = None

  def terminator_config_cb(self, cfg: TerminatorConfig):
    raise NotImplementedError

  def fts_handler_cb(self, fts_termination_signal: TerminationSignal):
    if self.use_fts_handler:
      self.termination_pub.publish(fts_termination_signal)


def main(args):
  rospy.init_node('terminator_node', anonymous=True)
  terminator_node = TerminatorNode()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)