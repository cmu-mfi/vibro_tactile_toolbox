#!/usr/bin/env python3

import sys
import rospy
import json

from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

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
      TerminationConfig,
      self.terminator_config_cb,
      queue_size=1
    )

    self.fts_handler_sub = rospy.Subscriber("/terminator/FTS_termination_signal", TerminationSignal, self.fts_handler_cb)
    self.timeout_handler_sub = None
    self.robotstate_handler_sub = None
    self.audio_handler_sub = None
    self.vision_handler_sub = None

  def terminator_config_cb(self, cfg: TerminationConfig):
    cfg_jsons = cfg.cfg_json

    print(f"==Received Termination Config==\n{cfg_jsons}")

    cfg_json = json.loads(cfg_jsons)
    self.use_fts_handler = 'FTS' in cfg_json
    self.use_timeout_handler = 'timeout' in cfg_json
    self.use_robotstate_handler = 'robot' in cfg_json
    self.use_audio_handler = 'audio' in cfg_json
    self.use_vision_handler = 'vision' in cfg_json

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