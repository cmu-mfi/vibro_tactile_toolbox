#!/usr/bin/env python3

import rospy
import json
import sys

from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

class TerminatorNode:
  """
  Node that listens to all termination handlers and will terminate the skill based on the termination config
  """
  def __init__(self, namespace):
    self.use_fts_handler = False
    self.use_timeout_handler = False
    self.use_joint_handler = False
    self.use_pose_handler = False
    self.use_audio_handler = False
    self.use_vision_handler = False
    self.id = -1
    self.published = False

    self.termination_pub = rospy.Publisher(
      f"/{namespace}/terminator/skill_termination_signal", 
      TerminationSignal,
      queue_size=1
    )

    self.termination_config_sub = rospy.Subscriber(
      f"/{namespace}/terminator/termination_config", 
      TerminationConfig,
      self.terminator_config_cb,
      queue_size=1
    )

    self.audio_handler_sub = rospy.Subscriber(f"/{namespace}/terminator/audio_termination_signal", TerminationSignal, self.audio_handler_cb)
    self.fts_handler_sub = rospy.Subscriber(f"/{namespace}/terminator/fts_termination_signal", TerminationSignal, self.fts_handler_cb)
    self.timeout_handler_sub = rospy.Subscriber(f"/{namespace}/terminator/time_termination_signal", TerminationSignal, self.timeout_handler_cb)
    self.joint_handler_sub = rospy.Subscriber(f"/{namespace}/terminator/joint_termination_signal", TerminationSignal, self.joint_handler_cb)
    self.pose_handler_sub = rospy.Subscriber(f"/{namespace}/terminator/pose_termination_signal", TerminationSignal, self.pose_handler_cb)
    self.audio_handler_sub = None
    self.vision_handler_sub = None

  def terminator_config_cb(self, cfg: TerminationConfig):
    cfg_jsons = cfg.cfg_json

    print(f"==Received Termination Config==\n{cfg_jsons}")

    cfg_json = json.loads(cfg_jsons)
    self.id = cfg_json['id']
    self.use_fts_handler = 'fts' in cfg_json
    self.use_timeout_handler = 'time' in cfg_json
    self.use_joint_handler = 'joint' in cfg_json
    self.use_pose_handler = 'pose' in cfg_json
    self.use_audio_handler = 'audio' in cfg_json
    self.use_vision_handler = 'vision' in cfg_json
    self.published = False

  def audio_handler_cb(self, audio_termination_signal: TerminationSignal):
    if self.use_audio_handler and audio_termination_signal.id == self.id and not self.published:
      self.termination_pub.publish(audio_termination_signal)
      self.published = True

  def fts_handler_cb(self, fts_termination_signal: TerminationSignal):
    if self.use_fts_handler and fts_termination_signal.id == self.id and not self.published:
      self.termination_pub.publish(fts_termination_signal)
      self.published = True

  def timeout_handler_cb(self, time_termination_signal: TerminationSignal):
    if self.use_timeout_handler and time_termination_signal.id == self.id and not self.published:
      self.termination_pub.publish(time_termination_signal)
      self.published = True

  def joint_handler_cb(self, joint_termination_signal: TerminationSignal):
    if self.use_joint_handler and joint_termination_signal.id == self.id and not self.published:
      self.termination_pub.publish(joint_termination_signal)
      self.published = True

  def pose_handler_cb(self, pose_termination_signal: TerminationSignal):
    if self.use_pose_handler and pose_termination_signal.id == self.id and not self.published:
      self.termination_pub.publish(pose_termination_signal)
      self.published = True


def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  rospy.init_node(f"{namespace}_terminator_node", anonymous=True)
  node = TerminatorNode(namespace)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
