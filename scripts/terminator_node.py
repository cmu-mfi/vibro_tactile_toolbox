#!/usr/bin/env python3

import rospy
import json
import argparse

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

    self.termination_pub = rospy.Publisher(
      f"/{args.namespace}/terminator/skill_termination_signal", 
      TerminationSignal,
      queue_size=1
    )

    self.termination_config_sub = rospy.Subscriber(
      f"/{args.namespace}/terminator/termination_config", 
      TerminationConfig,
      self.terminator_config_cb,
      queue_size=1
    )

    self.fts_handler_sub = rospy.Subscriber(f"/{args.namespace}/terminator/fts_termination_signal", TerminationSignal, self.fts_handler_cb)
    self.timeout_handler_sub = rospy.Subscriber(f"/{args.namespace}/terminator/time_termination_signal", TerminationSignal, self.timeout_handler_cb)
    self.joint_handler_sub = rospy.Subscriber(f"/{args.namespace}/terminator/joint_termination_signal", TerminationSignal, self.joint_handler_cb)
    self.pose_handler_sub = rospy.Subscriber(f"/{args.namespace}/terminator/pose_termination_signal", TerminationSignal, self.pose_handler_cb)
    self.audio_handler_sub = None
    self.vision_handler_sub = None

  def terminator_config_cb(self, cfg: TerminationConfig):
    cfg_jsons = cfg.cfg_json

    print(f"==Received Termination Config==\n{cfg_jsons}")

    cfg_json = json.loads(cfg_jsons)
    self.id = cfg_json['id']
    self.use_fts_handler = 'fts' in cfg_json
    self.use_timeout_handler = 'timeout' in cfg_json
    self.use_joint_handler = 'joint' in cfg_json
    self.use_pose_handler = 'pose' in cfg_json
    self.use_audio_handler = 'audio' in cfg_json
    self.use_vision_handler = 'vision' in cfg_json

  def fts_handler_cb(self, fts_termination_signal: TerminationSignal):
    if self.use_fts_handler and fts_termination_signal.id == self.id:
      self.termination_pub.publish(fts_termination_signal)

  def timeout_handler_cb(self, time_termination_signal: TerminationSignal):
    if self.use_timeout_handler and time_termination_signal.id == self.id:
      self.termination_pub.publish(time_termination_signal)

  def joint_handler_cb(self, joint_termination_signal: TerminationSignal):
    if self.use_joint_handler and joint_termination_signal.id == self.id:
      self.termination_pub.publish(joint_termination_signal)

  def pose_handler_cb(self, pose_termination_signal: TerminationSignal):
    if self.use_pose_handler and pose_termination_signal.id == self.id:
      self.termination_pub.publish(pose_termination_signal)


def main(args):
  rospy.init_node(f'{args.namespace}_terminator_node', anonymous=True)
  terminator_node = TerminatorNode(args.namespace)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-n', '--namespace', type=str,
      help='Namespace to use')
    args = args.parse_args()

    main(args)