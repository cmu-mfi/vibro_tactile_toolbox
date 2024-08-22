#!/usr/bin/env python3

import sys
import rospy
import argparse

from std_msgs.msg import Bool
from rospy.timer import TimerEvent
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

from terminator.base_termination_handler import BaseTerminationHandler
import terminator.utils as t_utils


class TerminationHandlerNode:
  """
  The common node used for creating termination handlers.
  Each termination handler node is created by specifying the:
  1. Termination handler (from src/terminator/)
  2. Input signal topics (must match the expected input signal message types of the termination handler)
  """
  termination_handler: BaseTerminationHandler

  def __init__(self, namespace: str, handler_name: str, input_topic: str):
    # Publisher for the output signal
    self.termination_signal_pub = rospy.Publisher(
      f"/{namespace}/terminator/{handler_name}_termination_signal", 
      TerminationSignal,
      queue_size=1
    )
    # Subscriber for terminator config
    self.termination_config_sub = rospy.Subscriber(
      f"/{namespace}/terminator/termination_config",
      TerminationConfig,
      self.termination_config_cb,
      queue_size=1
    )

    # Instantiate the termination handler
    self.termination_handler = t_utils.get_handler_from_name(handler_name)

    # Subscriber for the input signal
    if input_topic == "":
      self.input_signal_sub = None
    else:
      self.input_signal_sub = rospy.Subscriber(
        f"{input_topic}",
        self.termination_handler.input_data_class,
        self.termination_handler.update_input_data,
        queue_size=10
      )

    # Timer for publishing the termination signal
    self.check_termination_timer = rospy.Timer(
      rospy.Duration(0, self.termination_handler.check_rate_ns), 
      self.check_termination_cb)
    
  def termination_config_cb(self, cfg: TerminationConfig):
    """
    Update the termination handler configuration and node check rate
    """
    # Update the termination handler's config
    self.termination_handler.update_config(cfg)

    # Reset the check timer
    self.check_termination_timer.shutdown()
    self.check_termination_timer = rospy.Timer(
      rospy.Duration(0, self.termination_handler.check_rate_ns), 
      self.check_termination_cb)
    
  def check_termination_cb(self, event: TimerEvent):
    """
    Callback to check if this termination handler should publish a termination signal
    """
    if not self.termination_handler.live:
      return
    termination_signal = self.termination_handler.get_termination_signal()
    if termination_signal.terminate and termination_signal.id > 0:
      termination_signal.stamp = rospy.Time.now()
      # print(f"\nTermination Handler {self.termination_handler.__class__.__name__} publishing termination signal\n")
      self.termination_signal_pub.publish(termination_signal)

def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  node_name = args[1].split(':=')[1]
  rospy.init_node(namespace + node_name, anonymous=True)
  termination_handler = rospy.get_param(f'{node_name}/type')
  topic = rospy.get_param(f'{node_name}/topic')
  node = TerminationHandlerNode(namespace, termination_handler, topic)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)