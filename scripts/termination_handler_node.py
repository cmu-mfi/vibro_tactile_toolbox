#!/usr/bin/env python3

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

  def __init__(self, handler_name: str, input_topic: str):
    # Publisher for the output signal
    self.termination_signal_pub = rospy.Publisher(
      f"/terminator/{handler_name}_termination_signal", 
      TerminationSignal
    )
    # Subscriber for terminator config
    self.termination_config_sub = rospy.Subscriber(
      "/terminator/termination_config",
      TerminationConfig,
      self.termination_config_cb,
      queue_size=1
    )

    # Instantiate the termination handler
    self.termination_handler = t_utils.get_handler_from_name(handler_name)

    # Subscriber for the input signal
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
    termination_signal = self.termination_handler.get_termination_signal()
    if termination_signal.terminate:
      self.termination_signal_pub.publish(termination_signal)

def main(args):
  print(args)
  rospy.init_node(f'{args.termination_handler}_termination_handler_node', anonymous=True)
  node = TerminationHandlerNode(args.termination_handler, args.input_topic)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-t', '--termination_handler', type=str,
      help='Termination handler to load into this node')
    # Arg for specifying the ROS topics to use for the input to the termination handler
    args.add_argument('-i', '--input_topic', type=str,
      help='ROS topic to input to termination handler')
    
    args = args.parse_args()

    main(args)