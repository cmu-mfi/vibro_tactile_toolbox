#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import json
import time

from vibro_tactile_toolbox.msg import TerminationConfig, TerminationSignal
from geometry_msgs.msg import Wrench

import terminator.utils as t_utils


class TerminatorTest:

  def __init__(self):
    self.termination_cfg_pub = rospy.Publisher("/terminator/termination_config", TerminationConfig, queue_size=1)

    self.termination_signal_sub = rospy.Subscriber("/terminator/skill_termination_signal", TerminationSignal, self.terminate_skill_cb)

    self.FTS_lims = {'force': 30, 'torque': 1.5}

    self.termination_cfg_pub.publish(self.make_random_cfg())

  def terminate_skill_cb(self, msg: TerminationSignal):
    print(f"Terminating skill with cause: \n{msg.cause}")
    self.termination_cfg_pub.publish(self.make_random_cfg())

  def make_random_cfg(self) -> TerminationConfig:
    id = np.random.randint(1000)

    # FTS config
    FTS_check_rate_ns = np.random.randint(1E6, 10E6) #1ms to 10ms
    FTS_thresh = Wrench()
    FTS_thresh.force.x = np.random.random() * self.FTS_lims['force']
    FTS_thresh.force.y = np.random.random() * self.FTS_lims['force']
    FTS_thresh.force.z = np.random.random() * self.FTS_lims['force']
    FTS_thresh.torque.x = np.random.random() * self.FTS_lims['torque']
    FTS_thresh.torque.y = np.random.random() * self.FTS_lims['torque']
    FTS_thresh.torque.z = np.random.random() * self.FTS_lims['torque']

    cfg_json = {
      'id': id,
      'FTS': {'check_rate_ns': FTS_check_rate_ns, 'threshold': t_utils.wrench_to_dict(FTS_thresh)},
      'timeout': {'timeout_duration_ns': 5E9}
    }

    msg = TerminationConfig()
    msg.cfg_json = json.dumps(cfg_json)
    return msg


def main(args):
  rospy.init_node('test_terminator', anonymous=True)
  node = TerminatorTest()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)