#!/usr/bin/env python3

import os
import time
import signal
import sys
import argparse
import rospy
import tf
import math
import numpy as np
import json
from geometry_msgs.msg import Wrench, WrenchStamped

from vibro_tactile_toolbox.srv import FTSOutcome, FTSOutcomeResponse
from vibro_tactile_toolbox.msg import FtsOutcomeRepub

class FTSDetector:

    def __init__(self):

        self.starting_wrench = Wrench()
        self.wrench_threshold = Wrench()
        self.service = rospy.Service('fts_detector', FTSOutcome, self.detect_fts)

        self.outcome_repub = rospy.Publisher('/outcome/fts_detector', FtsOutcomeRepub, queue_size=1)

    def detect_fts(self, req):
        print("Received Request")
        current_wrench = rospy.wait_for_message(req.topic_name, WrenchStamped).wrench
        print("Received Current Wrench")
        resp = FTSOutcomeResponse()
        resp.wrench = current_wrench
        self.wrench_threshold = req.threshold

        if req.start:
            self.starting_wrench = current_wrench

            resp.result = json.dumps({'starting_forces' : [current_wrench.force.x, current_wrench.force.y, current_wrench.force.z],
                                      'result': 'Measuring FTS before next action',
                                      'success': None})
            
            print("Starting Forces : " + str([current_wrench.force.x, current_wrench.force.y, current_wrench.force.z]))

        else:
            
            result = ''
            if self.wrench_threshold.force.x > 0:
                if (current_wrench.force.x - self.starting_wrench.force.x) > self.wrench_threshold.force.x:
                    result += f"Fx diff ({(current_wrench.force.x - self.starting_wrench.force.x):0.2f}) exceeds threshold ({self.wrench_threshold.force.x:0.2f})\n"
            else:
                if (current_wrench.force.x - self.starting_wrench.force.x) < self.wrench_threshold.force.x:
                    result += f"Fx diff ({(current_wrench.force.x - self.starting_wrench.force.x):0.2f}) exceeds threshold ({self.wrench_threshold.force.x:0.2f})\n"
            if self.wrench_threshold.force.y > 0:
                if (current_wrench.force.y - self.starting_wrench.force.y) > self.wrench_threshold.force.y:
                    result += f"Fy diff ({(current_wrench.force.y - self.starting_wrench.force.y):0.2f}) exceeds threshold ({self.wrench_threshold.force.y:0.2f})\n"
            else:
                if (current_wrench.force.y - self.starting_wrench.force.y) < self.wrench_threshold.force.y:
                    result += f"Fy diff ({(current_wrench.force.y - self.starting_wrench.force.y):0.2f}) exceeds threshold ({self.wrench_threshold.force.y:0.2f})\n"
            if self.wrench_threshold.force.z > 0:
                if (current_wrench.force.z  - self.starting_wrench.force.z) > self.wrench_threshold.force.z:
                    result += f"Fz diff ({(current_wrench.force.z - self.starting_wrench.force.z):0.2f}) exceeds threshold ({self.wrench_threshold.force.z:0.2f})\n"
            else:
                if (current_wrench.force.z - self.starting_wrench.force.z) < self.wrench_threshold.force.z:
                    result += f"Fz diff ({(current_wrench.force.z - self.starting_wrench.force.z):0.2f}) exceeds threshold ({self.wrench_threshold.force.z:0.2f})\n"

            if self.wrench_threshold.torque.x > 0:
                if (current_wrench.torque.x - self.starting_wrench.torque.x) > self.wrench_threshold.torque.x:
                    result += f"Fx diff ({(current_wrench.torque.x - self.starting_wrench.torque.x):0.2f}) exceeds threshold ({self.wrench_threshold.torque.x:0.2f})\n"
            else:
                if (current_wrench.torque.x - self.starting_wrench.torque.x) < self.wrench_threshold.torque.x:
                    result += f"Fx diff ({(current_wrench.torque.x - self.starting_wrench.torque.x):0.2f}) exceeds threshold ({self.wrench_threshold.torque.x:0.2f})\n"
            if self.wrench_threshold.torque.y > 0:
                if (current_wrench.torque.y - self.starting_wrench.torque.y) > self.wrench_threshold.torque.y:
                    result += f"Fy diff ({(current_wrench.torque.y - self.starting_wrench.torque.y):0.2f}) exceeds threshold ({self.wrench_threshold.torque.y:0.2f})\n"
            else:
                if (current_wrench.torque.y - self.starting_wrench.torque.y) < self.wrench_threshold.torque.y:
                    result += f"Fy diff ({(current_wrench.torque.y - self.starting_wrench.torque.y):0.2f}) exceeds threshold ({self.wrench_threshold.torque.y:0.2f})\n"
            if self.wrench_threshold.torque.z > 0:
                if (current_wrench.torque.z - self.starting_wrench.torque.z) > self.wrench_threshold.torque.z:
                    result += f"Fz diff ({(current_wrench.torque.z - self.starting_wrench.torque.z):0.2f}) exceeds threshold ({self.wrench_threshold.torque.z:0.2f})\n"
            else:
                if (current_wrench.torque.z - self.starting_wrench.torque.z) < self.wrench_threshold.torque.z:
                    result += f"Fz diff ({(current_wrench.torque.z - self.starting_wrench.torque.z):0.2f}) exceeds threshold ({self.wrench_threshold.torque.z:0.2f})\n"

            success = (len(result) > 0)
            resp.result = json.dumps({'starting_forces' : [self.starting_wrench.force.x, self.starting_wrench.force.y, self.starting_wrench.force.z], 
                                      'ending_forces' : [current_wrench.force.x, current_wrench.force.y, current_wrench.force.z], 
                                      'result' : result,
                                      'success': success})
            print("Starting Forces : " + str([self.starting_wrench.force.x, self.starting_wrench.force.y, self.starting_wrench.force.z]))
            print("Ending Forces : " + str([current_wrench.force.x, current_wrench.force.y, current_wrench.force.z]))
            print(result)

        # For rosbag record purposes
        outcome_repub_msg = FtsOutcomeRepub()
        outcome_repub_msg.id = resp.id
        outcome_repub_msg.wrench = resp.wrench
        outcome_repub_msg.result = resp.result
        self.outcome_repub.publish(outcome_repub_msg)

        return resp
                                                        
def main():
    rospy.init_node('fts_detector_server')
    fts_detector = FTSDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
