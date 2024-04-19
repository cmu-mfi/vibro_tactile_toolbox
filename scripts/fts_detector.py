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

class FTSDetector:

    def __init__(self):

        self.starting_wrench = Wrench()
        self.wrench_threshold = Wrench()
        self.service = rospy.Service('fts_detector', FTSOutcome, self.detect_fts)

        self.outcome_pub = rospy.Publisher('/outcome/fts_detector', Wrench, queue_size=1)

    def detect_fts(self, req):
        print("Received Request")
        current_wrench = rospy.wait_for_message(req.topic_name, WrenchStamped).wrench
        print("Received Current Wrench")
        resp = FTSOutcomeResponse()
        resp.wrench = current_wrench
        self.wrench_threshold = req.threshold

        # For rosbag record purposes
        outcome_msg = Wrench()
        outcome_msg.force.x = current_wrench.force.x
        outcome_msg.force.y = current_wrench.force.y
        outcome_msg.force.z = current_wrench.force.z
        outcome_msg.torque.x = current_wrench.torque.x
        outcome_msg.torque.y = current_wrench.torque.y
        outcome_msg.torque.z = current_wrench.torque.z
        self.outcome_pub.publish(outcome_msg)

        if req.start:
            self.starting_wrench = current_wrench

            resp.result = json.dumps({'starting_forces' : [current_wrench.force.x, current_wrench.force.y, current_wrench.force.z]})
            print("Starting Forces : " + str([current_wrench.force.x, current_wrench.force.y, current_wrench.force.z]))
            return resp

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
                if (current_wrench.force.z - self.starting_wrench.force.z) > self.wrench_threshold.force.z:
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

            resp.result = json.dumps({'starting_forces' : [self.starting_wrench.force.x, self.starting_wrench.force.y, self.starting_wrench.force.z], 
                                      'ending_forces' : [current_wrench.force.x, current_wrench.force.y, current_wrench.force.z], 
                                      'result' : result})
            print("Starting Forces : " + str([self.starting_wrench.force.x, self.starting_wrench.force.y, self.starting_wrench.force.z]))
            print("Ending Forces : " + str([current_wrench.force.x, current_wrench.force.y, current_wrench.force.z]))
            print(result)

            return resp
                                                        
def main():
    rospy.init_node('fts_detector_server')
    fts_detector = FTSDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
