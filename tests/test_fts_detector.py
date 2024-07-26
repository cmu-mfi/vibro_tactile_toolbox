#!/usr/bin/env python3

import argparse
import rospy
from vibro_tactile_toolbox.srv import *
from geometry_msgs.msg import Wrench
import json

def test_fts_detector(topic_name):
    rospy.wait_for_service('/fts_detector')
    try:
        detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
        req = FTSOutcomeRequest()
        req.id = 0
        req.topic_name = topic_name
        req.start = True
        req.threshold = Wrench()
        req.threshold.force.x = 10
        req.threshold.force.y = 10
        req.threshold.force.z = 2.0
        req.threshold.torque.x = 10
        req.threshold.torque.y = 10
        req.threshold.torque.z = 10
        resp = detect_fts(req)
        resp.result
        print(resp)
        result = json.loads(resp.result)
        print("force smaller than threshold?",result["starting_forces"][2]<req.threshold.force.z)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main(args):
    rospy.init_node('test_fts_detector', anonymous=True)
    test_fts_detector(args.topic_name)

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-t', '--topic_name', type=str,
      help='Topic name to use')
    args = args.parse_args()

    main(args)