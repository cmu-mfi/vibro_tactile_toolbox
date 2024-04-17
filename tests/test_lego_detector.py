#!/usr/bin/env python3

import argparse
import rospy
from vibro_tactile_toolbox.srv import *
from vibro_tactile_toolbox.msg import BoundingBox
import json

def test_lego_detector(topic_name):
    rospy.wait_for_service('/lego_detector')
    try:
        detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
        req = LegoOutcomeRequest()
        req.id = 0
        req.topic_name = topic_name
        req.start = False
        req.score_threshold = 0.8
        top_bbox = BoundingBox()
        top_bbox.coords = [800, 200, 1100, 350]
        bot_bbox = BoundingBox()
        bot_bbox.coords = [800, 350, 1100, 500]
        req.top_bbox = top_bbox
        req.bot_bbox = bot_bbox
        resp = detect_lego(req)
        result = json.loads(resp.result)
        print("resp result is",resp.result, "resp starting_bottom is", result["starting_bottom"])
        print("combined",not result["starting_top"] and  not result["starting_bottom"])
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main(args):
    rospy.init_node('test_lego_detector', anonymous=True)
    test_lego_detector(args.topic_name)

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-t', '--topic_name', type=str,
      help='Topic name to use')
    args = args.parse_args()

    main(args)