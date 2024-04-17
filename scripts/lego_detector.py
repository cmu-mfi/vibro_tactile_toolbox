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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

import cv2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

from vibro_tactile_toolbox.srv import LegoOutcome, LegoOutcomeResponse
from vibro_tactile_toolbox.msg import BoundingBox

class LegoDetector:

    def __init__(self):

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = os.path.join("./models/lego_model.pth")
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.predictor = DefaultPredictor(self.cfg)
        self.bridge = CvBridge()
        self.starting_top = 0
        self.starting_bottom = 0
        self.service = rospy.Service('lego_detector', LegoOutcome, self.detect_lego)

    def detect_lego(self, req):

        img_msg = rospy.wait_for_message(req.topic_name, Image)
        resp = LegoOutcomeResponse()
        resp.img = img_msg

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        outputs = self.predictor(cv_image)

        outputs_on_cpu = outputs['instances'].to("cpu")
        # print(outputs_on_cpu)
        bounding_boxes = outputs_on_cpu.pred_boxes
        scores = outputs_on_cpu.scores.numpy()

        if req.start:

            ind = 0
            self.starting_top = 0
            self.starting_bottom = 0

            for i in bounding_boxes.__iter__():
                current_score = scores[ind]
                ind += 1
                if current_score >= req.score_threshold:
                    bbox = i.numpy()
                    print(bbox)
                    bbox_msg = BoundingBox()
                    bbox_msg.coords = bbox
                    resp.obj_bboxes.append(bbox_msg)
                    if bbox[0] >= req.top_bbox.coords[0] and bbox[1] >= req.top_bbox.coords[1] and bbox[2] <= req.top_bbox.coords[2] and bbox[3] <= req.top_bbox.coords[3]:
                        self.starting_top += 1
                    if bbox[0] >= req.bot_bbox.coords[0] and bbox[1] >= req.bot_bbox.coords[1] and bbox[2] <= req.bot_bbox.coords[2] and bbox[3] <= req.bot_bbox.coords[3]:
                        self.starting_bottom += 1

            resp.result = json.dumps({'starting_top' : str(self.starting_top), 'starting_bottom' : str(self.starting_bottom)})
            print("Starting Top : " + str(self.starting_top))
            print("Starting Bottom : " + str(self.starting_bottom))
            return resp

        else:
            ind = 0

            self.ending_top = 0
            self.ending_bottom = 0

            for i in bounding_boxes.__iter__():
                current_score = scores[ind]
                ind += 1
                if current_score >= req.score_threshold:
                    bbox = i.numpy()
                    bbox_msg = BoundingBox()
                    bbox_msg.coords = bbox
                    resp.obj_bboxes.append(bbox_msg)
                    if bbox[0] >= req.top_bbox.coords[0] and bbox[1] >= req.top_bbox.coords[1] and bbox[2] <= req.top_bbox.coords[2] and bbox[3] <= req.top_bbox.coords[3]:
                        self.ending_top += 1
                    if bbox[0] >= req.bot_bbox.coords[0] and bbox[1] >= req.bot_bbox.coords[1] and bbox[2] <= req.bot_bbox.coords[2] and bbox[3] <= req.bot_bbox.coords[3]:
                        self.ending_bottom += 1

            
            result = ''

            if self.starting_top == self.ending_top and self.starting_bottom == self.ending_bottom:
                result = 'Failed to pick or place block.'
            elif self.starting_top < self.ending_top and self.starting_bottom > self.ending_bottom:
                result = 'Successfully picked up ' + str(self.ending_top - self.starting_top) + ' block(s).'
            elif self.starting_top > self.ending_top and self.starting_bottom < self.ending_bottom:
                result = 'Successfully placed ' + str(self.ending_bottom - self.starting_bottom) + ' block(s).'
            else:
                result = 'Some error has occurred.'

            resp.result = json.dumps({'starting_top' : str(self.starting_top), 'starting_bottom' : str(self.starting_bottom), 
                                      'ending_top' : str(self.ending_top), 'ending_bottom' : str(self.ending_bottom),
                                      'result' : result})
            print("Starting Top : " + str(self.starting_top))
            print("Starting Bottom : " + str(self.starting_bottom))
            print("Ending Top : " + str(self.ending_top))
            print("Ending Bottom : " + str(self.ending_bottom))
            # print(result)
            return resp
                                                        
def main():
    rospy.init_node('lego_detector_server')
    lego_detector = LegoDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
