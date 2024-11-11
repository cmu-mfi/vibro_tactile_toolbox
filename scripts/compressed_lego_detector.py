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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import json

import cv2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

from vibro_tactile_toolbox.srv import LegoOutcome, LegoOutcomeResponse
from vibro_tactile_toolbox.msg import BoundingBox, VisionOutcomeRepub

class LegoDetector:

    def __init__(self, namespace):

        self.cfg = get_cfg()
        self.root_pwd = rospy.get_param('compressed_lego_detector_node/root_pwd')
        self.model_path = rospy.get_param('compressed_lego_detector_node/model_path')
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = os.path.join(self.root_pwd, self.model_path)
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.predictor = DefaultPredictor(self.cfg)
        self.bridge = CvBridge()
        self.starting_top = 0
        self.starting_bottom = 0
        self.service = rospy.Service(f"/{namespace}/lego_detector", LegoOutcome, self.detect_lego)

        self.outcome_repub = rospy.Publisher(f"/{namespace}/outcome/lego_detector", VisionOutcomeRepub, queue_size=1)

    def detect_lego(self, req):

        img_msg = rospy.wait_for_message(req.topic_name, CompressedImage)
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        resp = LegoOutcomeResponse()
        resp.img = img_msg

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
                print(current_score)
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

            resp.result = json.dumps({'starting_top' : self.starting_top, 'starting_bottom' : self.starting_bottom,
                                      'result': 'Recording brick locations before next action',
                                      'success': None})
            print("Starting Top : " + str(self.starting_top))
            print("Starting Bottom : " + str(self.starting_bottom))

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

            success = ('success' in result.lower())

            resp.result = json.dumps({'starting_top' : self.starting_top, 'starting_bottom' : self.starting_bottom, 
                                      'ending_top' : self.ending_top, 'ending_bottom' : self.ending_bottom,
                                      'result' : result,
                                      'success': success})
            print("Starting Top : " + str(self.starting_top))
            print("Starting Bottom : " + str(self.starting_bottom))
            print("Ending Top : " + str(self.ending_top))
            print("Ending Bottom : " + str(self.ending_bottom))
            # print(result)

        # Publish an annotated image for rosbag recording purposes
        image_ann = cv_image.copy()
        ind = 0
        for i in bounding_boxes.__iter__():
            current_score = scores[ind]
            ind += 1
            if current_score >= req.score_threshold:
                bbox = i.numpy()
                cv2.rectangle(image_ann, (int(bbox[0]), int(bbox[1])), 
                                         (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)

        outcome_repub_msg = VisionOutcomeRepub()
        outcome_repub_msg.id = resp.id
        outcome_repub_msg.img.header = img_msg.header
        outcome_repub_msg.img.format = "jpeg"
        outcome_repub_msg.img.data = np.array(cv2.imencode('.jpg', image_ann)[1]).tobytes()
        outcome_repub_msg.result = resp.result
        outcome_repub_msg.obj_bboxes = resp.obj_bboxes.copy()
        outcome_repub_msg.obj_ids = resp.obj_ids.copy()
        self.outcome_repub.publish(outcome_repub_msg)

        return resp
                                                        
def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  rospy.init_node("compressed_lego_detector_node", anonymous=True)
  node = LegoDetector(namespace)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

