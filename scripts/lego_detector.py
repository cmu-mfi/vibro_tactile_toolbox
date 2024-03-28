#!/usr/bin/env python

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

from detectron2.data.datasets import register_coco_instances
register_coco_instances("lego", {}, "./datasets/lego/annotations.json", "./datasets/lego/")

import cv2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

lego_metadata = MetadataCatalog.get("lego")
dataset_dicts = DatasetCatalog.get("lego")

from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode

from vibro_tactile_toolbox.srv import LegoOutcome, LegoOutcomeResponse

import random

import pickle

clicks = []

class Detectron2:

    def __init__(self):

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = os.path.join("./datasets/lego/model_final.pth")
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.cfg.DATASETS.TEST = ("lego", )
        self.predictor = DefaultPredictor(self.cfg)
        self.bridge = CvBridge()
        self.starting_top = 0
        self.starting_bottom = 0
        self.service = rospy.Service('lego_detector', LegoOutcome, self.detect_lego)

    def detect_lego(self, req):

        img_msg = rospy.wait_for_message(req.topic_name, Image)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        outputs = self.predictor(im)

        outputs_on_cpu = outputs['instances'].to("cpu")
        bounding_boxes = outputs_on_cpu.pred_boxes
        scores = outputs_on_cpu.scores.numpy()

        if req.start:

            ind = 0
            self.starting_top = 0
            self.starting_bottom = 0

            for i in bounding_boxes.__iter__():
                current_score = scores[ind]
                ind += 1
                if current_score >= self.score_threshold:
                    bbox = i.numpy()
                    if bbox[0] >= self.top_bbox[0] and bbox[1] >= self.top_bbox[1] and bbox[2] <= self.top_bbox[2] and bbox[3] <= self.top_bbox[3]:
                        self.starting_top += 1
                    if bbox[0] >= self.bottom_bbox[0] and bbox[1] >= self.bottom_bbox[1] and bbox[2] <= self.bottom_bbox[2] and bbox[3] <= self.bottom_bbox[3]:
                        self.starting_bottom += 1

            print("Starting Top : " + str(self.starting_top))
            print("Starting Bottom : " + str(self.starting_bottom))

        else:
            ind = 0
            self.ending_top = 0
            self.ending_bottom = 0

            for i in bounding_boxes.__iter__():
                current_score = scores[ind]
                ind += 1
                if current_score >= self.score_threshold:
                    bbox = i.numpy()
                    if bbox[0] >= self.top_bbox[0] and bbox[1] >= self.top_bbox[1] and bbox[2] <= self.top_bbox[2] and bbox[3] <= self.top_bbox[3]:
                        self.ending_top += 1
                    if bbox[0] >= self.bottom_bbox[0] and bbox[1] >= self.bottom_bbox[1] and bbox[2] <= self.bottom_bbox[2] and bbox[3] <= self.bottom_bbox[3]:
                        self.ending_bottom += 1

            print("Ending Top : " + str(self.ending_top))
            print("Ending Bottom : " + str(self.ending_bottom))

            if self.starting_top == self.ending_top and self.starting_bottom == self.ending_bottom:
                print('Failed to pick or place block.')
            elif self.starting_top < self.ending_top and self.starting_bottom > self.ending_bottom:
                print('Successfully picked up ' + str(self.ending_top - self.starting_top) + ' block(s).')
            elif self.starting_top > self.ending_top and self.starting_bottom < self.ending_bottom:
                print('Successfully placed ' + str(self.ending_bottom - self.starting_bottom) + ' block(s).')
            else:
                print('Some error has occurred.')


                                                        
def main():
    rospy.init_node('lego_detector_server')
    lego_detector = LegoDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
