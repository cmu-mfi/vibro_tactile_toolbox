import rospy
import json
from geometry_msgs.msg import Wrench

from vibro_tactile_toolbox.msg import *
from vibro_tactile_toolbox.srv import *

from enum import Enum

class LegoOutcome(Enum):
    """
    Codes to describe skill outcomes
    """
    SUCCESS = 0
    FAIL = 1
    CONNECTED = 2
    DISCONNECTED = 3



def send_start_outcome_request(params):
    rospy.wait_for_service('/fts_detector')
    rospy.wait_for_service('/lego_detector')

    try:
        detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
        fts_req = FTSOutcomeRequest()
        fts_req.id = 0
        fts_req.topic_name = 'fts'  
        fts_req.start = True
        fts_req.threshold = Wrench()
        fts_req.threshold.force.x = 10
        fts_req.threshold.force.y = 10
        fts_req.threshold.force.z = 10
        fts_req.threshold.torque.x = 10
        fts_req.threshold.torque.y = 10
        fts_req.threshold.torque.z = 10
        

        fts_resp = detect_fts(fts_req)
        fts_result = json.loads(fts_resp.result)
        print("FTS Detector Response:", fts_resp.result)
        

        detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
        lego_req = LegoOutcomeRequest()
        lego_req.id = 0
        lego_req.topic_name = '/side_camera/color/image_cropped'
        lego_req.start = True
        lego_req.score_threshold = params['detection_threshold']
        
        top_bbox = BoundingBox()
        top_bbox.coords = params['top_bbox']
        bot_bbox = BoundingBox()
        bot_bbox.coords = params['bot_bbox']
        lego_req.top_bbox = top_bbox
        lego_req.bot_bbox = bot_bbox

        lego_resp = detect_lego(lego_req)
        lego_result = json.loads(lego_resp.result)
        print("Vision Detection Response:", lego_resp.result)
        return lego_result


    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return None


def send_end_fts_outcome_request(params):
    rospy.wait_for_service('/fts_detector')
    try:
      
        detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
        
        req = FTSOutcomeRequest()
        req.id = 0  
        req.topic_name ='fts'
        req.start = False  
        req.threshold = Wrench()
   
        req.threshold.force.x = 10
        req.threshold.force.y = 10
        req.threshold.force.z = param['force_threshold']
        req.threshold.torque.x = 10
        req.threshold.torque.y = 10
        req.threshold.torque.z = 10
        
        resp = detect_fts(req)
        print("FTS Detector Response:", resp.result)
        result = json.loads(resp.result)
        return result
    
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return None


def send_end_vision_outcome_request(params):
    rospy.wait_for_service('/lego_detector')
    try:
       
        detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
        
    
        req = LegoOutcomeRequest()
        req.id = 0  
        req.topic_name = '/side_camera/color/image_cropped'
        req.start = False 
        req.score_threshold = params['detection_threshold']  
        
        top_bbox = BoundingBox()
        top_bbox.coords = params['top_bbox']
        bot_bbox = BoundingBox()
        bot_bbox.coords = params['bot_bbox']
        req.top_bbox = top_bbox
        req.bot_bbox = bot_bbox
        
        resp = detect_lego(req)
        print("Vision Detection Response:", resp.result)
        result = json.loads(resp.result)
        return result
    except rospy.ServiceException as e:
        print("Vision service call failed:", e)
        return None
