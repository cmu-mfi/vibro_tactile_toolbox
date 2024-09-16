import rospy
import json
from geometry_msgs.msg import Wrench

from vibro_tactile_toolbox.msg import *
from vibro_tactile_toolbox.srv import *

from enum import Enum

def send_start_outcome_request(params):
    namespace = params['namespace']
    rospy.wait_for_service(f"/{namespace}/fts_detector")

    try:
        detect_fts = rospy.ServiceProxy(f"/{namespace}/fts_detector", FTSOutcome)
        fts_req = FTSOutcomeRequest()
        fts_req.id = 0
        fts_req.topic_name = params['topic_name']  
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

        return fts_result


    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return None


def send_end_fts_outcome_request(params):
    namespace = params['namespace']
    rospy.wait_for_service(f"/{namespace}/fts_detector")

    try:
      
        detect_fts = rospy.ServiceProxy(f"/{namespace}/fts_detector", FTSOutcome)
        
        req = FTSOutcomeRequest()
        req.id = 0  
        req.topic_name = params['topic_name'] 
        req.start = False  
        req.threshold = Wrench()
   
        req.threshold.force.x = 10
        req.threshold.force.y = 10
        req.threshold.force.z = params['force_threshold']
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

def send_audio_outcome_request(params, timestamp):
    namespace = params['namespace']
    rospy.wait_for_service(f"/{namespace}/audio_detector")

    try:
        detect_audio = rospy.ServiceProxy(f"/{namespace}/audio_detector", AudioOutcome)
        audio_req = AudioOutcomeRequest()
        audio_req.id = 0
        audio_req.topic_name = params['topic_name']  
        audio_req.stamp = timestamp
        audio_req.model_path = params['outcome_model_path'] 
        

        audio_resp = detect_audio(audio_req)
        audio_result = json.loads(audio_resp.result)
        print("Audio Detector Response:", audio_resp.result)
        
        return audio_result


    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return None