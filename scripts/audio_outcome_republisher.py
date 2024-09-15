import rospy
from vibro_tactile_toolbox.msg import AudioOutcomeRepub
from std_msgs.msg import Int16
import json
import sys
import time

class AudioOutcomeRepublisher:

    def __init__(self, namespace):
        self.time_delay = rospy.get_param('audio_outcome_republisher_node/time_delay')
        print(f'/{namespace}/outcome/audio_detector')
        self.audio_outcom_repub_sub = rospy.Subscriber(f'/{namespace}/outcome/audio_detector', AudioOutcomeRepub, self.republish, queue_size=1) #standard publisher to control movement
        self.outcome_repub = rospy.Publisher(f'/{namespace}/audio_detector/outcome_int', Int16, queue_size=1)
        self.last_time = 0

    def republish(self, msg):
        self.last_time = time.time()
        if json.loads(msg.result)['success'] == 0:
            self.outcome_repub.publish(0)
        else:
            self.outcome_repub.publish(1)

    def publish_no_data(self):
        current_time = time.time()
        if current_time - self.last_time > self.time_delay:
            self.outcome_repub.publish(2)

def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  rospy.init_node("audio_outcome_republisher_node", anonymous=True)
  aor = AudioOutcomeRepublisher(namespace)
  frequency = rospy.get_param('audio_outcome_republisher_node/frequency')
  rate = rospy.Rate(frequency)

  try:
     while not rospy.is_shutdown():
        aor.publish_no_data()
        rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)