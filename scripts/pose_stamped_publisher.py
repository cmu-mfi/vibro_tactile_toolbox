import sys
import argparse
import rospy
from geometry_msgs.msg import PoseStamped
from yk_msgs.srv import GetPoseStamped

def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  node_name = args[1].split(':=')[1]
  rospy.init_node(f"{namespace}_pose_stamped_publisher", anonymous=True)
  freq = rospy.get_param(f"{node_name}/frequency")
  pub = rospy.Publisher('/'+ namespace +'/pose', PoseStamped, queue_size=10)
  get_pose_stamped_client = rospy.ServiceProxy('/'+ namespace +'/yk_get_pose_stamped', GetPoseStamped)

  r = rospy.Rate(freq)
  while not rospy.is_shutdown():
    pub.publish(get_pose_stamped_client("base_link").pose)
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)
