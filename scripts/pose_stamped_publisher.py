import argparse
import rospy
from geometry_msgs.msg import PoseStamped
from yk_tasks.srv import GetPoseStamped


def main(args):
   pub = rospy.Publisher('/'+args.namespace+'/pose', PoseStamped, queue_size=10)
   get_pose_stamped_client = rospy.ServiceProxy('/'+args.namespace+'/yk_get_pose_stamped', GetPoseStamped)

   rospy.init_node(args.namespace + '_pose_stamped_publisher')
   r = rospy.Rate(args.freq)
   while not rospy.is_shutdown():
      pub.publish(get_pose_stamped_client("base_link").pose)
      r.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Published Pose Stamped')
    parser.add_argument('--namespace', '-n', type=str, required=True,
                        help='Robot Namespace')
    parser.add_argument('--freq', '-f', type=int, required=True,
                        help='Publishing Frequency')
    args = parser.parse_args()

    main(args)