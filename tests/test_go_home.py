#!/usr/bin/env python3

import argparse
import rospy
import time

from skill.base_skill import create_skill_publishers
from skill.util_skills import GoHomeSkill
from robot_controller.yk_controller import YaskawaRobotController

def main(args):
    rospy.init_node(f'{args.namespace}_test_go_home', anonymous=True)
    skill_publishers = create_skill_publishers(args.namespace)
    robot = YaskawaRobotController(args.namespace)
    go_home_skill = GoHomeSkill(robot, args.namespace, skill_publishers)
    go_home_skill.execute_skill([{'skill': {}}])

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-n', '--namespace', type=str,
      help='Namespace to use')
    args = args.parse_args()

    main(args)
