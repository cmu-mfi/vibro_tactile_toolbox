#!/usr/bin/env python3

import sys
import rospy
import time

from skill.util_skills import GoHomeSkill
from robot_controller.yk_controller import YaskawaRobotController


def main(args):
  rospy.init_node('test_go_home', anonymous=True)
  robot = YaskawaRobotController('yk_creator')
  go_home_skill = GoHomeSkill(robot)
  go_home_skill.execute_skill([{'skill': {}}])

if __name__ == '__main__':
    main(sys.argv)