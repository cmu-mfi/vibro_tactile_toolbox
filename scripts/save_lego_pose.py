#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from robot_controller.yk_controller import YaskawaRobotController

from autolab_core import RigidTransform

def run():

    rospy.init_node("save_lego_pose")

    namespace = rospy.get_param("save_lego_pose/namespace")
    root_pwd = rospy.get_param("save_lego_pose/root_pwd")
    x = rospy.get_param("save_lego_pose/x")
    y = rospy.get_param("save_lego_pose/y")

    robot_commander = YaskawaRobotController(namespace)
    
    T_lego_ee = RigidTransform.load(root_pwd+'/transforms/lego_ee.tf')

    T_ee_world = RigidTransform.from_pose_msg(robot_commander.get_current_pose(), from_frame='ee')

    T_lego_world = T_ee_world * T_lego_ee

    T_lego_world.save(f'{root_pwd}/transforms/T_lego_world/lego_world_{x}_{y}.tf')

if __name__ == "__main__":
    run()
