#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from robot_controller.yk_commander import YaskawaRobotCommander

from autolab_core import RigidTransform

def run():

    rospy.init_node("save_hande_pose")

    namespace = rospy.get_param("save_hande_pose/namespace")
    root_pwd = rospy.get_param("save_hande_pose/root_pwd")

    robot_mg = YaskawaRobotCommander(namespace)
    
    T_lego_ee = RigidTransform.load(root_pwd+'/transforms/hande_ee.tf')

    T_ee_world = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_lego_world = T_ee_world * T_lego_ee

    T_lego_world.save(root_pwd+'/transforms/hande_world.tf')

if __name__ == "__main__":
    run()
