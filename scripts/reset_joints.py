#!/usr/bin/env python3

import rospy
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from skill.util_skills import ResetJoints

if __name__ == "__main__":
    # Start Node
    rospy.init_node("reset_joints")

    # Messaging Namespace
    namespace = rospy.get_param("reset_joints/namespace")
    verbose = rospy.get_param("reset_joints/verbose")
    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)

    # 2. Begin Skill
    params = {'verbose': verbose}
    reset_joints_skill = ResetJoints(robot_commander, namespace, params)
    terminals = reset_joints_skill.execute_skill(None)
    
    for i, terminal in enumerate(terminals):
        print(f"step {i} termination cause:\n{terminal.cause}\n")