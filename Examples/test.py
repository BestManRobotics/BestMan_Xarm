# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_arm_to_home.py
# @Time           : 2024-12-01 15:21:45
# @Author         : Zhaxi & Yan 
# @Email          : zhaxizhuoma.ayang@gmail.com & yding25@binghamton.edu 
# @Description    : Move arm to default home position
# @Usage          : python test.py 192.168.1.208
"""

import argparse
import time
from Robotics_API import Bestman_Real_Xarm6
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(
        description="Move the robot arm to follow a trajectory."
    )
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    args = argparser.parse_args()

    try:
        # Instantiate the robot interface
        bestman = Bestman_Real_Xarm6(args.robot_ip)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # bestman.update_robot_states()

        # bestman.go_home()
        # bestman.get_tcp_link()
        # bestman.get_current_eef_pose()

        # Test
        gt_pose = bestman.get_eef_pos()
        gt_joint = bestman.get_joint_ang()

        # gt_joint[1] += 0.3
        # bestman.move_to_joint_angles(gt_joint)
        
        # gt_pose.position[2] += 0.3
        # bestman.move_to_eef_pose(gt_pose)

        # print(f'gt position:{gt_pose.position}, gt orientation:{gt_pose.orientation}')
        # print(f'gt joint:{gt_joint}')
        # # joint_values = [-1.6, -27.2, -2.2, -1.3, -60.6, 0.2]
        # # joint_values_rad = [v / 180 * math.pi for v in joint_values]
        # # joint_values_rad = [-0.12792803756892681, -0.47472870349884033, -0.038403209298849106, -0.022691410034894943, -1.0576682090759277, 0.0034917236771434546]
        # # bestman.move_arm_to_joint_values(joint_values_rad, wait_for_finish=False)

        # # joint_values_rad = [-0.22792803756892681, -0.47472870349884033, -0.038403209298849106, -0.022691410034894943, -1.0576682090759277, 0.0034917236771434546]
        # # bestman.move_arm_to_joint_values(joint_values_rad, wait_for_finish=False)
        
        # # pose.position[2] += 0.1
        # # print(f'test position:{pose.position}, test orientation:{pose.orientation}')
        # # bestman.move_eef_to_goal_pose(pose)
        
        trans_joint = bestman.cartesian_to_joints(gt_pose)
        print(f'trans_joint:{trans_joint}')
        bestman.move_to_joint_ang(trans_joint)

        trans_pose = bestman.joints_to_cartesian(gt_joint)
        print(f'trans_pose:{trans_pose}')
        bestman.move_to_eef_pos(trans_pose)

        bestman.close_gripper_robotiq()
        bestman.open_gripper_robotiq()
        bestman.set_gripper_pos_robotiq(100)
        # print(bestman.get_gripper_position_robotiq())
        
    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
