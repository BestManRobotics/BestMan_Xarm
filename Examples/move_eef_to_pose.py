# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_eef_to_pose.py
# @Time           : 2024-12-01 15:21:45
# @Author         : Zhaxi & Yan 
# @Email          : zhaxizhuoma.ayang@gmail.com & yding25@binghamton.edu 
# @Description    : Move end effector to targeted pose
# @Usage          : python move_eef_to_pose.py 192.168.1.208
"""

import argparse
import time
from Robotics_API import Bestman_Real_Xarm7, Pose
import numpy as np

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
        bestman = Bestman_Real_Xarm7(args.robot_ip)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        # Define the target trajectory
        angles_rad = np.radians([-84.09874516930229, -83.31184493346971, -98.6411015590766])
        mvpose = Pose([0.5414959720000001, -0.0031613120000000003, 0.302334076], angles_rad)
        # target_pose = Pose([0.420434235, -0.00581615, 0.459876038], [0.7071068218077394, -0.00032550013355177364, 0.7071065907288765, 0.0003255002399235735]) # 四元数

        # Move the arm to follow the target trajectory
        bestman.move_eef_to_goal_pose(mvpose)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
