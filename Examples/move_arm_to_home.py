# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : move_arm_to_home.py
# @Time           : 2024-12-01 15:21:45
# @Author         : Zhaxi & Yan 
# @Email          : zhaxizhuoma.ayang@gmail.com & yding25@binghamton.edu 
# @Description    : Move arm to default home position
# @Usage          : python move_arm_to_home.py 192.168.1.208
"""

import argparse
import time
from Robotics_API import Bestman_Real_Xarm7

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

        # Go back to home pose
        bestman.reset_to_home()
        time.sleep(1)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
