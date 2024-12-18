# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : display_states.py
# @Time           : 2024-12-01 15:21:45
# @Author         : Zhaxi & Yan 
# @Email          : zhaxizhuoma.ayang@gmail.com & yding25@binghamton.edu 
# @Description    : Display arm state
# @Usage          : python display_states.py 192.168.1.208
"""

import argparse
import time
from Robotics_API import Bestman_Real_Xarm6, Pose

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

        bestman.update_robot_states()

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
