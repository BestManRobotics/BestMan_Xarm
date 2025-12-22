'''
Run this script using:

python3 move_arm_to_joint_angles.py 192.168.1.208
'''

import sys
import os
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import argparse
from RoboticsToolBox.Bestman_real_xarm6 import Bestman_Real_Xarm6
from time import time, sleep
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
        bestman = Bestman_Real_Xarm6(args.robot_ip)

        # Initialize robot
        if not bestman.initialize_robot():
            return  # Exit if initialization fails

        import math
        target_joint = [16.8, -24.1, -24.7, 11.2, -77.7, -5.1]
        target_joint = [math.radians(target_joint[n]) for n in range(6)]
        bestman.move_arm_to_joint_values(target_joint)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
