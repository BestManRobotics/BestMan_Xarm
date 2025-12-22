import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
# import pyRobotiqGripper
from Bestman_real_xarm6 import *
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

        # Define the target trajectory
        target_pose = [0.420434235, -0.00581615, 0.459876038, 180, -90, 0] # 四元数

        # Move the arm to follow the target trajectory
        bestman.move_eef_to_goal_pose(target_pose, is_radian=False)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
