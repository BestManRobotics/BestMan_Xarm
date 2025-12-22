'''
Run this script using:

python3 move_arm_to_home.py 192.168.1.208
'''

import sys
import os
from Bestman_real_xarm6 import *
from time import time, sleep

def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Xarm6.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    args = argparser.parse_args()

    try:
        # Instantiate the robot interface
        bestman = Bestman_Real_Xarm6(args.robot_ip, None, None)

        # Go back to home pose
        bestman.go_home()
        sleep(1)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
