'''
Run this script using:

python3 move_arm_to_home.py 192.168.1.208
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
# import pyRobotiqGripper
from Bestman_real_xarm6 import *
from time import time, sleep

def main():
    # Parse Arguments
    argparser = argparse.ArgumentParser(description="Xarm6.")
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    # argparser.add_argument("local_ip", help="IP address of this PC")
    # argparser.add_argument("frequency", type=int, help="Command frequency, 1 to 200 [Hz]")
    # Optional arguments
    # argparser.add_argument("--hold", action="store_true", help="Robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    try:
        # Instantiate the robot interface
        bestman = Bestman_Real_Xarm6(args.robot_ip, None, None)

        # Clear fault on the robot server if any
        # if bestman.robot.isFault():
        #     log.warn("Fault occurred on the robot server, trying to clear ...")
        #     bestman.robot.clearFault()
        #     time.sleep(2)
        #     if bestman.robot.isFault():
        #         log.error("Fault cannot be cleared, exiting ...")
        #         return
        #     log.info("Fault on the robot server is cleared")

        # Enable the robot, ensuring the E-stop is released before enabling
        # log.info("Enabling robot ...")
        # bestman.robot.enable()

        # Go back to home pose
        bestman.reset_to_home()
        sleep(1)

    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
