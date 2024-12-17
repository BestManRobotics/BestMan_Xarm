# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : Bestman_real_xarm6.py
# @Time           : 2024-12-17 22:22:23
# @Author         : Zhaxi & Yan 
# @Email          : zhaxizhuoma.ayang@gmail.com & yding25@binghamton.edu 
# @Description    : XXX
# @Usage          : XXX
"""


import os
import time
import numpy as np
from .Pose import Pose
from scipy.spatial.transform import Rotation as R
import ikpy
from ikpy.chain import Chain
from ikpy.link import URDFLink
from xarm.wrapper import XArmAPI


class Bestman_Real_Xarm6:
    """
    A class to interface with the XArm6 robotic arm.

    Attributes:
        robot: The XArm6 API object initialized with the provided IP address.
        frequency: The control frequency for the robot.
        log: A logger object to log messages for debugging and monitoring.
        robot_states: A container for storing the current state of the robot.
        mode: Stores the current operation mode.
        gripper: The gripper object for controlling the end effector.
    """

    def __init__(self, robot_ip, local_ip, frequency):
        """
        Initializes the xArm robot and related components.

        Args:
            robot_ip (str): The IP address of the xArm robot.
            frequency (float): Control frequency for the robot.
        """
        # Robot initialization
        self.robot = XArmAPI(robot_ip)
        self.gripper = None
        self.frequency = frequency
        self.log = self.robot.logger
        self.mode = self.robot.set_mode(0)  # Default mode
        self.robot_states = self.robot.set_state(0)

        # Parse URDF for kinematic chain
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(current_dir, "../Asset/xarm6_robot.urdf")

        if not os.path.exists(urdf_file):
            raise FileNotFoundError(f"URDF file not found: {urdf_file}")

        # Parse the URDF file and configure the kinematic chain
        self.robot_chain = Chain.from_urdf_file(urdf_file)

        # Define active links mask for xArm6 (Base + 6 DOF + End Effector)
        active_links_mask = [False] + [True] * 6 + [False]  # Adjust for 6-DOF robot
        self.robot_chain.active_links_mask = active_links_mask

        # Filter active joints (revolute or prismatic)
        self.active_joints = [
            joint
            for joint in self.robot_chain.links
            if isinstance(joint, URDFLink)
            and (joint.joint_type == "revolute" or joint.joint_type == "prismatic")
        ]

        # Validate the number of active joints for xArm6
        if len(self.active_joints) != 6:  # xArm6 is a 6-DOF robot
            raise ValueError(
                f"Expected 6 active joints, but found {len(self.active_joints)}. "
                f"Check the URDF file and active_links_mask."
            )

        # Log successful initialization
        self.log.info("xArm6 robot kinematic chain successfully initialized.")

        # Additional initializations can go here
        self.log_file = "xarm_command_log.tum"

    # ----------------------------------------------------------------
    # Device Initialization and State Management
    # ----------------------------------------------------------------

    def initialize_robot(self):
        """
        Initializes the robot by clearing faults and enabling it.

        This function ensures the robot is operational by:
        1. Clearing any existing faults and warnings.
        2. Setting the robot state to operational (ready).
        3. Waiting until the robot becomes ready.

        Returns:
            bool: True if the robot is successfully initialized, False otherwise.
        """
        try:
            # Step 1: Clear faults and warnings
            self.log.info("Checking for faults on the robot...")
            self.robot.clean_error()  # Clear errors
            self.robot.clean_warn()   # Clear warnings
            time.sleep(1)  # Wait for a short duration to allow the robot to reset

            # Step 2: Set the robot to operational state
            self.log.info("Setting robot to operational state...")
            self.robot.set_state(0)  # Set state to '0' (Ready)
            time.sleep(1)  # Wait for state transition

            # Step 3: Check for ready state
            for seconds_waited in range(10):
                if self.robot.state == 0:  # State '0' indicates Ready
                    self.log.info("Robot is now operational.")
                    return True
                time.sleep(1)

            self.log.warn("Robot is not operational after 10 seconds.")
            return False

        except Exception as e:
            self.log.error(f"Failed to initialize the robot: {str(e)}")
            return False


    def update_robot_states(self):
        """
        Updates the current robot states by fetching them from the robot.
        """
        self.robot.getRobotStates(self.robot_states)

    def go_home(self, dist=0):
        """
        Moves the robot arm to its initial (home) pose.
        """
        self.robot.set_position(
            x=396.4 + dist, y=-5.5, z=360, roll=-90, pitch=-90, yaw=-90, wait=False
        )
        time.sleep(3)

    def set_mode(self, _mode):
        """
        Parameters:
        _mode=
            0: position controlmode
            1: servo motionmode
            2: joint teachingmodemode (invalid)3: cartesian teaching
            4: joint velocity control mode
            5: cartesian velocity control mode
            6: joint online trajectory planningmode
            7: cartesian online trajectory planning

        Notes:
            https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md#mode
        """
        print("set mode")
        self.robot.set_mode(_mode)

    def pose_to_euler(self, pose):
        """
        Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.

        Parameters:
        pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]

        Returns:
        tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
        """
        x, y, z, qw, qx, qy, qz = pose
        r = R.from_quat(
            [qx, qy, qz, qw]
        )  # Reordering to match scipy's [qx, qy, qz, qw]
        roll, pitch, yaw = r.as_euler("xyz", degrees=False)
        return [x, y, z, roll, pitch, yaw]

    def euler_to_pose(self, position_euler):
        """
        Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].

        Parameters:
        position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]

        Returns:
        list: [x, y, z, qw, qx, qy, qz]
        """
        x, y, z, roll, pitch, yaw = position_euler
        r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
        return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]

    def set_pay_load(self, payload_weight, payload_cog):

        self.robot.set_tcp_load(payload_weight, payload_cog)

    # ----------------------------------------------------------------
    # Functions for others
    # ----------------------------------------------------------------

    def get_joint_bounds(self):
        """
        Retrieves the joint bounds of the robot arm.

        Returns:
            list: A list of tuples representing the joint bounds, where each tuple contains the minimum and maximum values for a joint.
        """
        maxbounds = self.robot.info().qMax
        minbounds = self.robot.info().qMin
        jointbounds = list(zip(maxbounds, minbounds))
        return jointbounds

    def get_DOF(self):
        """
        Retrieves the degree of freedom (DOF) of the robot arm.

        Returns:
            int: The degree of freedom of the robot arm.
        """
        return 6

    # ----------------------------------------------------------------
    # Functions for joint control
    # ----------------------------------------------------------------

    def print_joint_link_info(self, name):
        """
        Prints the joint and link information of a robot.

        Args:
            name (str): 'base' or 'arm'
        """
        if name == "base":
            print("Base joint and link information:")
            for i, link in enumerate(
                self.robot_chain.links[:1]
            ):  # Assuming the base is the first link
                print(f"Link {i}: {link.name}")
        elif name == "arm":
            print("Arm joint and link information:")
            for i, link in enumerate(
                self.robot_chain.links[1:]
            ):  # Assuming the arm starts from the second link
                print(f"Link {i + 1}: {link.name}")

    def get_joint_idx(self):
        """
        Retrieves the indices of the joints in the robot arm.

        Returns:
            list: A list of indices for the joints in the robot arm.
        """
        return list(range(len(self.active_joints)))

    def get_tcp_link(self):
        """
        Retrieves the TCP (Tool Center Point) link of the robot arm.

        Returns:
            str: The TCP link of the robot arm.
        """
        return self.robot_chain.links[6].name

    def get_current_joint_angles(self):
        """
        Retrieves the current joint angles of the robot arm.

        Returns:
            list: A list of the current joint angles of the robot arm.
        """
        _joint_states = self.robot.get_joint_states(is_radian=True)
        _joint_angles = _joint_states[1][0][0:6]

        return _joint_angles

    def get_current_joint_velocities(self):
        """
        Retrieves the current joint velocities of the robot arm.

        Returns:
            list: A list of the current joint velocities of the robot arm.
        """

        _joint_states = self.robot.get_joint_states(is_radian=True)
        _joint_velocities = _joint_states[1][1][0:6]

        return _joint_velocities

    def move_arm_to_joint_angles(
        self,
        joint_angles,
        target_vel=None,
        target_acc=None,
        MAX_VEL=None,
        MAX_ACC=None,
        wait_for_finish=None,
    ):
        """
        Move arm to a specific set of joint angles, considering physics.

        Args:
            Set the servo angle, the API will modify self.last_used_angles value
        Note: https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md#def-set_servo_angleself-servo_idnone-anglenone-speednone-mvaccnone-mvtimenone-relativefalse-is_radiannone-waitfalse-timeoutnone-radiusnone-kwargs
        """

        #! Force mode switch
        self.robot.set_mode(6)  # 6: online joint
        self.robot.set_state(0)

        self.robot.set_servo_angle(
            angle=joint_angles, is_radian=True, speed=0.7, wait=wait_for_finish
        )  # speed in rad/s

    def move_arm_follow_joint_angles(
        self,
        target_trajectory,
        target_vel=None,
        target_acc=None,
        MAX_VEL=None,
        MAX_ACC=None,
    ):
        """
        Move arm to a few set of joint angles, considering physics.

        Args:
            target_trajectory: A list of desired joint angles (in radians) for each joint of the arm.
            target_vel: Optional. A list of target velocities for each joint.
            target_acc: Optional. A list of target accelerations for each joint.
            MAX_VEL: Optional. A list of maximum velocities for each joint.
            MAX_ACC: Optional. A list of maximum accelerations for each joint.
        """
        period = 1.0 / self.frequency
        self.update_robot_states()
        DOF = len(self.robot_states.q)

        for target_pos in target_trajectory:
            # Monitor fault on robot server
            if self.robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Send command
            self.robot.set_servo_angle(
                angle=target_pos, is_radian=True, speed=target_vel, wait=True
            )  # speed in rad/s
            print(f"Sent joint positions: {target_pos}")

            # Use sleep to control loop period
            time.sleep(period)

    # ----------------------------------------------------------------
    # Functions for end effector
    # ----------------------------------------------------------------

    # TODO
    def get_current_tcp_speed(self):
        """
        Retrieves the current tcp velocities of the robot arm.

        Returns:
            list: A list of the current tcp velocities of the robot arm.
        """
        speed = self.robot.realtime_tcp_speed
        return speed

    def get_current_eef_pose(self):
        """
        Retrieves the current pose of the robot arm's end effector.

        This function obtains the position and orientation of the end effector.

        Returns:
            pose: the [x, y, z, roll, pitch, yaw] value of tcp in meter and radian
        """

        _pose = self.robot.get_position(is_radian=True)
        pose = _pose[1]
        pose[0] = pose[0] / 1000
        pose[1] = pose[1] / 1000
        pose[2] = pose[2] / 1000
        return pose

    def move_eef_to_tcp_velocity(self, _velocity_setpoint, _duration):
        """
        Move arm's end effector to a target tcp velocity.

        Args:
            _velocity_setpoint: mm/s in transition, rad/s in orientation
            _duration: function calling loop time
        Notes:
            https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md#def-vc_set_cartesian_velocityself-speeds-is_radiannone-is_tool_coordfalse-duration-1-kwargs
        """

        self.robot.set_mode(5)  # 5 for cartesian vel
        self.robot.set_state(0)
        self.robot.vc_set_cartesian_velocity(
            speeds=[
                _velocity_setpoint[0],
                _velocity_setpoint[1],
                _velocity_setpoint[2],
                _velocity_setpoint[3],
                _velocity_setpoint[4],
                _velocity_setpoint[5],
            ],
            duration=_duration,
        )

    def move_eef_to_goal_pose(self, goal_pose, speed=1000, mvacc=50000, wait=False):
        """
        Move arm's end effector to a target position.

        Args:
            goal_pose (Pose): The desired pose of the end effector (includes both position in mm and euler_orientation), please use radian for orientaiton.
        """

        self.robot.set_position(
            x=goal_pose[0] * 1000,
            y=goal_pose[1] * 1000,
            z=goal_pose[2] * 1000,
            roll=goal_pose[3],
            pitch=goal_pose[4],
            yaw=goal_pose[5],
            speed=speed,
            mvacc=mvacc,
            is_radian=True,
            wait=wait,
        )

    def rotate_eef_joint(self, angle):
        """
        Rotate the end effector of the robot arm by a specified angle by joint.

        Args:
            angle (float): The desired rotation angle in radians.
        """
        current_joint_angles = self.get_current_joint_angles()

        target_joint_angles = current_joint_angles.copy()
        target_joint_angles[6] += angle
        target_vel = 1
        self.robot.set_mode(6)  # 0: joint control mode; 6: online joint
        self.robot.set_state(0)

        self.robot.set_servo_angle(
            angle=target_joint_angles, is_radian=True, speed=target_vel, wait=False
        )  # speed in rad/s

    # ----------------------------------------------------------------
    # Functions for IK
    # ----------------------------------------------------------------

    def joints_to_cartesian(self, joint_angles):
        """
        Transforms the robot arm's joint angles to its Cartesian coordinates.

        Args:
            joint_angles (list): A list of joint angles for the robot arm.

        Returns:
            tuple: A tuple containing the Cartesian coordinates (position and orientation) of the robot arm.
        """
        # Validate the number of joint values matches the number of active joints
        if len(joint_angles) != len(self.active_joints):
            raise ValueError(
                "The number of joint values does not match the number of active joints"
            )

        # Map joint values to the full joint chain
        full_joint_angles = np.zeros(len(self.robot_chain.links))
        active_joint_indices = [
            self.robot_chain.links.index(joint) for joint in self.active_joints
        ]

        for i, joint_value in enumerate(joint_angles):
            full_joint_angles[active_joint_indices[i]] = joint_value

        # Calculate the end effector position and orientation
        cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_angles)

        # Extract position and orientation
        position = cartesian_matrix[:3, 3]
        orientation_matrix = cartesian_matrix[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(orientation_matrix)
        quaternion = r.as_quat()
        orientation = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

        return position, orientation

    def cartesian_to_joints(self, position, orientation):
        """
        Transforms the robot arm's Cartesian coordinates to its joint angles.

        Args:
            position (list): The Cartesian position of the robot arm.
            orientation (list): The Cartesian orientation of the robot arm.

        Returns:
            list: A list of joint angles corresponding to the given Cartesian coordinates.
        """
        rotation_matrix = R.from_euler("xyz", orientation).as_matrix()

        # Combine rotation matrix and position into a list
        target_pose = np.eye(4)
        target_pose[:3, :3] = rotation_matrix
        target_pose[:3, 3] = position

        initial_joint_angles = [0] * len(self.robot_chain)

        # inverse kinematics calculations and return joint angles
        joint_angles = ikpy.inverse_kinematics.inverse_kinematic_optimization(
            chain=self.robot_chain,
            target_frame=target_pose,
            starting_nodes_angles=initial_joint_angles,
            orientation_mode="all",
        )

        return joint_angles[1:8]

    def calculate_IK_error(self, goal_position, goal_orientation):
        """
        Calculate the inverse kinematics (IK) error for performing pick-and-place manipulation of an object using a robot arm.

        Args:
            goal_position: The desired goal position for the target object.
            goal_orientation: The desired goal orientation for the target object.
        """
        pass

    # ----------------------------------------------------------------
    # Functions for gripper
    # ----------------------------------------------------------------

    def find_gripper_xarm(self):
        """
        Searches for the gripper on available serial ports and returns the port if found.

        Returns:
            str: The serial port where the gripper is connected, or None if not found.
        """
        _pos = self.robot.get_gripper_position()
        _ver = self.robot.get_gripper_version()

        if _ver is not None and _pos is not None:
            print("Have Xarm gripper", _ver)
            return True
        else:
            print("Not found Xarm gripper")
            return None

    def find_gripper_robotiq(self):
        """
        Config the parameter via Python SDK
        """
        # Baud rate
        # Modify the baud rate to 115200, the default is 2000000.
        self.robot.set_tgpio_modbus_baudrate(115200)

        # TCP Payload and offset
        # Robotiq 2F/85 Gripper
        self.robot.set_tcp_load(0.925, [0, 0, 58])
        self.robot.set_tcp_offset([0, 0, 174, 0, 0, 0])
        self.robot.save_conf()

        # Self-Collision Prevention Model
        # Robotiq 2F/85 Gripper
        self.robot.set_collision_tool_model(4)

        self.robot.robotiq_reset()
        self.robot.robotiq_set_activate()  # enable the robotiq gripper

    def get_gripper_position_xarm(self):
        """
        Get the position of the XArm gripper.
        """
        gripper_pos = self.robot.get_gripper_position()

        return gripper_pos[1]

    def get_gripper_position_robotiq(self, number_of_registers=3):
        """
        Reading the status of robotiq gripper

        :param number_of_registers: number of registers, 1/2/3, default is 3
            number_of_registers=1: reading the content of register 0x07D0
            number_of_registers=2: reading the content of register 0x07D0/0x07D1
            number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2

            Note:
                register 0x07D0: Register GRIPPER STATUS
                register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO
                register 0x07D2: Register POSITION and register CURRENT
        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        status = self.robot.robotiq_get_status(number_of_registers=number_of_registers)
        gripper_width = status[1][-2]
        return gripper_width

    def gripper_goto_xarm(self, value, speed=5000, force=None):
        """
        Moves the gripper to a specified position with given speed.

        Args:
            value (int): Position of the gripper. Integer between 0 and 800.
                        0 represents the open position, and 255 represents the closed position.
            speed (int): Speed of the gripper movement, between 0 and 8000.
            force (int): Not applicable for xarm gripper

        Note:
            - 0 means fully closed.
            - 800 means fully open.
        """
        self.robot.set_gripper_position(
            pos=value, speed=speed, wait=False, timeout=1, auto_enable=True
        )

    def gripper_goto_robotiq(
        self, pos, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs
    ):
        """
        Go to the position with determined speed and force.

        :param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.
        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotion motion complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True

        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        return self.robot.robotiq_set_position(
            pos, speed=speed, force=force, wait=wait, timeout=timeout, **kwargs
        )

    def open_gripper_xarm(self):
        """Opens the gripper to its maximum position with maximum speed and force."""
        self.gripper_goto(value=850, speed=5000, force=None)

    def open_gripper_robotiq(
        self, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs
    ):
        """
        Open the robotiq gripper

        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotiq motion to complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True

        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        return self.robot.robotiq_open(
            speed=speed, force=force, wait=wait, timeout=timeout, **kwargs
        )

    def close_gripper_xarm(self):
        """Closes the gripper to its minimum position with maximum speed and force."""
        self.gripper_goto(value=0, speed=5000, force=None)

    def close_gripper_robotiq(
        self, speed=0xFF, force=0xFF, wait=False, timeout=5, **kwargs
    ):
        """
        Close the robotiq gripper

        :param speed: gripper speed between 0 and 255
        :param force: gripper force between 0 and 255
        :param wait: whether to wait for the robotiq motion to complete, default is True
        :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True

        :return: tuple((code, robotiq_response))
            code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.
            robotiq_response: See the robotiq documentation
        """
        return self.robot.robotiq_close(
            speed=speed, force=force, wait=wait, timeout=timeout, **kwargs
        )
