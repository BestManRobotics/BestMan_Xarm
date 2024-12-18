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
import sys
import time
import numpy as np
from .Pose import Pose
from scipy.spatial.transform import Rotation as R
import ikpy
from ikpy.chain import Chain
from ikpy.link import URDFLink
from xarm.wrapper import XArmAPI
import logging
import math


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

    def __init__(self, robot_ip, local_ip=None, frequency=None):
        """
        Initializes the xArm robot and related components.

        Args:
            robot_ip (str): The IP address of the xArm robot.
        """
        # Robot initialization
        self.robot = XArmAPI(robot_ip)
        self.gripper = None
        self.local_ip = local_ip
        self.frequency = frequency
        self.robot.set_mode(0)  # Default mode

        # Parse URDF for kinematic chain
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(current_dir, "../Asset/xarm6_robot.urdf")

        if not os.path.exists(urdf_file):
            raise FileNotFoundError(f"URDF file not found: {urdf_file}")

        # Parse the URDF file and configure the kinematic chain
        self.robot_chain = Chain.from_urdf_file(urdf_file, base_elements=["world"])

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

        # Configure the logger (simplified setup)
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[
                logging.StreamHandler(),  # Output to console
                logging.FileHandler("xarm_command_log.log"),  # Output to file
            ],
        )
        self.log = logging.getLogger("XArmRobotController")

    # ----------------------------------------------------------------
    # Device Initialization and State Management
    # ----------------------------------------------------------------
    
    

    def initialize_robot(self):
        """
        Initializes the robot by clearing faults and enabling it.

        Returns:
            bool: True if the robot is successfully initialized, False otherwise.
        """
        try:
            # Step 1: Clear faults and warnings
            self.log.info("Checking for faults on the robot...")
            self.robot.clean_error()  # Clear errors
            self.robot.clean_warn()  # Clear warnings
            time.sleep(3)  # Wait for a short duration to allow the robot to reset
            self.log.info(f"Successfully initialize the robot")
            return True

        except Exception as e:
            self.log.error(f"Failed to initialize the robot: {str(e)}")
            return False

    def update_robot_states(self):
        """
        Fetches and updates the current robot states (joint angles and TCP pose in quaternion format).
        """
        try:
            # Retrieve joint states
            joint_states = self.robot.get_joint_states(is_radian=True)
            joint_angles = joint_states[1][0][:6]
            joint_velocities = joint_states[1][1][:6]
            
            # Retrieve end effector pose (Euler angles)
            success, tcp_pose_raw = self.robot.get_position(is_radian=True)
            if success != 0:
                raise ValueError("Failed to retrieve end-effector pose")
            
            # Convert position from millimeters to meters
            position = [value / 1000 for value in tcp_pose_raw[:3]]  # x, y, z in meters
            
            # Convert orientation from Euler angles to quaternion
            roll, pitch, yaw = tcp_pose_raw[3:6]  # Extract Euler angles
            orientation_quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
            
            # Store updated robot states
            self.robot_states = {
                "joint_angles": joint_angles,
                "joint_velocities": joint_velocities,
                "tcp_pose": {
                    "position": position,  # Position in meters
                    "orientation_quat": orientation_quat.tolist(),  # Quaternion (x, y, z, w)
                },
            }
            
            # Log and display TCP pose in quaternion format
            formatted_quaternion = ["%.2f" % i for i in orientation_quat]
            self.log.info(f"Robot states updated: {self.robot_states}")
            print("tcp_pose(quaternion):", formatted_quaternion)

        except Exception as e:
            self.log.error(f"Error updating robot states: {str(e)}")

    def go_home(self):
        """
        Moves the robot arm to its initial (home) pose.
        Converts input position (in meters) to millimeters for the xArm API,
        and quaternion orientation to Euler angles.
        """
        position = [0.420, -0.0055, 0.36]  # x, y, z in meters
        orientation_quat = [
            0.7071065119402606,
            -0.0006170669964236503,
            0.7071065119402605,
            0.0006170669964236504,
        ]  # Identity quaternion (no rotation)
        pose = Pose(position, orientation_quat)

        position_mm = [
            p * 1000 for p in position
        ]  # Convert position to millimeters for xArm API

        euler_orientation = pose.get_orientation(
            type="euler"
        )  # Convert to [roll, pitch, yaw]
        euler_orientation_degree = np.degrees(
            euler_orientation
        )  # Convert radians to degrees for xArm API

        # Command the robot to move to the home pose
        self.robot.set_position(
            x=position_mm[0],
            y=position_mm[1],
            z=position_mm[2],
            roll=euler_orientation_degree[0],
            pitch=euler_orientation_degree[1],
            yaw=euler_orientation_degree[2],
        )

        self.log.info(f"Robot moved to home position")

    def set_pay_load(self, payload_weight, payload_cog):
        self.robot.set_tcp_load(payload_weight, payload_cog)

    # ----------------------------------------------------------------
    # Arm Functions
    # ----------------------------------------------------------------
    def get_joint_bounds(self):
        """
        Retrieves the joint limits of the xArm6 robot arm.

        Returns:
            list: A list of tuples, where each tuple contains the minimum and maximum value for each joint.
        """
        joint_min = [-3.14, -1.57, -3.14, -1.57, -3.14, -1.57]
        joint_max = [3.14, 2.09, 3.14, 2.09, 3.14, 2.09]

        return list(zip(joint_min, joint_max))

    def print_arm_jointInfo(self):
        """
        Prints the joint and link information of the robot's arm.

        Returns:
            None
        """
        print("Arm joint and link information:")
        for i, link in enumerate(
            self.robot_chain.links[1:]
        ):  # Assuming arm starts at the second link
            print(f"Link {i + 1}: {link.name}")
            self.log.info(f"Link {i + 1}: {link.name}")

    def get_joint_idx(self):
        """
        Retrieves the indices of the joints in the robot arm.

        Returns:
            list: A list of indices for the joints in the robot arm.
        """
        return list(range(len(self.active_joints)))

    def get_DOF(self):
        """
        Retrieves the degree of freedom (DOF) of the xArm robot.

        Returns:
            int: The number of degrees of freedom.
        """
        dof = 6  # xArm6 is a 6-DOF robotic arm
        self.log.info(f"Robot DOF: {dof}")
        return dof

    def get_tcp_link(self):
        """
        Retrieves the name of the TCP (Tool Center Point) link.

        Returns:
            str: Name of the TCP link.
        """
        try:
            tcp_link_name = self.robot_chain.links[-1].name
            self.log.info(f"TCP link retrieved: {tcp_link_name}")
            return tcp_link_name
        except IndexError as e:
            self.log.error(f"Failed to retrieve TCP link: {str(e)}")
            return None

    def get_current_joint_values(self):
        """
        Retrieves the current joint angles of the robot arm.

        Returns:
            list[float]: A list of the current joint angles of the robot arm (in radians).
        """
        try:
            # Get joint states and extract joint angles for the first 6 joints
            joint_states = self.robot.get_joint_states(is_radian=True)
            joint_values = joint_states[1][0][:6]
            self.log.info(f"Current joint values: {joint_values}")
            return joint_values
        except Exception as e:
            self.log.error(f"Failed to retrieve current joint values: {str(e)}")
            return []

    def get_current_joint_velocities(self):
        """
        Retrieves the current joint velocities of the robot arm.

        Returns:
            list[float]: A list of the current joint velocities of the robot arm (in radians per second).
        """
        try:
            # Get joint states and extract joint velocities for the first 6 joints
            joint_states = self.robot.get_joint_states(is_radian=True)
            joint_velocities = joint_states[1][1][:6]
            self.log.info(f"Current joint velocities: {joint_velocities}")
            return joint_velocities
        except Exception as e:
            self.log.error(f"Failed to retrieve current joint velocities: {str(e)}")
            return []

    def get_current_eef_pose(self):
        """
        Retrieves the current pose of the robot arm's end effector.

        Returns:
            Pose: A Pose object representing the end effector's position and orientation.
        """
        try:
            success, raw_pose = self.robot.get_position(is_radian=True)
            if success != 0:
                raise ValueError(
                    f"Failed to retrieve end effector pose, error code: {success}"
                )
            # Convert position values from millimeters to meters
            position = [
                raw_pose[0] / 1000,  # x in meters
                raw_pose[1] / 1000,  # y in meters
                raw_pose[2] / 1000,  # z in meters
            ]
            orientation = raw_pose[3:]  # roll, pitch, yaw (already in radians)
            pose = Pose(position, orientation)
            self.log.info(f"Current end effector pose: {pose}")
            return pose
        except Exception as e:
            self.log.error(f"Failed to retrieve end effector pose: {str(e)}")
            return None

    def move_arm_to_joint_values(
        self, joint_values, target_vel=None, target_acc=None, max_vel=None, max_acc=None
    ):
        """
        Moves the robotic arm to a specific set of joint values.

        Args:
            joint_values (list[float]): Target joint values in radians.
        """
        try:
            # Set robot to online joint control mode
            if self.robot.set_mode(6) != 0:
                self.log.error(
                    "Failed to set robot mode to 'online joint control mode'."
                )
                return
            if self.robot.set_state(0) != 0:
                self.log.error("Failed to set robot state to operational.")
                return

            # Execute joint angle command
            result = self.robot.set_servo_angle(
                angle=joint_values,
                is_radian=True,
                speed=0.7,  # Fixed speed in radians per second
                wait=False,  # Disable this function
            )

            if result != 0:
                self.log.error(
                    f"Failed to move arm to joint values: {joint_values}. Error code: {result}"
                )
            else:
                self.log.info(f"Moving arm to joint values: {joint_values}")

        except Exception as e:
            self.log.error(
                f"An error occurred while moving arm to joint values: {str(e)}"
            )

    # TODO
    def get_current_tcp_speed(self):
        """
        Retrieves the current tcp velocities of the robot arm.

        Returns:
            list: A list of the current tcp velocities of the robot arm.
        """
        speed = self.robot.realtime_tcp_speed
        return speed

    # ----------------------------------------------------------------
    # End Effector (EEF) Functions
    # ----------------------------------------------------------------

    def move_eef_to_goal_pose(self, goal_pose, max_linear_vel=100, max_angular_vel=5000):
        """
        Move arm's end effector to a target position.

        Args:
            goal_pose (Pose): The desired pose of the end effector (includes both position in mm and euler_orientation), please use radian for orientaiton.
        """
        try:
            # Set robot to online joint control mode
            if self.robot.set_mode(7) != 0:
                self.log.error(
                    "Failed to set robot mode to 'online joint control mode'."
                )
                return
            if self.robot.set_state(0) != 0:
                self.log.error("Failed to set robot state to operational.")
                return
            
            if not isinstance(goal_pose, Pose):
                raise TypeError("pose must be an instance of Pose")
            
            position = goal_pose.get_position()
            orientation = goal_pose.get_orientation(type="euler")

            self.robot.set_position(
                x=position[0] * 1000,
                y=position[1] * 1000,
                z=position[2] * 1000,
                roll=orientation[0],
                pitch=orientation[1],
                yaw=orientation[2],
                speed=max_linear_vel,
                mvacc=max_angular_vel,
                is_radian=True,
                wait=False,
            )
            self.log.info(f"Moved end effector to pose: {goal_pose.position}, {goal_pose.orientation}")

        except Exception as e:
            self.log.error(f"Failed to move end effector to goal pose: {str(e)}")

    def move_eef_to_tcp_velocity(self, _velocity_setpoint, _duration):
        #TODO: No test
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

    def rotate_eef_tcp(self, axis, angle):
        # TODO: No Test
        """
        Rotate the end effector of the robot arm by a specified angle by joint.

        Args:
            angle (float): The desired rotation angle in radians.
        """
        current_joint_values = self.get_current_joint_values()

        target_joint_values = current_joint_values.copy()
        target_joint_values[6] += angle
        target_vel = 1
        self.robot.set_mode(6)  # 0: joint control mode; 6: online joint
        self.robot.set_state(0)

        self.robot.set_servo_angle(
            angle=target_joint_values, is_radian=True, speed=target_vel, wait=False
        )  # speed in rad/s

    # ----------------------------------------------------------------
    # Functions for IK
    # ----------------------------------------------------------------

    # def joints_to_cartesian(self, joint_values):
    #     """
    #     Converts the robot's joint angles to its Cartesian coordinates.

    #     Args:
    #         joint_values (list[float]): A list of joint angles for the robot arm.

    #     Returns:
    #         Pose: A Pose object representing the Cartesian position and quaternion orientation.

    #     Raises:
    #         ValueError: If the number of joint values does not match the number of active joints.
    #     """
    #     try:
    #         if len(joint_values) != len(self.active_joints):
    #             raise ValueError(
    #                 "The number of joint values does not match the number of active joints."
    #             )

    #         full_joint_values = np.zeros(len(self.robot_chain.links))
    #         active_joint_indices = [
    #             self.robot_chain.links.index(joint) for joint in self.active_joints
    #         ]

    #         for i, joint_value in enumerate(joint_values):
    #             full_joint_values[active_joint_indices[i]] = joint_value

    #         cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_values)
    #         position = cartesian_matrix[:3, 3]
    #         orientation_matrix = cartesian_matrix[:3, :3]
    #         quaternion = R.from_matrix(orientation_matrix).as_quat()

    #         self.log.info(f"Converted joint values {joint_values} to Cartesian Pose {position} and {quaternion}.")

    #         def calculate_new_pose(x, y, z, quaternion, distance = -0.1703):
    #             # Step 1: 根据 roll, pitch, yaw 计算旋转矩阵
    #             rotation = R.from_quat(quaternion)
    #             rotation_matrix = rotation.as_matrix()
    #             # Step 2: 提取旋转矩阵的 z 轴方向
    #             z_axis = rotation_matrix[:, 2]  # 第三列就是 z 轴方向
    #             new_position = np.array([x, y, z]) - distance * z_axis
    #             # Step 3: 返回新的位姿 (新位置 + 原来的姿态)
    #             return [new_position[0], new_position[1], new_position[2]], quaternion
                        
    #         position, quaternion= calculate_new_pose(position[0],position[1],position[2], quaternion)
    #         print('nnn',position)

    #         return Pose(position, quaternion)
    #     except Exception as e:
    #         self.log.error(f"Error converting joint values to Cartesian: {str(e)}")
    #         raise
    
    def joints_to_cartesian(self, joint_values):
        """
        Converts the robot's joint angles to its Cartesian coordinates.

        Args:
            joint_values (list[float]): A list of joint angles for the robot arm.

        Returns:
            Pose: A Pose object representing the Cartesian position and quaternion orientation.

        Raises:
            ValueError: If the number of joint values does not match the number of active joints.
        """
        try:
            if len(joint_values) != len(self.active_joints):
                raise ValueError(
                    "The number of joint values does not match the number of active joints."
                )

            full_joint_values = np.zeros(len(self.robot_chain.links))
            active_joint_indices = [
                self.robot_chain.links.index(joint) for joint in self.active_joints
            ]

            for i, joint_value in enumerate(joint_values):
                full_joint_values[active_joint_indices[i]] = joint_value

            cartesian_matrix = self.robot_chain.forward_kinematics(full_joint_values)
            position = cartesian_matrix[:3, 3]
            orientation_matrix = cartesian_matrix[:3, :3]
            quaternion = R.from_matrix(orientation_matrix).as_quat()

            # Add the gripper length along the end-effector's Z-axis
            gripper_length = 0.17  # 17cm; adjust as needed
            gripper_offset = gripper_length * orientation_matrix[:, 2]  # Z-axis of the end-effector
            adjusted_position = position + gripper_offset

            self.log.info(f"Converted joint values {joint_values} to Cartesian Pose {position} and {quaternion}.")
            return Pose(adjusted_position, quaternion)
        except Exception as e:
            self.log.error(f"Error converting joint values to Cartesian: {str(e)}")
            raise
    
    # def cartesian_to_joints(self, pose, initial_joint_angles=None):
    #     position = pose.position
    #     orientation = pose.orientation

    #     def calculate_new_pose(x, y, z, quaternion, distance = 0.1703):
    #         # Step 1: 根据 roll, pitch, yaw 计算旋转矩阵
    #         rotation = R.from_quat(quaternion)
    #         rotation_matrix = rotation.as_matrix()
    #         # Step 2: 提取旋转矩阵的 z 轴方向
    #         z_axis = rotation_matrix[:, 2]  # 第三列就是 z 轴方向
    #         new_position = np.array([x, y, z]) - distance * z_axis
    #         # Step 3: 返回新的位姿 (新位置 + 原来的姿态)
    #         return [new_position[0], new_position[1], new_position[2]], quaternion
        
    #     position, quaternion= calculate_new_pose(position[0],position[1],position[2], orientation)
        
    #     my_chain = ikpy.chain.Chain.from_urdf_file("../Asset/xarm6_robot.urdf", base_elements=['world'])
    #     """
    #     将机械臂的笛卡尔坐标转换为关节角度。
    #     """
    #     rotation = R.from_quat(orientation)
    #     rotation_matrix = rotation.as_matrix()

    #     if initial_joint_angles is None:
    #         # initial_joint_angles = [0, 0, -0.02792803756892681, -0.47473636269569397, -0.0384012907743454, -0.022689493373036385, -1.057666301727295, 0.0034878887236118317]
    #         initial_joint_angles = [0, 0] + self.get_current_joint_values()

    #     joint_angles = my_chain.inverse_kinematics(
    #         position,
    #         rotation_matrix,
    #         orientation_mode='all',
    #         initial_position=initial_joint_angles)

    #     print('4',joint_angles)
    #     return joint_angles


    def cartesian_to_joints(self, pose, initial_joint_values=None):
        """
        Converts the robot's Cartesian coordinates to its joint angles.

        Args:
            position (list[float]): Cartesian position of the robot arm.
            orientation (list[float]): Quaternion orientation of the robot arm.

        Returns:
            list[float]: Joint angles corresponding to the given Cartesian coordinates.

        Raises:
            ValueError: If the solution is invalid or out of bounds.
        """
        try:
            orientation = pose.orientation
            rotation = R.from_quat(orientation) # qx, qy, qz, qw
            rotation_matrix = rotation.as_matrix() 
            
            target_pose = np.eye(4)
            target_pose[:3, :3] = rotation_matrix
            # Adjust position by subtracting the gripper length along the end-effector Z-axis
            gripper_length = 0.17  # Gripper length in meters (15cm open, 17cm closed)
            target_pose[:3, 3] = pose.position - gripper_length * rotation_matrix[:, 2]

            if initial_joint_values is None:
                initial_joint_values = [0, 0] + self.get_current_joint_values() # 不同的urdf，[0]位置不一样

            joint_values = ikpy.inverse_kinematics.inverse_kinematic_optimization(
                chain=self.robot_chain,
                target_frame=target_pose,
                starting_nodes_angles=initial_joint_values,
                orientation_mode="all",
            )
            self.log.info(f"Converted Cartesian position {pose.position} to joint values {joint_values}.")
            return joint_values[2:9]
        except Exception as e:
            self.log.error(f"Error converting Cartesian to joint values: {str(e)}")
            raise


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
