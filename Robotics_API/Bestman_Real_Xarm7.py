# !/usr/bin/env python
# -*- coding: utf-8 -*-
"""
# @FileName       : Bestman_real_xarm7.py
# @Description    : Minimal extension for xArm7 based on Bestman_Real_Xarm6
"""

import os
from .Bestman_Real_Xarm6 import Bestman_Real_Xarm6
from ikpy.link import URDFLink
from scipy.spatial.transform import Rotation as R

class Bestman_Real_Xarm7(Bestman_Real_Xarm6):
    def __init__(self, robot_ip, local_ip=None, frequency=None):
        # Call parent constructor
        super().__init__(robot_ip, local_ip, frequency)

        # Override: Load xArm7 URDF
        # current_dir = os.path.dirname(os.path.abspath(__file__))

        # urdf_file = os.path.join(current_dir, "../Asset/xarm7_robot.urdf")
        # if not os.path.exists(urdf_file):
        #     raise FileNotFoundError(f"URDF file not found: {urdf_file}")
        # self.robot_chain = self.robot_chain.__class__.from_urdf_file(urdf_file, base_elements=["world"])

        # Set active joints mask for 7-DOF
        # active_links_mask = [False] + [False] + [True] * 7
        # self.robot_chain.active_links_mask = active_links_mask

        # # Update active joints
        # self.active_joints = [
        #     joint
        #     for joint in self.robot_chain.links
        #     if isinstance(joint, URDFLink)
        #     and (joint.joint_type == "revolute" or joint.joint_type == "prismatic")
        # ]

        # Validate DOF
    #     if len(self.active_joints) != 7:
    #         raise ValueError(
    #             f"Expected 7 active joints, but found {len(self.active_joints)}. "
    #             f"Check the URDF file and active_links_mask."
    #         )
    # Override Dofs!
    def update_robot_states(self):
        """
        Fetches and returns the current robot states (joint angles and TCP pose in quaternion format) for xArm7.
        """
        try:
            joint_states = self.robot.get_joint_states(is_radian=True)
            joint_values = joint_states[1][0][:7]  # xArm7: 7 joints
            joint_velocities = joint_states[1][1][:7]  # xArm7: 7 velocities

            success, tcp_pose = self.robot.get_position(is_radian=True)
            if success != 0:
                raise ValueError("Failed to retrieve end-effector pose")

            position = [value / 1000 for value in tcp_pose[:3]]
            orientation = R.from_euler('xyz', tcp_pose[3:6]).as_quat()

            robot_states = {
                "joint_values": joint_values,
                "tcp_pose (quaternion)": {
                    "position": position,
                    "orientation": orientation.tolist(),
                },
            }

            self.log.info(f"[XArm7] Robot states updated: {robot_states}")
            return robot_states

        except Exception as e:
            self.log.error(f"[XArm7] Error updating robot states: {str(e)}")

    def get_dof(self):
        """Override DOF to 7."""
        dof = 7
        self.log.info(f"Robot DOF: {dof}")
        return dof

    def get_joint_bounds(self):
        """Override joint limits for 7-DOF."""
        joint_min = [-3.14] * 7
        joint_max = [3.14] * 7
        return list(zip(joint_min, joint_max))
