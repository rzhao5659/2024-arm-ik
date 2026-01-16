import math
from typing import List

import numpy as np

from arm_ik.angles_utility import (
    map_joint_angles_from_physical_to_software,
    map_joint_angles_from_software_to_physical,
)
from arm_ik.solvers import ArmSolver

import rclpy
from rclpy.node import Node

from arm_msgs.msg import (
    ArmCommand,
    ArmStatus,
    ControlMode,
    JointCommand,
    JointStatus,
    ModelArmState,
)

# How it works:
# 1. Take model arm state (q1,q2,q3)
# 2. Take arm/status/all to compute the current orientation of end effector R.
# 3. Solve orientation kinematics alone:
#    keep (q1,q2,q3) same so we get R_{03}.
#    Solve for R_{36} = R_{03}^T * R that maintains same ee orientation R.


class ConstantEEIKNode(Node):
    def __init__(self):
        super().__init__("constant_ee_ik_node")
        self.mode = ControlMode.MODE_STOP

        # Subscribers
        self.arm_state_sub = self.create_subscription(
            ArmStatus, "/arm/status/all", self.arm_state_cb, 1
        )
        self.model_arm_state_sub = self.create_subscription(
            ModelArmState, "/arm/model/raw", self.model_arm_state_cb, 1
        )
        self.mode_sub = self.create_subscription(ControlMode, "/arm/mode/current", self.mode_cb, 1)

        # Publishers:
        # The command will be published whenever we receive a new ModelArmState.
        self.command_pub = self.create_publisher(ArmCommand, "/arm/command/ik_constant_ee", 1)

        # Private variables:
        self.solver = ArmSolver()
        self.arm_pos_valid = False
        self.joints_names = [
            "turret",
            "shoulder",
            "elbow",
            "elbow_roll",
            "wrist_pitch",
            "wrist_roll",
        ]

        # The program won't use these initialized values as arm_pos_valid is False.
        # arm_joint_values (/arm/status/all) is needed to compute the current
        # orientation R.
        self.arm_joint_values = np.zeros(6)

    def mode_cb(self, msg: ControlMode) -> None:
        """Store the current mode"""
        self.mode = msg.mode

    def _validate_arm_status(self, msg: ArmStatus, joints_to_validate: List[str]) -> bool:
        """
        Validate arm status for a specific selection of joints.
        """
        arm_status_is_valid = True
        for i in range(len(joints_to_validate)):
            joint: JointStatus = getattr(msg, joints_to_validate[i])
            if joint.status != JointStatus.STATUS_OK:
                arm_status_is_valid = False
                break
        # Only true if all received joints status in the list `joints_to_validate` are
        # known.
        return arm_status_is_valid

    def arm_state_cb(self, msg: ArmStatus):
        """
        Store the arm joint values (which are in deg) in self.arm_joint_values (in rad).
        If one of the joint's status is unknown, consider the received arm configuration
        to be invalid (self.arm_pos_valid set to False)
        """
        # Validate all status of all joints
        self.arm_pos_valid = self._validate_arm_status(msg, self.joints_names)
        if self.arm_pos_valid is False:
            return

        # Extract arm joint values
        new_arm_joint_values = np.zeros(6)
        for i in range(len(self.joints_names)):
            joint = getattr(msg, self.joints_names[i])
            new_arm_joint_values[i] = joint.position

        # Map them to software range
        new_arm_joint_values = np.deg2rad(new_arm_joint_values)
        new_arm_joint_values = map_joint_angles_from_physical_to_software(new_arm_joint_values)

        # Store
        self.arm_joint_values = new_arm_joint_values
        if self.mode != ControlMode.MODE_IK_CONSTANT_EE:
            self.goal_arm_R = self.solver.fk_solve(np.array(self.arm_joint_values))[0:3, 0:3]

    def model_arm_state_cb(self, msg: ModelArmState):
        """
        Publish the Constant EE command, computed from model arm state and current arm state.
        """
        # Validate real arm status.
        if self.arm_pos_valid is False:
            return

        # Receive model arm status, assumed to be valid.
        # TODO: Add joint status validation
        # model_arm_joints = self.joints_names[0:3]
        # model_arm_status_is_valid = self._validate_arm_status(msg, model_arm_joints)
        model_arm_joint_values = map_joint_angles_from_physical_to_software(
            np.array(
                [
                    math.radians(msg.turret),
                    math.radians(msg.shoulder),
                    math.radians(msg.elbow - msg.shoulder),
                    0,
                    0,
                    0,
                ]
            )
        )
        model_arm_joint_values = model_arm_joint_values[0:3]

        # Compute ConstantEEIK command:
        # Denoting q1-q6 as desired joint angles.
        # q1-q3 is simply the model arm joint values.
        # q4-q6 is obtained from inverse orientation kinematics.
        curr_arm_wrist_angles = np.array(self.arm_joint_values[3:6])
        current_joint_angles = np.hstack((model_arm_joint_values, curr_arm_wrist_angles))

        solved_joint_angles = self.solver.ik_solve_orientation_only(
            goal_orientation=self.goal_arm_R, curr_joint_angles=current_joint_angles
        )

        if solved_joint_angles is not None:
            solved_joint_angles = map_joint_angles_from_software_to_physical(
                solved_joint_angles, current_joint_angles
            )
            solved_joint_angles = np.rad2deg(solved_joint_angles)
            self.publish_ik_command(solved_joint_angles)

    def publish_ik_command(self, goal_joint_values: np.ndarray) -> None:
        """Create a ArmCommand msg and publish it"""
        msg = ArmCommand()
        for i in range(6):
            joint = getattr(msg, self.joints_names[i])
            joint.command_type = JointCommand.COMMAND_TYPE_POSITION
            joint.value = goal_joint_values[i]
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    constant_ee_local_ik_node = ConstantEEIKNode()
    rclpy.spin(constant_ee_local_ik_node)
    rclpy.shutdown()
