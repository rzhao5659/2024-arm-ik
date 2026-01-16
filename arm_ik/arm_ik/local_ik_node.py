import math
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

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
    IKInput,
    JointCommand,
    JointStatus,
)

# Constants for the program
EE_FRAME = 1
BASE_FRAME = 0

# Configuration parameters
SENSITIVITY_POS = 0.003  # Max increment in translation (x,y,z).
SENSITIVITY_ORIENT = 0.25 * math.pi / 180  # Max increment in rotation (r, p, y).


class LocalIKNode(Node):
    def __init__(self):
        super().__init__("local_ik_node")

        # Subscribers
        self.arm_pos_sub = self.create_subscription(
            ArmStatus, "/arm/status/all", self.arm_position_cb, 1
        )
        self.mode_sub = self.create_subscription(ControlMode, "/arm/mode/current", self.mode_cb, 1)
        self.ikinput_state_sub = self.create_subscription(
            IKInput, "/arm/ik_input/filtered", self.ikinput_state_cb, 1
        )

        # Publishers:
        # The command will be published whenever we receive a new IKInput.
        self.command_pub = self.create_publisher(ArmCommand, "/arm/command/ik", 1)

        # Private variables:
        self.arm_pos_valid = False
        self.frame_for_increments = BASE_FRAME
        self.mode = ControlMode.MODE_STOP
        self.joints_names = [
            "turret",
            "shoulder",
            "elbow",
            "elbow_roll",
            "wrist_pitch",
            "wrist_roll",
        ]

        # Initial joint values are set to all 0.0 rad.
        # The program won't use these values if arm_pos_valid is False.
        self.current_joint_values = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # The reference pose (transformation matrix) for increments.
        # This will be the arm's pose as it enters IK mode.
        self.reference_T = None

        # Total increments from reference_T (meters and rad)
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.droll = 0.0
        self.dpitch = 0.0
        self.dyaw = 0.0

        self.solver = ArmSolver()

    def mode_cb(self, msg):
        """
        Store the current mode and initializes the reference pose when the arm enters IK mode.
        """

        if self.mode not in [ControlMode.MODE_IK_CHASSIS_FRAME] and msg.mode in [
            ControlMode.MODE_IK_CHASSIS_FRAME
        ]:
            self._initialize_reference_frame()
            self.frame_for_increments = BASE_FRAME
            self.get_logger().info("Frame changed to BASE_FRAME")

        elif self.mode not in [ControlMode.MODE_IK_GRIPPER_FRAME] and msg.mode in [
            ControlMode.MODE_IK_GRIPPER_FRAME
        ]:
            self._initialize_reference_frame()
            self.frame_for_increments = EE_FRAME
            self.get_logger().info("Frame changed to EE_FRAME")

        self.mode = msg.mode

    def _initialize_reference_frame(self) -> None:
        """
        Compute the current arm pose using FK and assign it as the reference frame for
        increments. This also reset the increments to 0.0,
        effectively restarting IK node. Fails if some joint's status is unknown.
        """
        if self.arm_pos_valid:
            # Map the arm's joint angles into the range that the solvers assume
            curr_joint_angles = map_joint_angles_from_physical_to_software(
                self.current_joint_values
            )
            self.reference_T = self.solver.fk_solve(curr_joint_angles)

            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.droll = 0.0
            self.dpitch = 0.0
            self.dyaw = 0.0
        else:
            self.get_logger().error(
                "Reference pose fail to initialize: some joint's status is unknown."
            )

    def arm_position_cb(self, msg):
        """
        Store the measured joint values (which are in deg) in self.current_joint_values

        This callback function has two behaviors depending on mode:
        - If in IK mode, it stores the measured joint values.
        - If not in IK mode, it stores the measured joint values and update reference frame.

        If one of the joint's status is unknown, consider the received arm configuration
        to be invalid (self.arm_pos_valid set to False)
        """
        arm_pos_valid = True
        for i in range(len(self.joints_names)):
            joint = getattr(msg, self.joints_names[i])
            if joint.status == JointStatus.STATUS_OK:
                self.current_joint_values[i] = math.radians(joint.position)
            else:
                arm_pos_valid = False
                break

        # Only true if all received joints status are known.
        self.arm_pos_valid = arm_pos_valid

        # When arm isn't in IK mode, we additionally keep updating the reference frame
        # (which also reset increments to 0) such that whenever the ik solver is invoked,
        # it will solve for the arm's current pose, which should return the arm's current
        # joint angles. This allows arm safety validator to verify that the IK node
        # is currently outputting a safe command, and allow the arm to switch into IK mode.
        if self.mode not in [
            ControlMode.MODE_IK_CHASSIS_FRAME,
            ControlMode.MODE_IK_GRIPPER_FRAME,
        ]:
            self._initialize_reference_frame()

    def ikinput_state_cb(self, msg: IKInput) -> None:
        """
        Accumulate increments from IKInput and call IK Solver
        based on arm's initial pose (when it entered IK mode) and the accumulated increments.
        If successful (Arm's initial pose is valid, IK solved successfully and
        they are within limits), publish the joint angles as position command.
        """

        if self.reference_T is None:
            return

        # Get the total increments from accumulated increments + received IKInput values
        # Don't store them immediately to their respective attributes in case IK solver
        # fails (e.g., joint out of reach)
        dx = self.dx + msg.x * SENSITIVITY_POS
        dy = self.dy + msg.y * SENSITIVITY_POS
        dz = self.dz + msg.z * SENSITIVITY_POS
        droll = self.droll + msg.roll * SENSITIVITY_ORIENT
        dpitch = self.dpitch + msg.pitch * SENSITIVITY_ORIENT
        dyaw = self.dyaw + msg.yaw * SENSITIVITY_ORIENT

        # Compute the equivalent transformation matrix dT for these increments
        dR = Rotation.from_euler(seq="xyz", angles=[droll, dpitch, dyaw]).as_matrix()
        dt = np.array([dx, dy, dz]).reshape(-1, 1)
        dT = np.hstack((dR, dt))
        dT = np.vstack((dT, np.array([0, 0, 0, 1])))

        # Compute the goal pose
        if self.frame_for_increments == EE_FRAME:
            Tgoal = self.reference_T @ dT
        else:
            # Interpret increments in BASE_FRAME.
            # To make it intuitive, it rotates about base frame axes BUT at the origin
            # of ee_frame.
            dT_translation = np.eye(4, 4)
            dT_translation[0:3, 3] = dT[0:3, 3]
            Tgoal = dT_translation @ self.reference_T
            dR = dT[0:3, 0:3]
            Tgoal[0:3, 0:3] = dR @ Tgoal[0:3, 0:3]
        Tgoal = self._truncate_values_near_zero(Tgoal)

        # Use inverse kinematics to compute the required joint angles
        solved_joint_angles = self.solver.ik_solve(
            Tgoal[0:3, 3],
            Tgoal[0:3, 0:3],
            self.current_joint_values,
        )

        if solved_joint_angles is None:
            self.get_logger().error("IK Solver Failed")
            return

        # Convert solution to physical range in deg before publishing
        solved_joint_angles = map_joint_angles_from_software_to_physical(
            solved_joint_angles, self.current_joint_values
        )
        solved_joint_angles = np.rad2deg(solved_joint_angles)

        # Update the increments and publish joint values as command
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.droll = droll
        self.dpitch = dpitch
        self.dyaw = dyaw
        self.publish_ik_command(solved_joint_angles)

    def publish_ik_command(self, goal_joint_values: List) -> None:
        """Create a ArmCommand msg and publish it"""
        msg = ArmCommand()
        for i in range(6):
            joint = getattr(msg, self.joints_names[i])
            joint.command_type = JointCommand.COMMAND_TYPE_POSITION
            joint.value = goal_joint_values[i]
        self.command_pub.publish(msg)

    def _truncate_values_near_zero(self, matrix: np.ndarray, threshold: float = 1e-8) -> np.ndarray:
        return np.where(np.abs(matrix) < threshold, 0.0, matrix)


def main(args=None):
    rclpy.init(args=args)
    local_ik_node = LocalIKNode()
    rclpy.spin(local_ik_node)
    rclpy.shutdown()
