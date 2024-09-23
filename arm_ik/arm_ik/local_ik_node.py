import rclpy
from rclpy.node import Node
from arm_msgs.msg import (
    JointStatus,
    JointCommand,
    ArmStatus,
    MouseState,
    ArmCommand,
    ControlMode,
)
from arm_msgs.srv import LocalIKFrameRequest
from arm_ik.solvers import ik_solve, fk_solve, truncate_values_near_zero
from arm_ik.angles_utility import (
    map_joint_angles_from_physical_to_software,
    map_joint_angles_from_software_to_physical,
    convert_angles_from_rad_to_deg,
)
import math
import numpy as np
from scipy.spatial.transform import Rotation
from typing import List

# Constants for the program
PI = math.pi
INF = math.inf
EE_FRAME = 1
BASE_FRAME = 0
DEG2RAD = PI / 180
RAD2DEG = 180 / PI

# Configuration parameters
SENSITIVITY_POS = 0.003  # Max increment in translation (x,y,z).
SENSITIVITY_ORIENT = 0.25 * PI / 180  # Max increment in rotation (r, p, y).
# Note that these joint limits are the physical limits.
# Used to verify solution of IK is not out of limits.
JOINTS_UPPER_LIMITS = (
    270,  # turret
    90,  # shoulder
    270,  # elbow
    180,  # elbow_roll
    75,  # wrist_pitch
    INF,  # wrist_roll
)  # Upper limit for each joint [deg]
JOINTS_LOWER_LIMITS = (
    -90,  # turret
    -90,  # shoulder
    -90,  # elbow
    -180,  # elbow_roll
    -75,  # wrist_pitch
    -INF,  # wrist_roll
)  # Lower limit for each joint [deg]


class LocalIKNode(Node):
    def __init__(self):
        super().__init__("local_ik_node")

        # Subscribers
        self.arm_pos_sub = self.create_subscription(
            ArmStatus, "/arm/status/all", self.arm_position_cb, 1
        )
        self.mode_sub = self.create_subscription(ControlMode, "/arm/mode/current", self.mode_cb, 1)
        self.mouse_state_sub = self.create_subscription(
            MouseState, "/arm/mouse/filtered", self.mouse_state_cb, 1
        )

        # Publishers:
        # The command will be published whenever we receive a new MouseState.
        self.command_pub = self.create_publisher(ArmCommand, "/arm/command/ik", 1)

        # Service:
        # This will change frame of reference of the Local IK increments
        # Either base frame (DEFAULT) or ee frame.
        self.change_frame_srv = self.create_service(
            LocalIKFrameRequest, "/arm/command/ik/frame", self.change_frame
        )

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
        # Initial joint values are set to all 0.0 rad. The program won't use these values as
        # arm_pos_valid is False.
        self.current_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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

    def mode_cb(self, msg):
        """
        Store the current mode and initializes the reference pose when the arm enters IK mode.
        """
        if (
            self.mode != ControlMode.MODE_INVERSE_KINEMATICS
            and msg.mode == ControlMode.MODE_INVERSE_KINEMATICS
        ):
            # Recompute the reference frame when arm enter IK mode.
            self._initialize_reference_frame()

        self.mode = msg.mode

    def _initialize_reference_frame(self) -> None:
        """
        Compute the current arm pose using FK and assign it as the reference frame for increments.
        This also reset the increments to 0.0, effectively restarting IK node.
        Fails if some joint's status is unknown.
        """
        if self.arm_pos_valid:
            # Map the arm's joint angles into the range that the solvers assume (all [-pi, pi]).
            joint_angles = map_joint_angles_from_physical_to_software(self.current_joint_values)
            self.reference_T = fk_solve(joint_angles)

            print("Reference pose initialized to current pose.")
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.droll = 0.0
            self.dpitch = 0.0
            self.dyaw = 0.0
        else:
            self.get_logger().warn(
                "Reference pose fail to initialize: some joint's status is unknown."
            )

    def change_frame(self, request, response) -> None:
        """
        Change frame of reference (either base frame or ee frame)
        for interpretation of increments. Only works if arm is in IK mode.
        This also causes a recomputation of the reference pose.
        """
        response.success = False

        if self.mode == ControlMode.MODE_INVERSE_KINEMATICS:
            # Change frame
            if request.use_ee_frame is True:
                self.frame_for_increments = EE_FRAME
                self.get_logger().info("Frame changed to EE_FRAME\n")
            elif request.use_ee_frame is False:
                self.frame_for_increments = BASE_FRAME
                self.get_logger().info("Frame changed to BASE_FRAME\n")

            # Recompute reference pose, regardless of whether the frame was actually
            # changed or not. This is more convenient,
            # so we can reset reference whenever service is requested.
            self._initialize_reference_frame()
            response.success = True

        return response

    def arm_position_cb(self, msg):
        """
        Store the measured joint values (which are in deg) in self.current_joint_values.

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
                self.current_joint_values[i] = joint.position * DEG2RAD
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
        if self.mode != ControlMode.MODE_INVERSE_KINEMATICS:
            self._initialize_reference_frame()

    def mouse_state_cb(self, msg):
        """
        Accumulate increments from MouseState and call IK Solver
        based on arm's initial pose (when it entered IK mode) and the accumulated increments.
        If successful(Arm's initial pose is valid, IK solved successfully and
        they are within limits), publish the joint angles as position command.
        """

        if self.reference_T is None:
            self.get_logger().warn(
                "IK Fail: some joint's status is unknown in the received ArmStatus"
                " when the arm entered IK mode or changed frame for increments. Doing Nothing.\n"
            )
            return

        # Get the total increments from accumulated increments + received MouseState values
        # Don't store them immediately to their respective attributes because we don't want to
        # continue accumulating increments when IK solver fails, or joint is out of reach.
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

        # Compute the desired pose after increment
        if self.frame_for_increments == EE_FRAME:
            Tgoal = self.reference_T @ dT
        else:
            # Interpret increments in BASE_FRAME.
            # To make it intuitive, it rotates about base frame axes BUT at the origin of
            # ee_frame.
            dT_translation = np.eye(4, 4)
            dT_translation[0:3, 3] = dT[0:3, 3]
            Tgoal = dT_translation @ self.reference_T
            dR = dT[0:3, 0:3]
            Tgoal[0:3, 0:3] = dR @ Tgoal[0:3, 0:3]
        Tgoal = truncate_values_near_zero(Tgoal)

        # Use inverse kinematics to compute the required joint angles
        ik_success, solved_joint_angles = ik_solve(
            Tgoal[0:3, 3],
            Tgoal[0:3, 0:3],
            np.array(self.current_joint_values),
        )

        # Check ik solver success
        if ik_success is False:
            self.get_logger().warn(
                "IK Fail: Couldn't solve for the specified goal pose"
                "(out of range or too close to base frame)."
            )
            return
        # Check joint limits: Map the solved angles to their original physical range,
        # then check joint limits.
        solved_joint_angles = map_joint_angles_from_software_to_physical(
            solved_joint_angles, self.current_joint_values
        )
        solved_joint_angles = convert_angles_from_rad_to_deg(solved_joint_angles)
        print(f"Solved joint angles in physical range: \n{solved_joint_angles}")
        joints_out_of_limits = self.check_joints_limits(solved_joint_angles)

        if joints_out_of_limits:
            warn_msg = "IK Fail: Solution contains out of limits joints values for: "
            for joint in joints_out_of_limits:
                warn_msg += f"{joint} "
            self.get_logger().warn(warn_msg)
            return

        solved_joint_angles = self.clamp_joints(solved_joint_angles)

        # The solved joint values are verified to be safe.
        # Update the increments and publish joint values as command
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.droll = droll
        self.dpitch = dpitch
        self.dyaw = dyaw
        self.publish_ik_command(solved_joint_angles)

    def clamp_joints(self, joint_values: List[float]) -> List[float]:
        joints_clamped = []
        for i in range(len(joint_values)):
            limit_barrier = 5.0  # [deg]
            joint = joint_values[i]
            if joint < JOINTS_LOWER_LIMITS[i] + limit_barrier:
                joint = JOINTS_LOWER_LIMITS[i] + limit_barrier
            if joint > JOINTS_UPPER_LIMITS[i] - limit_barrier:
                joint = JOINTS_UPPER_LIMITS[i] - limit_barrier
            joints_clamped.append(joint)
        return joints_clamped

    def check_joints_limits(self, joint_values: List) -> List:
        """
        Return the list of joints that violate limits.
        """
        joints_out_of_limits = []
        for i in range(len(joint_values)):
            limit_barrier = 5  # [deg]
            violate_lower_lim = not (JOINTS_LOWER_LIMITS[i] + limit_barrier <= joint_values[i])
            violate_upper_lim = not (joint_values[i] <= JOINTS_UPPER_LIMITS[i] - limit_barrier)
            if violate_lower_lim or violate_upper_lim:
                joints_out_of_limits.append(self.joints_names[i])
        return joints_out_of_limits

    def publish_ik_command(self, goal_joint_values: List) -> None:
        """Create a ArmCommand msg and publish it"""
        msg = ArmCommand()
        for i in range(6):
            joint = getattr(msg, self.joints_names[i])
            joint.command_type = JointCommand.COMMAND_TYPE_POSITION
            joint.value = goal_joint_values[i]
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    local_ik_node = LocalIKNode()
    rclpy.spin(local_ik_node)
    rclpy.shutdown()
