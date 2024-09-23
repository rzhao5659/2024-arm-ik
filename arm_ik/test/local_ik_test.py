import rclpy
from rclpy.node import Node
from arm_msgs.msg import JointStatus, ArmStatus, ArmCommand
from sensor_msgs.msg import JointState
import math
from arm_ik.angles_utility import (
    map_joint_angles_from_physical_to_software,
    map_joint_angles_from_software_to_physical,
)

PI = math.pi
DEG2RAD = PI / 180
RAD2DEG = 180 / PI


class LocalIKTest(Node):
    """
    Simulate an arm by receiving the IK commands and publish back these joints angles both as
    JointState messages for RVIZ visualization, and ArmStatus as simulated arm current
    configuration.

    ArmStatus and IK Commands are joint values in PHYSICAL range.
    JointStatus are joint values in SOFTWARE range.
    """

    def __init__(self):
        super().__init__("local_ik_test")
        self.sub = self.create_subscription(
            ArmCommand, "/arm/command/ik", self.ik_cmd_cb, qos_profile=10
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", qos_profile=10
        )
        self.arm_status_pub = self.create_publisher(
            ArmStatus, "/arm/status/all", qos_profile=10
        )
        self.pub_timer = self.create_timer(0.1, self.publish_all)
        self.joints_names = [
            "turret",
            "shoulder",
            "elbow",
            "elbow_roll",
            "wrist_pitch",
            "wrist_roll",
        ]
        self.goal_joints_values = [
            0.0,
            0.0,
            PI / 2,
            0.0,
            PI / 4,
            0.0,
        ]  # Initial values. This will be in software range.

    def publish_all(self):
        # Publish the stored goal joint values as JointState message to visualize in RVIZ.
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "ax1-turret",
            "ax2-shoulder",
            "ax3-elbow",
            "ax4-elbow-roll",
            "ax5-wrist-pitch",
            "ax6-wrist-roll",
        ]
        joint_state_msg.position = self.goal_joints_values
        self.joint_state_pub.publish(joint_state_msg)

        # Publish the stored goal joint values as ArmStatus message to simulate arm following
        # IK command.  Unfortunately, it doesn't simulate multiturn joints.
        physical_goal_joint_values = map_joint_angles_from_software_to_physical(
            self.goal_joints_values, self.goal_joints_values
        )
        arm_status_msg = ArmStatus()
        for i in range(len(self.joints_names)):
            joint_status = JointStatus()
            joint_status.status = JointStatus.STATUS_OK
            joint_status.position = physical_goal_joint_values[i] * RAD2DEG
            setattr(arm_status_msg, self.joints_names[i], joint_status)
        self.arm_status_pub.publish(arm_status_msg)

    def ik_cmd_cb(self, msg):
        # Retrieve each joint commanded value from msg.
        # Note that these joint angles are in physical range, so we need to map them
        # into the range that the software assumes.
        physical_goal_joint_values = []
        for i in range(len(self.joints_names)):
            physical_goal_joint_values.append(
                getattr(msg, self.joints_names[i]).value * DEG2RAD
            )

        goal_joint_values = map_joint_angles_from_physical_to_software(
            tuple(physical_goal_joint_values)
        )
        self.goal_joints_values = goal_joint_values


def main(args=None):
    rclpy.init(args=args)
    node = LocalIKTest()
    rclpy.spin(node)
    rclpy.shutdown()
