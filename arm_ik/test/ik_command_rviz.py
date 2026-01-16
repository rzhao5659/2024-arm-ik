import math

import numpy as np
from arm_ik.angles_utility import map_joint_angles_from_physical_to_software

import rclpy
from rclpy.node import Node

from arm_msgs.msg import ArmCommand
from sensor_msgs.msg import JointState


class IKCommandRviz(Node):
    """
    Simulate an arm by receiving the IK commands and publish back these joints angles both as
    JointState messages for RVIZ visualization, and ArmStatus as simulated arm current
    configuration.

    ArmStatus and IK Commands are joint values in PHYSICAL range.
    JointStatus are joint values in SOFTWARE range.
    """

    def __init__(self):
        super().__init__("ik_command_rviz")
        self.declare_parameter("ik_topic")
        ik_topic = self.get_parameter("ik_topic").value
        self.get_logger().info(f"Mapping from {ik_topic} to /joint_states")
        self.sub = self.create_subscription(
            ArmCommand,
            ik_topic,
            self.ik_cmd_cb,
            qos_profile=10,
        )
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", qos_profile=10)
        self.pub_timer = self.create_timer(0.1, self.publish_all)
        self.joints_names = [
            "turret",
            "shoulder",
            "elbow",
            "elbow_roll",
            "wrist_pitch",
            "wrist_roll",
        ]
        self.goal_joints_values = np.array(
            [
                0.0,
                0.0,
                math.pi / 2,
                0.0,
                math.pi / 4,
                0.0,
            ]
        )  # Initial values. This will be in software range.

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

    def ik_cmd_cb(self, msg):
        # Retrieve each joint commanded value from msg.
        # Note that these joint angles are in physical range, so we need to map them
        # into the range that the software assumes.
        physical_goal_joint_values = np.zeros(6)
        for i in range(len(self.joints_names)):
            physical_goal_joint_values[i] = getattr(msg, self.joints_names[i]).value

        physical_goal_joint_values = np.deg2rad(physical_goal_joint_values)
        goal_joint_values = map_joint_angles_from_physical_to_software(physical_goal_joint_values)

        self.goal_joints_values = goal_joint_values


def main(args=None):
    rclpy.init(args=args)
    node = IKCommandRviz()
    rclpy.spin(node)
    rclpy.shutdown()
