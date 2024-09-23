import rclpy
from rclpy.node import Node
import numpy as np
from arm_msgs.msg import MouseState
from arm_ik.solvers import ik_solve
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

# This is a manual test for the ik solver (not the local ik node).
# The user puts a desired goal pose and goal orientation through a MouseState msg in rqt.
# This code will subscribe to it, solve ik, and publish the joint values as JointStates.
# Verify by visualization in RVIZ with display.launch.py,


class IKTestNode(Node):
    def __init__(self):
        super().__init__("ik_solve_test")
        self.sub = self.create_subscription(
            MouseState, "/test/ik_cmd", self.ik_cb, qos_profile=10
        )
        self.pub = self.create_publisher(JointState, "/joint_states", qos_profile=10)

        # This last joint state will simulate current arm position.
        self.last_joint_states = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def ik_cb(self, msg):
        goal_position = np.array([msg.x, msg.y, msg.z])
        goal_orientation = Rotation.from_euler(
            seq="xyz", angles=[msg.roll, msg.pitch, msg.yaw]
        ).as_matrix()

        ik_success_flag, [q1, q2, q3, q4, q5, q6] = ik_solve(
            goal_position, goal_orientation, self.last_joint_states
        )
        self.get_logger().info(
            "=====\n"
            f"Goal Position = {goal_position}\n"
            f"Goal Orientation (Rotation matrix) = {goal_orientation}\n"
            f"Success? {ik_success_flag} \n"
            f"Solved Joint Angles = {q1, q2, q3, q4, q5, q6}\n"
            "=====\n"
        )

        if ik_success_flag:
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
            joint_state_msg.position = [q1, q2, q3, q4, q5, q6]
            self.pub.publish(joint_state_msg)
            self.last_joint_states = np.array([q1, q2, q3, q4, q5, q6])


def main(args=None):
    rclpy.init(args=args)
    node = IKTestNode()
    rclpy.spin(node)
    rclpy.shutdown()
