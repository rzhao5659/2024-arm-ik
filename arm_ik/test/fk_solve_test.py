import rclpy
from rclpy.node import Node
from arm_ik.solvers import fk_solve
from sensor_msgs.msg import JointState


class FKTestNode(Node):
    def __init__(self):
        super().__init__("fk_solve_test")
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.cb, qos_profile=10
        )

    def cb(self, msg):
        joints_values = msg.position
        T = fk_solve(joints_values)
        print(f"[FK] Solved T = {T}")


def main(args=None):
    rclpy.init(args=args)
    node = FKTestNode()
    rclpy.spin(node)
    rclpy.shutdown()
