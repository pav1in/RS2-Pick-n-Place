#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import Marker

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_control')
        # parameters
        self.force_threshold = 5.0       # N
        self.step = 0.001                # m per update
        self.current_width = None
        self.target_width = None
        self.closing = False
        self.latest_force = 0.0

        # publisher to gripper
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/finger_width_controller/commands',
            10)

        # subscriptions
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10)
        self.create_subscription(
            WrenchStamped,
            '/tcp_fts_sensor',
            self.force_cb,
            10)
        self.create_subscription(
            Marker,
            '/obb_marker',
            self.bbox_cb,
            10)

        # timer for closing loop
        self.timer = self.create_timer(0.05, self.update)

    def joint_cb(self, msg: JointState):
        if 'finger_width' in msg.name:
            idx = msg.name.index('finger_width')
            self.current_width = msg.position[idx]

    def force_cb(self, msg: WrenchStamped):
        # assume force.z is what matters
        self.latest_force = msg.wrench.force.z

    def bbox_cb(self, msg: Marker):
        # open to object width + small clearance
        obj_width = msg.scale.x
        self.target_width = obj_width + 0.01
        self.closing = False
        self.publish_width(self.target_width)
        self.get_logger().info(
            f"Opening gripper to {self.target_width:.3f} m")

    def publish_width(self, width: float):
        m = Float64MultiArray(data=[width])
        self.pub.publish(m)

    def update(self):
        # once at target, start closing
        if (self.target_width is not None and
            self.current_width is not None and
            not self.closing and
            abs(self.current_width - self.target_width) < 1e-4):
            self.closing = True
            self.get_logger().info("Beginning close until force threshold")

        # if closing, step until force limit
        if self.closing and self.current_width is not None:
            if self.latest_force >= self.force_threshold:
                self.get_logger().info(
                    f"Force {self.latest_force:.2f} N reached—stopping.")
                self.closing = False
            else:
                new_w = max(0.0, self.current_width - self.step)
                self.publish_width(new_w)

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
