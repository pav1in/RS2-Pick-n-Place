#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from visualization_msgs.msg import Marker

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_control')
        # declare parameters for force limits (kg -> N) and slip
        self.declare_parameter('cylinder_max_force', 2.0 * 9.81)
        self.declare_parameter('cube_max_force', 4.0 * 9.81)
        self.declare_parameter('force_increment', 1.0)  # N per slip
        self.declare_parameter('slip_distance_threshold', 0.015)  # m
        # initial thresholds
        self.force_threshold = self.get_parameter('cube_max_force').value
        self.cylinder_max_force = self.get_parameter('cylinder_max_force').value
        self.cube_max_force = self.get_parameter('cube_max_force').value
        self.force_increment = self.get_parameter('force_increment').value
        self.slip_distance_threshold = self.get_parameter('slip_distance_threshold').value

        self.step = 0.001  # m per update
        self.current_width = None
        self.target_width = None
        self.closing = False
        self.latest_force = 0.0
        self.object_label = 'unknown'
        self.latest_obj_pose = None
        self.latest_ee_pose = None

        # publishers
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/finger_width_controller/commands',
            10
        )
        self.grab_pub = self.create_publisher(
            Bool,
            '/gripper/grabbed',
            10
        )

        # subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(WrenchStamped, '/tcp_fts_sensor', self.force_cb, 10)
        self.create_subscription(Marker, '/obb_marker', self.bbox_cb, 10)
        self.create_subscription(String, '/detected_object_label', self.label_cb, 10)
        self.create_subscription(PoseStamped, '/detected_object_pose', self.obj_pose_cb, 10)
        self.create_subscription(PoseStamped, '/ee_pose', self.ee_pose_cb, 10)

        # timer for update loop
        self.timer = self.create_timer(0.05, self.update)

    def joint_cb(self, msg: JointState):
        if 'finger_width' in msg.name:
            idx = msg.name.index('finger_width')
            self.current_width = msg.position[idx]

    def force_cb(self, msg: WrenchStamped):
        self.latest_force = msg.wrench.force.z

    def bbox_cb(self, msg: Marker):
        obj_width = msg.scale.x
        self.target_width = obj_width + 0.01
        self.closing = False
        self.get_logger().info(
            f"Opening to {self.target_width:.3f} m (obj {obj_width:.3f} + 0.01)"
        )
        self.publish_width(self.target_width)

    def label_cb(self, msg: String):
        self.object_label = msg.data.lower()
        if self.object_label == 'cylinder':
            self.force_threshold = self.cylinder_max_force
        else:
            self.force_threshold = self.cube_max_force
        self.get_logger().info(
            f"Label '{self.object_label}' → max_force={self.force_threshold:.1f} N"
        )

    def obj_pose_cb(self, msg: PoseStamped):
        self.latest_obj_pose = msg.pose

    def ee_pose_cb(self, msg: PoseStamped):
        self.latest_ee_pose = msg.pose

    def publish_width(self, width: float):
        m = Float64MultiArray(data=[width])
        self.pub.publish(m)

    def update(self):
        # trigger closing
        if (self.target_width is not None and
            self.current_width is not None and
            not self.closing and
            abs(self.current_width - self.target_width) < 1e-4):
            self.closing = True
            self.get_logger().info("Beginning close until force threshold")

        if self.closing and self.current_width is not None:
            # detect slip
            if self.latest_obj_pose and self.latest_ee_pose:
                dx = self.latest_obj_pose.position.x - self.latest_ee_pose.position.x
                dy = self.latest_obj_pose.position.y - self.latest_ee_pose.position.y
                dz = self.latest_obj_pose.position.z - self.latest_ee_pose.position.z
                slip = math.sqrt(dx*dx + dy*dy + dz*dz)
                if slip > self.slip_distance_threshold:
                    new_thresh = min(
                        self.force_threshold + self.force_increment,
                        self.cylinder_max_force if self.object_label=='cylinder' else self.cube_max_force
                    )
                    if new_thresh > self.force_threshold:
                        self.force_threshold = new_thresh
                        self.get_logger().info(
                            f"Slip {slip:.3f} m > {self.slip_distance_threshold:.3f} m, "
                            f"increasing force_thresh to {self.force_threshold:.1f} N"
                        )
            # check force
            if self.latest_force >= self.force_threshold:
                self.get_logger().info(
                    f"Force {self.latest_force:.2f} N ≥ {self.force_threshold:.2f} N → grabbed"
                )
                self.grab_pub.publish(Bool(data=True))
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
