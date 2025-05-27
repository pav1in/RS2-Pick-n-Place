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
        # declare parameters
        self.declare_parameter('initial_force',             10.0)       # N
        self.declare_parameter('cylinder_max_force', 2.0 * 9.81)       # N
        self.declare_parameter('cube_max_force',     4.0 * 9.81)       # N
        self.declare_parameter('force_increment',          5.0)       # N per slip
        self.declare_parameter('slip_distance_threshold', 0.015)       # m
        self.declare_parameter('reach_distance_threshold',0.020)       # m

        # load parameters
        self.initial_force           = self.get_parameter('initial_force').value
        self.cylinder_max_force      = self.get_parameter('cylinder_max_force').value
        self.cube_max_force          = self.get_parameter('cube_max_force').value
        self.force_increment         = self.get_parameter('force_increment').value
        self.slip_distance_threshold = self.get_parameter('slip_distance_threshold').value
        self.reach_distance_threshold= self.get_parameter('reach_distance_threshold').value

        # runtime state
        self.force_threshold = self.initial_force
        self.max_force       = self.cube_max_force
        self.step            = 0.001  # m per update
        self.current_width   = None
        self.target_width    = None
        self.closing         = False
        self.latest_force    = 0.0
        self.object_label    = 'unknown'
        self.latest_obj_pose = None
        self.latest_ee_pose  = None

        # publisher for gripper commands
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/finger_width_controller/commands',
            10
        )
        # publisher for "grabbed" flag
        self.grab_pub = self.create_publisher(
            Bool,
            '/gripper/grabbed',
            10
        )

        # subscriptions
        self.create_subscription(JointState,   '/joint_states',           self.joint_cb,    10)
        self.create_subscription(WrenchStamped,'/tcp_fts_sensor',         self.force_cb,    10)
        self.create_subscription(Marker,       '/obb_marker',            self.bbox_cb,     10)
        self.create_subscription(String,       '/detected_object_label', self.label_cb,    10)
        self.create_subscription(PoseStamped,  '/detected_object_pose',  self.obj_pose_cb, 10)
        self.create_subscription(PoseStamped,  '/ee_pose',               self.ee_pose_cb,  10)

        # timer for the update loop
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
            f"Received OBB: width={obj_width:.3f} m → opening to {self.target_width:.3f} m"
        )
        self.publish_width(self.target_width)

    def label_cb(self, msg: String):
        self.object_label = msg.data.lower()
        if self.object_label == 'cylinder':
            self.max_force = self.cylinder_max_force
        else:
            self.max_force = self.cube_max_force
        # reset threshold to initial
        self.force_threshold = self.initial_force
        self.get_logger().info(
            f"Detected '{self.object_label}' → max_force={self.max_force:.1f} N, "
            f"starting at {self.force_threshold:.1f} N"
        )

    def obj_pose_cb(self, msg: PoseStamped):
        self.latest_obj_pose = msg.pose

    def ee_pose_cb(self, msg: PoseStamped):
        self.latest_ee_pose = msg.pose

    def publish_width(self, width: float):
        m = Float64MultiArray(data=[width])
        self.pub.publish(m)

    def update(self):
        # begin closing only when at target width AND within reach
        if (self.target_width is not None and
            self.current_width is not None and
            not self.closing and
            abs(self.current_width - self.target_width) < 1e-4):

            if self.latest_obj_pose and self.latest_ee_pose:
                dx = self.latest_obj_pose.position.x - self.latest_ee_pose.position.x
                dy = self.latest_obj_pose.position.y - self.latest_ee_pose.position.y
                dz = self.latest_obj_pose.position.z - self.latest_ee_pose.position.z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist <= self.reach_distance_threshold:
                    self.closing = True
                    self.get_logger().info(
                        f"Within reach ({dist:.3f} ≤ {self.reach_distance_threshold:.3f}) → closing"
                    )
                else:
                    self.get_logger().info(
                        f"Too far to close ({dist:.3f} > {self.reach_distance_threshold:.3f})"
                    )
            return

        # closing loop: slip‐based force increment & stop on max_force
        if self.closing and self.current_width is not None:
            # slip detection
            if self.latest_obj_pose and self.latest_ee_pose:
                dx = self.latest_obj_pose.position.x - self.latest_ee_pose.position.x
                dy = self.latest_obj_pose.position.y - self.latest_ee_pose.position.y
                dz = self.latest_obj_pose.position.z - self.latest_ee_pose.position.z
                slip = math.sqrt(dx*dx + dy*dy + dz*dz)
                if slip > self.slip_distance_threshold:
                    new_thresh = min(
                        self.force_threshold + self.force_increment,
                        self.max_force
                    )
                    if new_thresh > self.force_threshold:
                        self.force_threshold = new_thresh
                        self.get_logger().info(
                            f"Slip {slip:.3f} m > {self.slip_distance_threshold:.3f} m, "
                            f"raising threshold to {self.force_threshold:.1f} N"
                        )

            # apply force threshold
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

