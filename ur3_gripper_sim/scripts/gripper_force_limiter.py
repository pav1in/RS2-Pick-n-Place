#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class GripperController:
    def __init__(self):
        self.max_effort_limit = 2.2  # Adjust?
        self.command_sent = False

        rospy.loginfo("Gripper force limiter node started")

        self.force_pub = rospy.Publisher('/gripper_force_estimate', Float64, queue_size=10)
        self.cmd_pub = rospy.Publisher('/gripper_joint_position/command', Float64, queue_size=10)

        rospy.Subscriber('/joint_states', JointState, self.callback)

    def callback(self, msg):
        try:
            idx = msg.name.index('gripper_joint')
            effort = msg.effort[idx]
            position = msg.position[idx]  # ✅ Get current joint position

            self.force_pub.publish(effort)
            rospy.loginfo_throttle(1, f"[Effort] {effort:.6f}")

            # If effort exceeds limit → freeze at current position
            if abs(effort) >= self.max_effort_limit and not self.command_sent:
                rospy.logwarn(f"[FORCE LIMIT EXCEEDED] {effort:.4f} >= {self.max_effort_limit}. Holding at pos {position:.4f}")
                self.cmd_pub.publish(Float64(data=position))  # ✅ Lock at current position
                self.command_sent = True

            # Reset flag if effort drops again
            if abs(effort) < 0.1:
                self.command_sent = False

        except ValueError:
            pass

if __name__ == '__main__':
    rospy.init_node('gripper_force_limiter')
    GripperController()
    rospy.spin()
