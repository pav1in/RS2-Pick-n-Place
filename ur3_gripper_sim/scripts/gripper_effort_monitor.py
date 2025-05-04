#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class EffortMonitor:
    def __init__(self):
        self.min_effort = float('inf')
        self.max_effort = float('-inf')

        rospy.loginfo("Gripper effort monitor node started")  # ✅ inside __init__

        self.force_pub = rospy.Publisher('/gripper_force_estimate', Float64, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.callback)

    def callback(self, msg):
        try:
            idx = msg.name.index('gripper_joint')
            effort = msg.effort[idx]

            self.min_effort = min(self.min_effort, effort)
            self.max_effort = max(self.max_effort, effort)

            rospy.loginfo_throttle(1, f"[Effort] {effort:.6f}")
            rospy.loginfo_throttle(1, f"[Effort] Current: {effort:.6f} | Min: {self.min_effort:.6f} | Max: {self.max_effort:.6f}")
            self.force_pub.publish(effort)

        except ValueError:
            pass

if __name__ == '__main__':
    rospy.init_node('gripper_effort_monitor')
    EffortMonitor()
    rospy.spin()

