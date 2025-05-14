#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import time

class RG2GripperDriver:
    def __init__(self):
        rospy.init_node('rg2_gripper_driver', anonymous=True)
        self.script_pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)

        rospy.Service('rg2_grip', Trigger, self.handle_grip)
        rospy.Service('rg2_release', Trigger, self.handle_release)

        rospy.loginfo("✅ RG2 Gripper driver is ready.")

    def try_publish_script(self, script, action_name):
        max_retries = 3
        success = False

        for attempt in range(1, max_retries + 1):
            rospy.loginfo(f"[{action_name}] Attempt {attempt}: Sending script...")
            self.script_pub.publish(script)
            time.sleep(0.5)  # Let ROS handle it
            # Add any feedback checking logic here if needed

            # Just log and assume sent, as we don't get direct ACK
            success = True
            break

        return success

    def handle_grip(self, req):
        script = (
            'sleep(0.1)\n'
            'socket_open("127.0.0.1", 63352)\n'
            'socket_send_string("SET POS 200\\n")\n'
            'socket_close()\n'
            'sleep(0.2)\n'
        )
        success = self.try_publish_script(script, "GRIP")
        msg = "Gripper closed" if success else "Gripper command failed"
        return TriggerResponse(success=success, message=msg)

    def handle_release(self, req):
        script = (
            'sleep(0.1)\n'
            'socket_open("127.0.0.1", 63352)\n'
            'socket_send_string("SET POS 20\\n")\n'
            'socket_close()\n'
            'sleep(0.2)\n'
        )
        success = self.try_publish_script(script, "RELEASE")
        msg = "Gripper opened" if success else "Gripper release failed"
        return TriggerResponse(success=success, message=msg)

if __name__ == '__main__':
    driver = RG2GripperDriver()
    rospy.spin()
