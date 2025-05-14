#!/usr/bin/env python3

#rosservice call /rg2_release
#rosservice call /rg2_grip

import rospy
import subprocess
import socket
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class RG2GripperDriver:
    def __init__(self):
        rospy.init_node('rg2_gripper_driver', anonymous=True)

        self.robot_ip = rospy.get_param("~robot_ip", "192.168.0.194")  # UR IP address
        self.port = rospy.get_param("~port", 30002)  #50002 Port used by External Control URCap

        rospy.Service('rg2_grip', Trigger, self.handle_grip)
        rospy.Service('rg2_release', Trigger, self.handle_release)

        self.latest_roi = None
        self.roi_pc_sub = rospy.Subscriber('/segmented_roi', PointCloud2, self.roi_callback)

        rospy.loginfo("RG2 Gripper driver is ready.")

    def roi_callback(self, msg):
        self.latest_roi = msg

    def compute_roi_width(self):
        if self.latest_roi is None:
            rospy.logwarn("No ROI point cloud data received.")
            return None

        points = list(pc2.read_points(self.latest_roi, field_names=("x", "y", "z"), skip_nans=True))
        if len(points) < 2:
            rospy.logwarn("Not enough ROI points to compute width.")
            return None

        x_values = [p[0] for p in points]
        width_m = max(x_values) - min(x_values)
        return width_m

    def estimate_grip_position_from_width(self, width_mm):
        if width_mm > 110.0:
            rospy.logwarn(f"Maximum width exceeded: {width_mm:.2f} mm > 110 mm. Defaulting to fully open.")
            width_mm = 110.0
        elif width_mm < 0.0:
            width_mm = 0.0

        grip_pos = int((110.0 - width_mm) / 110.0 * 255.0)
        grip_pos = max(0, min(grip_pos, 255))
        return grip_pos

    def send_urscript_over_tcp(self, script, action_name):
        rospy.loginfo(f"[{action_name}] Connecting to robot at {self.robot_ip}:{self.port}")
        try:
            with socket.create_connection((self.robot_ip, self.port), timeout=2) as sock:
                sock.sendall((script + "\n").encode("utf-8"))
                rospy.loginfo(f"[{action_name}] Script sent successfully.")
                return True
        except Exception as e:
            rospy.logerr(f"[{action_name}] Failed to send script: {e}")
            return False

    def handle_release(self, req):
        width_m = self.compute_roi_width()
        if width_m is None:
            rospy.logwarn("Falling back to default open width.")
            width_mm = 110.0
        else:
            width_mm = width_m * 1000.0 + 5.0  # Add 5 mm clearance

        grip_pos = self.estimate_grip_position_from_width(width_mm)
        rospy.loginfo(f"Opening gripper to position: {grip_pos} for estimated width: {width_mm:.2f} mm")

        script = (
            'sleep(0.1)\n'
            'socket_open("127.0.0.1", 63352)\n'
            f'socket_send_string("SET POS {grip_pos}\\n")\n'
            'socket_close()\n'
            'sleep(0.2)\n'
        )

        success = self.send_urscript_over_tcp(script, "RELEASE")
        msg = f"Gripper released to position {grip_pos}" if success else "Release command failed"
        return TriggerResponse(success=success, message=msg)

    def handle_grip(self, req):
        rospy.loginfo("Launching force limiter script...")
        subprocess.Popen(['rosrun', 'ur3_gripper_sim', 'gripper_force_limiter.py'])

        grip_pos = 255  # 0 mm closed

        script = (
            'sleep(0.1)\n'
            'socket_open("127.0.0.1", 63352)\n'
            f'socket_send_string("SET POS {grip_pos}\\n")\n'
            'socket_close()\n'
            'sleep(0.2)\n'
        )

        success = self.send_urscript_over_tcp(script, "GRIP")
        msg = f"Gripper closed to 0 mm (position {grip_pos})" if success else "Grip command failed"
        return TriggerResponse(success=success, message=msg)

if __name__ == '__main__':
    driver = RG2GripperDriver()
    rospy.spin()
