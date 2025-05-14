#!/usr/bin/env python3

import rospy
import socket
from std_srvs.srv import Trigger, TriggerResponse

class RG2SocketDriver:
    def __init__(self):
        rospy.init_node('rg2_socket_driver', anonymous=True)

        self.robot_ip = rospy.get_param("~robot_ip", "192.168.0.194") 
        self.port = 30002  #50002 not working
        self.sock = None

        rospy.Service('rg2_grip', Trigger, self.handle_grip)
        rospy.Service('rg2_release', Trigger, self.handle_release)

        rospy.loginfo("RG2 Gripper Socket Driver is ready.")

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        self.sock.connect((self.robot_ip, self.port))

    def disconnect(self):
        if self.sock:
            self.sock.close()

    def send_script(self, script):
        try:
            self.connect()
            rospy.loginfo(f"📤 Sending: {script}")
            self.sock.sendall((script + '\n').encode('utf-8'))

            rospy.sleep(0.5)  # Optional: allow time for execution

            self.sock.shutdown(socket.SHUT_RDWR)
            self.disconnect()
            return True, "Command sent"
        except socket.timeout:
            rospy.logwarn("imeout: Robot did not accept the connection in time. Dropping packet.")
            return False, "Timeout – robot did not respond"
        except Exception as e:
            rospy.logerr(f"Socket send failed: {e}")
            return False, str(e)

    def handle_grip(self, req):
        script = 'rg_grip(30, 40, 0, True, False)'
        return TriggerResponse(*self.send_script(script))

    def handle_release(self, req):
        script = 'rg_grip(110, 40, 0, True, False)'
        return TriggerResponse(*self.send_script(script))

if __name__ == '__main__':
    try:
        driver = RG2SocketDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
