#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('unpause_gazebo_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause()
        rospy.loginfo("✅ Gazebo physics unpaused.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to unpause Gazebo: %s", e)