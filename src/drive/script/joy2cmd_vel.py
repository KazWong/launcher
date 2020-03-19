#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_LIN_VEL = 6.555456670490
MAX_ANG_VEL = 6.555456670490
PUB_HZ = 60.0
joy = Joy()
cmd_vel_msg = Twist()

def callback(msg):
    global joy
    joy = msg

def main():
    rospy.init_node('joy2cmd_vel', anonymous=True)

    rospy.Subscriber('joy', Joy, callback, queue_size = 1)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	
    cmd_vel_msg.linear.x = 0.0
    cmd_vel_msg.linear.y = 0.0
    cmd_vel_msg.linear.z = 0.0
    cmd_vel_msg.angular.z = 0.0
	
    while not rospy.is_shutdown():
        if (len(joy.axes) > 0):
            cmd_vel_msg.linear.x = MAX_LIN_VEL * joy.axes[1]
            cmd_vel_msg.linear.y = MAX_LIN_VEL * joy.axes[0]
            cmd_vel_msg.angular.z = MAX_ANG_VEL * joy.axes[3]

        if (len(joy.buttons) > 0):
            cmd_vel_msg.linear.z = joy.buttons[0]
		
        cmd_vel_pub.publish(cmd_vel_msg)
        rospy.sleep(1./PUB_HZ)
	
    rospy.spin()

if __name__ == '__main__':
    main()
