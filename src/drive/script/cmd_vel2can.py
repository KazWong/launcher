#!/usr/bin/env python3

import socket
import rospy
import struct
import math

from geometry_msgs.msg import Twist

sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

def callback(msg):
    #fmt = "<IB3x8sc"
    fmt = "<IB3x4H"
    max_vel = 6.555456670490
    vel = 1.0

    xyz_sum = math.fabs(msg.linear.x) + math.fabs(msg.linear.y) + math.fabs(msg.angular.z)
    if (xyz_sum > max_vel):
        x = (int)(65535 * msg.linear.x / xyz_sum)
        y = (int)(65535 * msg.linear.y / xyz_sum)
        z = (int)(65535 * msg.angular.z / xyz_sum)
    else:
        x = (int)(65535 * msg.linear.x / max_vel)
        y = (int)(65535 * msg.linear.y / max_vel)
        z = (int)(65535 * msg.angular.z / max_vel)

    s = 0
    if x < 0: s = s+4
    if y < 0: s = s+2
    if z < 0: s = s+1

    print(x, y, z)
    print(x*vel/65535., y*vel/65535., 0.842*z*vel/65535.)
    can_pkt = struct.pack(fmt, 0x88, 7, abs(x), abs(y), abs(z), s)
    #print(can_pkt)
    r = sock.sendall(can_pkt)
    #print("send: ", s)
    print()

    if (msg.linear.z == 1):
        can_pkt = struct.pack(fmt, 0x99, 8, 0xEF, 0x00, 0x00, 0x00)
        r = sock.sendall(can_pkt)

def main():
    sock.bind(("can0",))
    
    rospy.init_node('front_back', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, callback, queue_size = 1)
    rospy.loginfo("inited")

    rospy.spin()

    sock.close()

if __name__ == '__main__':
    main()
