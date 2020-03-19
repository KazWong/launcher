#!/usr/bin/env python3

import socket
import rospy
import struct
import math

from geometry_msgs.msg import Twist

x, y, z, s = 0, 0, 0, 0
rad = 0.0
sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

PUB_HZ = 60.0
step = 0.03

def sinwave():
    global x, rad, s
    
    x = (int)( 65535 * math.sin(rad)  )
    
    s = 0
    if x < 0: s = s+4
    
    rad = rad + step

def send():
    global x, y, z, s
    fmt = "<IB3x4H"
    sock.sendall( struct.pack(fmt, 0x88, 7, abs(x), abs(y), abs(z), s) )


def main():
    sock.bind(("can0",))
    
    rospy.init_node('sinwave', anonymous=True)

    while not rospy.is_shutdown():
        sinwave()
        send()
        rospy.sleep(1./PUB_HZ)
    
    sock.sendall( struct.pack("<IB3x4H", 0x99, 8, 0xEF, 0x00, 0x00, 0x00) )
    sock.close()

if __name__ == '__main__':
    main()
