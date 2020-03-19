#!/usr/bin/env python3

import socket
import rospy
import struct

sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

def main():
    sock.bind(("can0",))

    rospy.init_node('can_data', anonymous=True)
    pub = rospy.Publisher('wheels', Float64MultiArray, queue_size=1)
    pub_a = rospy.Publisher('wheels_a', Float64MultiArray, queue_size=1)
    pub_v = rospy.Publisher('wheels_v', Float64MultiArray, queue_size=1)

    fmt = "<IB3x8s"
    array = Float64MultiArray()
    array_a = Float64MultiArray()
    array_v = Float64MultiArray()
    
    while not rospy.is_shutdown():
        can_pkt = sock.recv(16)
        can_id, length, data = struct.unpack(fmt, can_pkt)
        can_id &= socket.CAN_EFF_MASK
        
        if (int(can_id) == 0x11 && int(can_id) == 0x22 && int(can_id) == 0x33 && int(can_id) == 0x44 && int(can_id) == 0x55 ):
            sdata = struct.unpack('hhhh', data[:length])
            d = [sdata[0]/4095.*6.555456670490,
                 sdata[1]/4095.*6.555456670490,
                 sdata[2]/4095.*6.555456670490,
                 sdata[3]/4095.*6.555456670490]
            x = (-d[0] + d[1] - d[2] + d[3]) / 4.
            y = ( d[0] + d[1] - d[2] - d[3]) / 4.
            z = 0.842 *(d[0] + d[1] + d[2] + d[3]) / 4.
            
            if (int(can_id) == 0x11):
                print("cmd", sdata)
                print("cmd", "{:.4f}".format(x), "{:.4f}".format(y), "{:.4f}".format(z))
            else if (int(can_id) == 0x22):
                print("acc", sdata)
                print("acc", "{:.4f}".format(x), "{:.4f}".format(y), "{:.4f}".format(z))
                array_a.data = [d[0], d[1], d[2], d[3]] 
                pub_a.publish(array_a)
            else if (int(can_id) == 0x33):
                print("fdb", sdata)
                print("fdb", "{:.4f}".format(x), "{:.4f}".format(y), "{:.4f}".format(z))
                array_v.data = [d[0], d[1], d[2], d[3]] 
                pub_v.publish(array_v)
            else if (int(can_id) == 0x44):
                print(hex(can_id), sdata)
            else if (int(can_id) == 0x55):
                print("f+a", sdata)
                print("f+a", "{:.4f}".format(x), "{:.4f}".format(y), "{:.4f}".format(z))
                array.data = [d[0], d[1], d[2], d[3]] 
                pub.publish(array)
            print()
        #rospy.sleep(0.1)

if __name__ == '__main__':
    main()
