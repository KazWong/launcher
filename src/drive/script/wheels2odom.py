#!/usr/bin/env python

import rospy
import struct
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

odom_msg = Odometry()
odom_tf = TransformStamped()
vx, vy, vz = 0.0, 0.0, 0.0

def callback(msg):
    global vx, vy, vz
    
    vx = (-msg.data[0] + msg.data[1] - msg.data[2] + msg.data[3]) / 4.
    vy = ( msg.data[0] + msg.data[1] - msg.data[2] - msg.data[3]) / 4.
    vz = 0.842 *(msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3]) / 4.
    

def main():
    global odom_msg, odom_tf, vx, vy, vz
    rospy.init_node('can_data', anonymous=True)
    rospy.Subscriber('wheels', Float64MultiArray, callback, queue_size = 1)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now() 
    tf_pub = tf.TransformBroadcaster(queue_size=1)
    odom_x = 0.0
    odom_y = 0.0
    odom_rz = 0.0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        odom_rz += vz * dt
        odom_x += ( vx * math.cos(odom_rz) - vy * math.sin(odom_rz) ) * dt
        odom_y += ( vx * math.sin(odom_rz) + vy * math.cos(odom_rz) ) * dt
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_rz, 'rxyz');

        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = odom_x
        odom_msg.pose.pose.position.y = odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x  = vx
        odom_msg.twist.twist.linear.y  = vy
        odom_msg.twist.twist.angular.z = vz

        odom_tf.header.stamp = current_time
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_footprint'
        odom_tf.transform.translation.x = odom_x
        odom_tf.transform.translation.y = odom_y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation.x = quat[0]
        odom_tf.transform.rotation.y = quat[1]
        odom_tf.transform.rotation.z = quat[2]
        odom_tf.transform.rotation.w = quat[3]

        last_time = current_time
  
        tf_pub.sendTransformMessage(odom_tf)
        odom_pub.publish(odom_msg)
        
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
