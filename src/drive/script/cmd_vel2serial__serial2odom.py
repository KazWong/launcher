#!/usr/bin/env python

import serial
import rospy
import tf
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


control = serial.Serial('/dev/ttyACM0', 9600)
encoder = serial.Serial('/dev/ttyACM1', 115200)

odom_pub = None
current_time = None
last_time = None
odom_msg = Odometry()
odom_tf = TransformStamped()


def callback(msg):
  control.flush()
  rospy.loginfo("CMD\t[%0.4f %0.4f %0.4f]", msg.linear.x, msg.linear.y, msg.angular.z)
  '''
  Read velocity command from navigation
  Send to control board
  '''
  
  cmd = str(-round(msg.linear.y, 4) * 1000.0) + ',' + str(round(msg.linear.x, 4) * 1000.0) + ',' + str(round(msg.angular.z, 4)) + '\n'
  control.write( cmd.encode("utf-8") )
	
  rospy.loginfo("CMD\t" + cmd)
  
def main():
  rospy.init_node('data_listener', anonymous=True)
  
  rospy.Subscriber('cmd_vel', Twist, callback, queue_size = 1)
  odom_pub = rospy.Publisher('odom', Odometry, queue_size = 1)
  rospy.loginfo("inited")
  
  current_time = rospy.Time.now()
  last_time = rospy.Time.now() 
  tf_pub = tf.TransformBroadcaster(queue_size=1)
  odom_x = 0.0
  odom_y = 0.0
  odom_rz = 0.0
  
  while not rospy.is_shutdown():
    '''
    Read velocity from stm32
    compute odom
    '''
    read = encoder.readline().decode("ascii")
    split = read.split()
 
    if len(split) != 3:
      continue 
  
    en = list( map(float, split) )

    rospy.loginfo("EN\t" + str(en))
    
    '''
    Cal and Pub odom tf, msg
    '''
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    odom_x += ( en[0] * math.cos(odom_rz) - en[1] * math.sin(odom_rz) ) * dt
    odom_y += ( en[0] * math.sin(odom_rz) + en[1] * math.cos(odom_rz) ) * dt
    odom_rz += en[2] * dt
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
    odom_msg.twist.twist.linear.x  = en[0]
    odom_msg.twist.twist.linear.y  = en[1]
    odom_msg.twist.twist.angular.z = en[2]

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
    
    encoder.flush()
    rospy.sleep(0.01)
    
  rospy.spin()

  control.write( '0.0,0.0,0.0\r\n'.encode('utf-8') )

if __name__ == '__main__':
  main()
