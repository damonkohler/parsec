#!/usr/bin/env python

import roslib
roslib.load_manifest('parsec_odometry_relay')

import rospy

from parsec_msgs.msg import Odometry as ParsecOdometry
from nav_msgs.msg import Odometry

odom_publisher = rospy.Publisher('odom', Odometry)

odom = Odometry()
odom.child_frame_id = 'base_link'

def relay(data):
  odom.header = data.header
  odom.pose.pose.position.x = data.position_x
  odom.pose.pose.position.y = data.position_y
  odom.pose.pose.orientation.z = data.orientation_z
  odom.pose.pose.orientation.w = data.orientation_w
  odom.twist.twist.linear.x = data.linear_x
  odom.twist.twist.linear.y = data.linear_y
  odom.twist.twist.angular.z = data.angular_z
  odom_publisher.publish(odom)

def listener():
  rospy.init_node('parsec_odometry_relay')
  rospy.Subscriber('odom_simple', ParsecOdometry, relay)
  rospy.spin()


if __name__ == '__main__':
  listener()
