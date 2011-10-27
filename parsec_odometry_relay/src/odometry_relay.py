#!/usr/bin/env python
#
# Copyright (C) 2011 Google Inc.
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#

import roslib
roslib.load_manifest('parsec_odometry_relay')

import rospy

from parsec_msgs.msg import Odometry as ParsecOdometry
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped

odom_publisher = rospy.Publisher('odom', Odometry)
tf_publisher = rospy.Publisher('/tf', tfMessage)

odom = Odometry()
odom.child_frame_id = 'base_link'

publish_tf = True

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

  if publish_tf:
    transform = TransformStamped()
    transform.header = data.header
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation = odom.pose.pose.position
    transform.transform.rotation = odom.pose.pose.orientation
    tf_publisher.publish(tfMessage(transforms=[transform]))


def listener():
  global publish_tf
  
  rospy.init_node('parsec_odometry_relay')
  odom.child_frame_id = rospy.get_param('~base_frame_id', 'base_link')
  publish_tf = rospy.get_param('~publish_tf', True)
  rospy.loginfo('Using base frame %s' % odom.child_frame_id)
  rospy.Subscriber('odom_simple', ParsecOdometry, relay)
  rospy.spin()


if __name__ == '__main__':
  listener()
