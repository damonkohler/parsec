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

"""Publish a zero velocity on /estop_cmd_vel to make the robot stop immediately"""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import roslib; roslib.load_manifest('parsec_bringup')

import rospy
import geometry_msgs.msg as geometry_msgs

if __name__ == '__main__':
  rospy.init_node('estop')
  estop_publisher = rospy.Publisher('~cmd_vel', geometry_msgs.Twist)
  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    estop_publisher.publish(geometry_msgs.Twist())
    rate.sleep()
