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

"""A simple geometry_msgs/Twist publisher."""

__author__ = 'damonkohler@google.com (Damon Kohler)'

import roslib; roslib.load_manifest('parsec_calibration')
import rospy
from rospy import timer

import geometry_msgs.msg as geometry_msgs

_DEFAULT_TIMER_PERIOD = 0.1
_DEFAULT_TOPIC = 'cmd_vel'


class TwistController(object):

  def __init__(self, topic=_DEFAULT_TOPIC, timer_period=_DEFAULT_TIMER_PERIOD):
    self._twist_publisher = rospy.Publisher(topic, geometry_msgs.Twist)
    self._twist_timer = timer.Timer(rospy.Duration(timer_period), self._publish_twist)
    self.linear_velocity = 0
    self.angular_velocity = 0

  def _publish_twist(self, unused_event):
    twist = geometry_msgs.Twist()
    twist.linear.x = self.linear_velocity
    twist.angular.z = self.angular_velocity
    self._twist_publisher.publish(twist)

  def stop(self):
    self.linear_velocity = 0
    self.angular_velocity = 0

  def go(self, linear_velocity, angular_velocity):
    self.linear_velocity = linear_velocity
    self.angular_velocity = angular_velocity

  def shutdown(self):
    self._twist_timer.shutdown()
    self._twist_timer.join()
