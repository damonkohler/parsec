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

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import roslib; roslib.load_manifest('publish_servo_joint_state')

import rospy
from sensor_msgs.msg import JointState
from parsec_msgs.msg import LaserTiltSignal, LaserTiltProfile


class PublishServoJointState(object):
  """Class for extrapolating the current servo position

  This class subscribes to the servo profile and the signal topic
  that is published by the servo. It extrapolates the current
  position of the servo based on the configured period, the minimal
  and maximal angle and the last signal received. Finally, it
  publishes a JointState message.
  """

  def __init__(self):
    self._publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 20.0))
    self._joint_name = rospy.get_param('~joint_name', 'tilt_laser_joint')
    self._joint_states_pub = rospy.Publisher('joint_states', JointState)
    self._profile = None
    self._signal = None
    self._velocity = 0
    self._signal_subscriber = rospy.Subscriber('~signal', LaserTiltSignal, self.onLaserTiltSignal)
    self._profile_subscriber = rospy.Subscriber('~profile', LaserTiltProfile, self.onLaserProfile)

  def run(self):
    while not rospy.is_shutdown():
      self._publish_rate.sleep()
      # If we didn't receive a signal or the current configuration
      # yet, do nothing.
      if not self._profile or self._velocity == 0 or not self._signal:
        continue
      now = rospy.Time.now()
      if self._signal.signal == LaserTiltSignal.DIRECTION_DOWN:
        pos = self._profile.min_angle + self._velocity * (now - self._signal.header.stamp).to_sec()
        vel = -self._velocity
      elif self._signal.signal == LaserTiltSignal.DIRECTION_UP:
        pos = self._profile.max_angle - self._velocity * (now - self._signal.header.stamp).to_sec()
        vel = self._velocity
      else:
        rospy.logerr('Unknown singal %d' % self._signal.signal)
        self._signal = None

      if pos < self._profile.min_angle or pos > self._profile.max_angle:
        rospy.logerr('Ran out of possible tilting range. That probably means we are missing a signal.')
        self._signal = None

      joint_state = JointState()
      joint_state.header.stamp = now
      joint_state.name = [self._joint_name]
      joint_state.position = [pos]
      joint_state.velocity = [-vel]
      joint_state.effort = [0.0]
      self._joint_states_pub.publish(joint_state)

  def onLaserTiltSignal(self, signal):
    self._signal = signal

  def onLaserProfile(self, profile):
    self._profile = profile
    self._velocity = 0
    if profile.period > 0:
      self._velocity = (profile.max_angle - profile.min_angle) / (profile.period / 2)


def main():
  rospy.init_node('publish_servo_joint_state')
  PublishServoJointState().run()


if __name__ == '__main__':
  main()
