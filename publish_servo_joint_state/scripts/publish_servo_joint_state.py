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

import roslib; roslib.load_manifest('publish_servo_joint_state')

import rospy
from sensor_msgs.msg import JointState
from parsec_msgs.msg import LaserTiltSignal, LaserTiltProfile


__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'


class PublishServoJointState(object):
  """Class for extrapolating the current servo position

  This class subscribes to the servo profile and the signal topic
  that is published by the servo. It extrapolates the current
  position of the servo based on the configured period, the minimal
  and maximal angle and the last signal received. Finally, it
  publishes a JointState message.
  """

  def __init__(self):
    # The publish rate at which to publish joint states (default 20Hz)
    self._publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 20.0))
    # The name of the joint we are generating states for (default
    # laser_tilt_joint)
    self._joint_name = rospy.get_param('~joint_name', 'tilt_laser_joint')
    # The maximal age a signal message can have before we consider it
    # being too old. An old message here means that our clock and the
    # clock used for generating the signal are probably out of
    # sync. Default is 50ms.
    self._max_signal_age = rospy.get_param('~max_signal_age', 0.05)
    self._joint_states_publisher = rospy.Publisher('joint_states', JointState)
    self._profile = None
    self._signal = None
    self._velocity = 0
    self._signal_subscriber = rospy.Subscriber('~signal', LaserTiltSignal, self._onLaserTiltSignal)
    self._profile_subscriber = rospy.Subscriber('~profile', LaserTiltProfile, self._onLaserProfile)

  def run(self):
    while not rospy.is_shutdown():
      self._publish_rate.sleep()
      # If we didn't receive a signal or the current configuration
      # yet, do nothing.
      if not self._profile or self._velocity == 0 or not self._signal:
        continue
      now = rospy.Time.now()
      delta_t = (now - self._signal.header.stamp).to_sec()
      if delta_t < -self._max_signal_age:
        rospy.logerr('The signal message is too old. Maybe clocks are out of sync.')
        self._signal = None
        continue
      elif self._signal.signal == LaserTiltSignal.DIRECTION_UP:
        extrapolated_position = self._profile.min_angle + self._velocity * max(delta_t, 0)
        current_velocity = -self._velocity
      elif self._signal.signal == LaserTiltSignal.DIRECTION_DOWN:
        extrapolated_position = self._profile.max_angle - self._velocity * max(delta_t, 0)
        current_velocity = self._velocity
      else:
        rospy.logerr('Unknown singal %d' % self._signal.signal)
        self._signal = None
        continue

      if extrapolated_position < self._profile.min_angle or extrapolated_position > self._profile.max_angle:
        rospy.logerr('Ran out of possible tilting range. That probably means we are missing a signal.')
        self._signal = None
        continue

      joint_state = JointState()
      joint_state.header.stamp = now
      joint_state.name = [self._joint_name]
      joint_state.position = [extrapolated_position]
      joint_state.velocity = [-current_velocity]
      joint_state.effort = [0.0]
      self._joint_states_publisher.publish(joint_state)

  def _onLaserTiltSignal(self, signal):
    self._signal = signal

  def _onLaserProfile(self, profile):
    self._profile = profile
    self._velocity = 0
    if profile.period > 0:
      self._velocity = (profile.max_angle - profile.min_angle) / (profile.period / 2)


def main():
  rospy.init_node('publish_servo_joint_state')
  PublishServoJointState().run()


if __name__ == '__main__':
  main()
