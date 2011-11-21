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

"""Publish extrapolated joint states of the tiliting laser."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import threading

import roslib; roslib.load_manifest('publish_servo_joint_state')
import rospy

from sensor_msgs.msg import JointState
from parsec_msgs.msg import LaserTiltSignal, LaserTiltProfile


class InvalidSignalError(Exception):
  pass


class ExtrapolatedPositionInvalid(Exception):
  pass


class _JointState(object):
  def __init__(self, position, velocity):
    self.position = position
    self.velocity = velocity

  def __eq__(self, other):
    return (self.position == other.position and
            self.velocity == other.velocity)

  def __repr__(self):
    return '_JointState<position: %r, velocity: %r>' % (self.position, self.velocity)


class PublishServoJointState(object):
  """Class for extrapolating the current servo position

  This class subscribes to the servo profile and the signal topic
  that is published by the servo. It extrapolates the current
  position of the servo based on the configured period, the minimal
  and maximal angle and the last signal received. Finally, it
  publishes a JointState message.
  """

  def __init__(self):
    self._lock = threading.Lock()
    # The publish rate at which to publish joint states (default 20Hz)
    self._profile = None
    self._signal = None
    self._increasing_velocity = None
    self._decreasing_velocity = None
    self._joint_name = None
    self._phase_offset = None
    self._joint_states_publisher = None
    self._signal_subscriber = None
    self._profile_subscriber = None
    self._publish_rate = None

  def run(self):
    self._publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 20.0))
    # The name of the joint we are generating states for (default
    # laser_tilt_joint)
    self._joint_name = rospy.get_param('~joint_name', 'tilt_laser_joint')
    self._phase_offset = rospy.get_param('~phase_offset', 0.00)
    self._joint_states_publisher = rospy.Publisher('joint_states', JointState)
    self._signal_subscriber = rospy.Subscriber('~signal', LaserTiltSignal, self._on_laser_tilt_signal)
    self._profile_subscriber = rospy.Subscriber('~profile', LaserTiltProfile, self._on_laser_profile)
    
    while not rospy.is_shutdown():
      self._publish_rate.sleep()
      # If we didn't receive a signal or the current configuration
      # yet, do nothing.
      with self._lock:
        if (self._profile is None or
            self._signal is None or
            self._increasing_velocity is None or
            self._decreasing_velocity is None):
          continue

        now = rospy.Time.now()
        delta_t = (now - self._signal.header.stamp).to_sec()
        if delta_t >= self._profile.increasing_duration + self._profile.decreasing_duration:
          rospy.logwarn('No signal received for a complete period.')
          self._signal = None
          continue
        extrapolated_joint_state = None
        if self._signal.signal == LaserTiltSignal.ANGLE_INCREASING:
          extrapolated_joint_state = self._extrapolate_increasing_angle(delta_t)
        else:
          extrapolated_joint_state = self._extrapolate_decreasing_angle(delta_t)

      if (extrapolated_joint_state.position > self._profile.max_angle or
          extrapolated_joint_state.position < self._profile.min_angle):
        raise ExtrapolatedPositionInvalid('Position %r outside angle limits %r and %r.' %
                                          (extrapolated_joint_state.position,
                                           self._profile.min_angle,
                                           self._profile.min_angle))
      joint_state = JointState()
      joint_state.header.stamp = now + rospy.Duration(self._phase_offset)
      joint_state.name = [self._joint_name]
      joint_state.position = [extrapolated_joint_state.position]
      joint_state.velocity = [extrapolated_joint_state.velocity]
      joint_state.effort = [0.0]
      self._joint_states_publisher.publish(joint_state)

  def _on_laser_tilt_signal(self, signal):
    with self._lock:
      self._signal = signal

  def _on_laser_profile(self, profile):
    with self._lock:
      # only reset the signal if the profile changed
      if (self._profile is None or
          self._profile.min_angle != profile.min_angle or
          self._profile.max_angle != profile.max_angle or
          self._profile.increasing_duration != profile.increasing_duration or
          self._profile.decreasing_duration != profile.decreasing_duration):
        self._signal = None

      self._profile = profile
      self._increasing_velocity = self._calculate_velocity(self._profile.min_angle, self._profile.max_angle,
                                                           self._profile.increasing_duration)
      self._decreasing_velocity = self._calculate_velocity(self._profile.max_angle, self._profile.min_angle,
                                                           self._profile.decreasing_duration)
      rospy.loginfo('min angle: %r, max angle: %r' % (profile.min_angle, profile.max_angle))
      rospy.loginfo('increasing velocity: %r, decreasing velocity: %r' % (self._increasing_velocity, self._decreasing_velocity))

  def _extrapolate_increasing_angle(self, delta_t):
    if delta_t > self._profile.increasing_duration:
      rospy.loginfo('Missing a singal ANGLE_DECREASING.')
      delta_t = delta_t - self._profile.increasing_duration
      return _JointState(self._profile.max_angle + self._decreasing_velocity * delta_t,
                         self._decreasing_velocity)
    if delta_t < 0:
      return self._extrapolate_decreasing_angle(-delta_t)
    return _JointState(self._profile.min_angle + self._increasing_velocity * delta_t,
                       self._increasing_velocity)

  def _extrapolate_decreasing_angle(self, delta_t): 
    if delta_t > self._profile.decreasing_duration:
      rospy.loginfo('Missing a singal ANGLE_INCREASING.')
      delta_t = delta_t - self._profile.decreasing_duration
      return _JointState(self._profile.min_angle + self._increasing_velocity * delta_t,
                         self._increasing_velocity)
    if delta_t < 0:
      return self._extrapolate_increasing_angle(-delta_t)
    return _JointState(self._profile.max_angle + self._decreasing_velocity * delta_t,
                       self._decreasing_velocity)

  def _calculate_velocity(self, min_angle, max_angle, duration):
    if duration > 0:
      return (max_angle - min_angle) / duration

def main():
  rospy.init_node('publish_servo_joint_state')
  PublishServoJointState().run()


if __name__ == '__main__':
  main()
