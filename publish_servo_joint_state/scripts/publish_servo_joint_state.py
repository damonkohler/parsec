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
# Author moesenle@google.com (Lorenz Moesenlechner)

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
    self.publish_rate = rospy.Rate(rospy.get_param('~publish_rate', 20.0))
    self.joint_name = rospy.get_param('~joint_name', 'tilt_laser_joint')
    self.joint_states_pub = rospy.Publisher('/joint_states', JointState)
    self.profile = None
    self.signal = None
    self.velocity = 0
    self.signal_sub = rospy.Subscriber('~signal', LaserTiltSignal, self.onLaserTiltSignal)
    self.profile_sub = rospy.Subscriber('~profile', LaserTiltProfile, self.onLaserProfile)

  def run(self):
    while not rospy.is_shutdown():
      self.publish_rate.sleep()
      # If we didn't receive a signal or the current configuration
      # yet, do nothing.
      if not self.profile or self.velocity == 0 or not self.signal:
        continue
      now = rospy.Time.now()
      if self.signal.signal == LaserTiltSignal.DIRECTION_DOWN:
        pos = self.profile.min_angle + self.velocity * (now - self.signal.header.stamp).to_sec()
        vel = -self.velocity
      elif self.signal.signal == LaserTiltSignal.DIRECTION_UP:
        pos = self.profile.max_angle - self.velocity * (now - self.signal.header.stamp).to_sec()
        vel = self.velocity
      else:
        rospy.logerr('Unknown singal %d' % self.signal.signal)
        self.signal = None

      if pos < self.profile.min_angle or pos > self.profile.max_angle:
        rospy.logerr('Ran out of bounds. That probably means we are missing a signal')
        self.signal = None

      joint_state = JointState()
      joint_state.header.stamp = now
      joint_state.name = [self.joint_name]
      joint_state.position = [pos]
      joint_state.velocity = [-vel]
      joint_state.effort = [0.0]
      self.joint_states_pub.publish(joint_state)

  def onLaserTiltSignal(self, signal):
    self.signal = signal

  def onLaserProfile(self, profile):
    self.profile = profile
    if profile.period > 0:
      self.velocity = (profile.max_angle - profile.min_angle) / (profile.period/2)
    else:
      self.velocity = 0


def main():
  rospy.init_node('publish_servo_joint_state')
  PublishServoJointState().run()

if __name__ == '__main__':
  main()
