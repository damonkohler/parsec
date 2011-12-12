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

import unittest

import roslib; roslib.load_manifest('publish_servo_joint_state')

import publish_servo_joint_state
import parsec_msgs.msg as parsec_msgs

class PublishServoJointStateTest(unittest.TestCase):
  
  def setUp(self):
    self._publish_servo_joint_state = publish_servo_joint_state.PublishServoJointState()
    self._publish_servo_joint_state._on_laser_profile(parsec_msgs.LaserTiltProfile(
        min_angle=-1, max_angle=1, increasing_duration=1, decreasing_duration=1))
    
  def test_extrapolate_increasing_angle(self):
    self.assertEqual(self._publish_servo_joint_state._extrapolate_increasing_angle(0),
                     publish_servo_joint_state._JointState(-1, 2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_increasing_angle(0.5),
                     publish_servo_joint_state._JointState(0, 2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_increasing_angle(1),
                     publish_servo_joint_state._JointState(1, 2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_increasing_angle(1.5),
                     publish_servo_joint_state._JointState(0, -2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_increasing_angle(-0.5),
                     publish_servo_joint_state._JointState(0, -2))

    self.assertEqual(self._publish_servo_joint_state._extrapolate_decreasing_angle(0),
                     publish_servo_joint_state._JointState(1, -2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_decreasing_angle(0.5),
                     publish_servo_joint_state._JointState(0, -2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_decreasing_angle(1),
                     publish_servo_joint_state._JointState(-1, -2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_decreasing_angle(1.5),
                     publish_servo_joint_state._JointState(0, 2))
    self.assertEqual(self._publish_servo_joint_state._extrapolate_decreasing_angle(-0.5),
                     publish_servo_joint_state._JointState(0, 2))


if __name__ == '__main__':
  unittest.main()

