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

import math
import unittest

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import servo_calibration_routine

import sensor_msgs.msg as sensor_msgs
import parsec_msgs.msg as parsec_msgs


class CalibrateTiltingServoTest(unittest.TestCase):

  def setUp(self):
    self._low_angle = -math.pi / 4
    self._high_angle = math.pi / 4
    self._wall_distance = 10
    self._phase_offset = 0.1
    
    self._calibration_routine = servo_calibration_routine.ServoCalibrationRoutine(
        self._low_angle, self._high_angle, 4.0)

    for scan in self._generate_laser_scans(
        self._wall_distance, self._low_angle, self._high_angle,
        start_time=0.0, end_time=2.0, count=20):
      self._calibration_routine._scans.add_scan(scan)
    for scan in self._generate_laser_scans(
        self._wall_distance, self._high_angle, self._low_angle,
        start_time=2.0, end_time=4.0, count=20):
      self._calibration_routine._scans.add_scan(scan)
    for scan in self._generate_laser_scans(
        self._wall_distance, self._low_angle, self._high_angle,
        start_time=4.0, end_time=6.0, count=20):
      self._calibration_routine._scans.add_scan(scan)
    for scan in self._generate_laser_scans(
        self._wall_distance, self._high_angle, self._low_angle,
        start_time=6.0, end_time=8.0, count=20):
      self._calibration_routine._scans.add_scan(scan)
    
    increasing_signal_1 = parsec_msgs.LaserTiltSignal()
    increasing_signal_1.header.stamp = rospy.Time(0.0 + self._phase_offset)
    increasing_signal_1.signal = parsec_msgs.LaserTiltSignal.ANGLE_INCREASING

    decreasing_signal = parsec_msgs.LaserTiltSignal()
    decreasing_signal.header.stamp = rospy.Time(2.0 + self._phase_offset)
    decreasing_signal.signal = parsec_msgs.LaserTiltSignal.ANGLE_DECREASING

    increasing_signal_2 = parsec_msgs.LaserTiltSignal()
    increasing_signal_2.header.stamp = rospy.Time(4.0 + self._phase_offset)
    increasing_signal_2.signal = parsec_msgs.LaserTiltSignal.ANGLE_INCREASING

    decreasing_signal_2 = parsec_msgs.LaserTiltSignal()
    decreasing_signal_2.header.stamp = rospy.Time(6.0 + self._phase_offset)
    decreasing_signal_2.signal = parsec_msgs.LaserTiltSignal.ANGLE_DECREASING
    
    self._calibration_routine._tilt_signals = [increasing_signal_1,
                                               decreasing_signal,
                                               increasing_signal_2,
                                               decreasing_signal_2]

  def _generate_laser_scans(
      self, wall_distance, start_angle, end_angle, start_time, end_time, count):
    angle_delta = (end_angle - start_angle) / count
    delta_t = (end_time - start_time) / count
    scans = []
    for index in xrange(count):
      time = start_time + index * delta_t
      angle = start_angle + index * angle_delta
      scan = sensor_msgs.LaserScan()
      scan.header.stamp = rospy.Time(time)
      scan.ranges = [self._calculate_scan_range(wall_distance, angle)]
      scans.append(scan)
    return scans

  def _calculate_scan_range(self, wall_distance, angle):
    return (wall_distance -
            servo_calibration_routine._LASER_DISTANCE_FROM_ROTATION_AXIS *
            math.sin(angle)) / math.cos(angle)

  def assertAlmostEqual(self, lhs, rhs, error=1e-6, msg=None):
    message = '%r - %r >= %r' % (lhs, rhs, error)
    if msg is not None:
      message = msg
    self.assertTrue(abs(lhs - rhs) < error, msg=message)
    
  def test_laser_scan_calibration(self):
    calibration_result = self._calibration_routine._calculate_calibration()
    self.assertTrue(calibration_result)
    self.assertAlmostEqual(calibration_result.low_angle, self._low_angle)
    self.assertAlmostEqual(calibration_result.low_multiplier, 1)
    self.assertAlmostEqual(calibration_result.high_angle, self._high_angle)
    self.assertAlmostEqual(calibration_result.high_multiplier, 1)
    self.assertAlmostEqual(calibration_result.phase_offset, self._phase_offset)


if __name__ == '__main__':
  unittest.main()
