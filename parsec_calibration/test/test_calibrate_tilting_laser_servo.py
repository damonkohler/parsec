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
    self._scan_count_per_period = 20
    
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

  def _make_calibration_routine(self, periods, period_duration, phase_offset):
    calibration_routine = servo_calibration_routine.ServoCalibrationRoutine(
        self._low_angle, self._high_angle, period_duration)

    for period in xrange(periods):
      increasing_scans = self._generate_laser_scans(
          self._wall_distance, self._low_angle, self._high_angle,
          start_time=period * period_duration,
          end_time=period * period_duration + period_duration / 2,
          count=self._scan_count_per_period)
      decreasing_scans = self._generate_laser_scans(
          self._wall_distance, self._high_angle, self._low_angle,
          start_time=period * period_duration + period_duration / 2,
          end_time=(period + 1) * period_duration,
          count=self._scan_count_per_period)
      for scan in increasing_scans + decreasing_scans:
        calibration_routine._scans.add_scan(scan)
      
      increasing_signal = parsec_msgs.LaserTiltSignal()
      increasing_signal.header.stamp = rospy.Time(period * period_duration + phase_offset)
      increasing_signal.signal = parsec_msgs.LaserTiltSignal.ANGLE_INCREASING
      calibration_routine._tilt_signals.append(increasing_signal)
  
      decreasing_signal = parsec_msgs.LaserTiltSignal()
      decreasing_signal.header.stamp = rospy.Time(
          period * period_duration + period_duration / 2 + phase_offset)
      decreasing_signal.signal = parsec_msgs.LaserTiltSignal.ANGLE_DECREASING
      calibration_routine._tilt_signals.append(decreasing_signal)

    # we need to add scans from the last scan to at least
    # phase_offset. We just add another complete sweep.
    scans = self._generate_laser_scans(
        self._wall_distance, self._low_angle, self._high_angle,
        start_time=periods * period_duration,
        end_time=periods * period_duration + period_duration / 2,
        count=self._scan_count_per_period)
    for scan in scans:
      calibration_routine._scans.add_scan(scan)
    last_signal = parsec_msgs.LaserTiltSignal()
    last_signal.header.stamp = rospy.Time(
        periods * period_duration + phase_offset)
    last_signal.signal = parsec_msgs.LaserTiltSignal.ANGLE_INCREASING
    calibration_routine._tilt_signals.append(last_signal)

    return calibration_routine

  def assertAlmostEqual(self, lhs, rhs, error=1e-6, msg=None):
    message = '%r - %r >= %r' % (lhs, rhs, error)
    if msg is not None:
      message = msg
    self.assertTrue(abs(lhs - rhs) < error, msg=message)
    
  def _check_laser_scan_calibration(self, calibration_routine, phase_offset):
    calibration_result = calibration_routine._calculate_calibration()
    self.assertTrue(calibration_result)
    self.assertAlmostEqual(calibration_result.low_angle, self._low_angle)
    self.assertAlmostEqual(calibration_result.low_multiplier, 1)
    self.assertAlmostEqual(calibration_result.high_angle, self._high_angle)
    self.assertAlmostEqual(calibration_result.high_multiplier, 1)
    self.assertAlmostEqual(calibration_result.phase_offset, phase_offset)

  def test_1_period(self):
    calibration_routine = self._make_calibration_routine(1, 4.0, 0.0)
    self._check_laser_scan_calibration(calibration_routine, 0.0)

  def test_2_periods(self):
    calibration_routine = self._make_calibration_routine(2, 4.0, 0.0)
    self._check_laser_scan_calibration(calibration_routine, 0.0)

  def test_10_periods(self):
    calibration_routine = self._make_calibration_routine(10, 4.0, 0.0)
    self._check_laser_scan_calibration(calibration_routine, 0.0)

  def test_1_period_with_phase_offset(self):
    calibration_routine = self._make_calibration_routine(1, 4.0, 0.5)
    self._check_laser_scan_calibration(calibration_routine, 0.5)

  def test_2_periods_with_phase_offset(self):
    calibration_routine = self._make_calibration_routine(2, 4.0, 0.5)
    self._check_laser_scan_calibration(calibration_routine, 0.5)

  def test_10_periods_with_phase_offset(self):
    calibration_routine = self._make_calibration_routine(10, 4.0, 0.5)
    self._check_laser_scan_calibration(calibration_routine, 0.5)


if __name__ == '__main__':
  unittest.main()
