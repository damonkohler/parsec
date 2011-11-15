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

import calibrate_tilting_servo

import sensor_msgs.msg as sensor_msgs


class CalibrateTiltingServoTest(unittest.TestCase):

  def setUp(self):
    self._high_scan_range = math.sqrt(125)
    self._low_scan_range = math.sqrt(104)
    self._middle_scan_range = 10
    
    self._calibration_routine = calibrate_tilting_servo.ServoCalibrationRoutine(-1, 1, 5)

    scan_1 = sensor_msgs.LaserScan()
    scan_1.header.stamp = rospy.Time(0)
    scan_1.ranges = [self._high_scan_range]

    scan_2 = sensor_msgs.LaserScan()
    scan_2.header.stamp = rospy.Time(1)
    scan_2.ranges = [self._middle_scan_range]

    scan_3 = sensor_msgs.LaserScan()
    scan_3.header.stamp = rospy.Time(2)
    scan_3.ranges = [self._low_scan_range]
    
    self._calibration_routine._scans.add_scan(scan_1)
    self._calibration_routine._scans.add_scan(scan_2)
    self._calibration_routine._scans.add_scan(scan_3)

    self._calibration_routine._angle_decreasing_stamp = rospy.Time(0)
    self._calibration_routine._angle_increasing_stamp = rospy.Time(2)

  def assertAlmostEqual(self, lhs, rhs, error=1e-6, msg=None):
    message = '%r - %r >= %r' % (lhs, rhs, error)
    if msg is not None:
      message = msg
    self.assertTrue(abs(lhs - rhs) < error, msg=message)
    
  def test_laser_scan_calibration(self):
    self._calibration_routine._calculate_calibration()
    self.assertTrue(self._calibration_routine._calibration_results)
    correct_high_angle = math.acos(self._middle_scan_range / self._high_scan_range)
    correct_low_angle = math.acos(self._middle_scan_range / self._low_scan_range)    
    self.assertAlmostEqual(
        self._calibration_routine._calibration_results[-1].low_angle, correct_low_angle)
    self.assertAlmostEqual(
        self._calibration_routine._calibration_results[-1].low_multiplier, correct_low_angle)
    self.assertAlmostEqual(
        self._calibration_routine._calibration_results[-1].high_angle, correct_high_angle)
    self.assertAlmostEqual(
        self._calibration_routine._calibration_results[-1].high_multiplier, correct_high_angle)

  def test_laser_scan_queue_intervals(self):
    scans = self._calibration_routine._scans.get_scans_in_interval(rospy.Time(0), rospy.Time(2))
    self.assertEqual(len(scans), 3)
    scans = self._calibration_routine._scans.get_scans_in_interval(rospy.Time(0), rospy.Time(3))
    self.assertEqual(len(scans), 3)
    scans = self._calibration_routine._scans.get_scans_in_interval(rospy.Time(0.5), rospy.Time(2.5))
    self.assertEqual(len(scans), 2)
    self.assertEqual(scans[0].header.stamp, rospy.Time(1))
    self.assertEqual(scans[1].header.stamp, rospy.Time(2))

  def test_laser_scan_queue_lookup_scan_time(self):
    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(0))
    self.assertEqual(scan.header.stamp, rospy.Time(0))
    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(1))
    self.assertEqual(scan.header.stamp, rospy.Time(1))
    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(2))
    self.assertEqual(scan.header.stamp, rospy.Time(2))

    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(0.9))
    self.assertEqual(scan.header.stamp, rospy.Time(1))
    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(0.5))
    self.assertEqual(scan.header.stamp, rospy.Time(1))

    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(1.4))
    self.assertEqual(scan.header.stamp, rospy.Time(1))

    scan = self._calibration_routine._scans.find_scan_at_time(rospy.Time(100))
    self.assertEqual(scan.header.stamp, rospy.Time(2))


if __name__ == '__main__':
  unittest.main()
