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

import unittest

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import laser_scans
import sensor_msgs.msg as sensor_msgs


class TestLaserScans(unittest.TestCase):

  def setUp(self):
    self._scans = laser_scans.LaserScanQueue()
    
    scan_1 = sensor_msgs.LaserScan()
    scan_1.header.stamp = rospy.Time(0)

    scan_2 = sensor_msgs.LaserScan()
    scan_2.header.stamp = rospy.Time(1)

    scan_3 = sensor_msgs.LaserScan()
    scan_3.header.stamp = rospy.Time(2)

    self._scans.add_scan(scan_1)
    self._scans.add_scan(scan_2)
    self._scans.add_scan(scan_3)

  def test_intervals(self):
    scans = self._scans.get_scans_in_interval(rospy.Time(0), rospy.Time(2))
    self.assertEqual(len(scans), 3)
    scans = self._scans.get_scans_in_interval(rospy.Time(0), rospy.Time(3))
    self.assertEqual(len(scans), 3)
    scans = self._scans.get_scans_in_interval(rospy.Time(0.5), rospy.Time(2.5))
    self.assertEqual(len(scans), 2)
    self.assertEqual(scans[0].header.stamp, rospy.Time(1))
    self.assertEqual(scans[1].header.stamp, rospy.Time(2))

  def test_laser_scan_queue_lookup_scan_time(self):
    scan = self._scans.find_scan_at_time(rospy.Time(0))
    self.assertEqual(scan.header.stamp, rospy.Time(0))
    scan = self._scans.find_scan_at_time(rospy.Time(1))
    self.assertEqual(scan.header.stamp, rospy.Time(1))
    scan = self._scans.find_scan_at_time(rospy.Time(2))
    self.assertEqual(scan.header.stamp, rospy.Time(2))

    scan = self._scans.find_scan_at_time(rospy.Time(0.9))
    self.assertEqual(scan.header.stamp, rospy.Time(1))
    scan = self._scans.find_scan_at_time(rospy.Time(0.5))
    self.assertEqual(scan.header.stamp, rospy.Time(1))

    scan = self._scans.find_scan_at_time(rospy.Time(1.4))
    self.assertEqual(scan.header.stamp, rospy.Time(1))

    scan = self._scans.find_scan_at_time(rospy.Time(100))
    self.assertEqual(scan.header.stamp, rospy.Time(2))


if __name__ == '__main__':
  unittest.main()
