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

"""Computes the distance in z from the floor."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import math

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import laser_scans

import sensor_msgs.msg as sensor_msgs


_SCAN_ANGLE = 50 * math.pi / 180
_RUNNING_AVERAGE_COUNT = 100


class CalibrateFloorLaser(object):
  "Provides a calibration routine for the floor laser."

  def __init__(self):
    self._z_distances = []
    self._laser_subscriber = rospy.Subscriber(
      'scan', sensor_msgs.LaserScan, self._laser_callback)

  def _laser_callback(self, scan):
    floor_distance = laser_scans.calculate_laser_scan_range(scan)
    z_distance = math.cos(_SCAN_ANGLE) * floor_distance
    self._z_distances.append(z_distance)
    self._z_distances = self._z_distances[-_RUNNING_AVERAGE_COUNT:]
    print 'Z position of the laser: %r (mean value of %d scans)' % (
      sum(self._z_distances) / len(self._z_distances), len(self._z_distances))


def main():
  rospy.init_node('calibrate_floor_laser')
  calibrate_floor_laser = CalibrateFloorLaser()
  rospy.spin()


if __name__ == '__main__':
  main()
