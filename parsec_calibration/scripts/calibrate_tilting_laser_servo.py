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

"""Find the angular limits of the tilting servo."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import math
import sys
import threading

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import laser_scans

import sensor_msgs.msg as sensor_msgs
import parsec_msgs.msg as parsec_msgs


def _print_results_mean(results, stream):
  count = len(results)
  low_angle_mean = sum(result.low_angle for result in results) / count
  low_multiplier_mean = sum(result.low_multiplier for result in results) / count
  high_angle_mean = sum(result.high_angle for result in results) / count
  high_multiplier_mean = sum(result.high_multiplier for result in results) / count
  stream.write('Low angle mean: %f, %f degrees, multiplier %f\n' % (
    low_angle_mean, low_angle_mean * 180 / math.pi, low_multiplier_mean))
  stream.write('High angle mean: %f, %f degrees, multiplier %f\n' % (
    high_angle_mean, high_angle_mean * 180 / math.pi, high_multiplier_mean))


class CalibrationError(Exception):
  pass


class CalibrationResult(object):

  def __init__(self, low_angle, low_multiplier, high_angle, high_multiplier):
    self.low_angle = low_angle
    self.high_angle = high_angle
    self.low_multiplier = low_multiplier
    self.high_multiplier = high_multiplier

  def write(self, stream):
    stream.write(
      'Calculated low angle %f, %f degrees, multiplier %f.\n' % (
          self.low_angle, self.low_angle * 180 / math.pi, self.low_multiplier) +
      'Calculated high angle %f, %f degrees, multiplier %f.\n' % (
          self.high_angle, self.high_angle * 180 / math.pi, self.high_multiplier))


class ServoCalibrationRoutine(object):
  """Calculates the minimal and maximal angles of the tilting servo"""

  def __init__(self, minimum_angle, maximum_angle, tilt_period):
    self._lock = threading.Lock()
    self._scans = laser_scans.LaserScanQueue()
    self._reset()
    self._minimum_angle = minimum_angle
    self._maximum_angle = maximum_angle
    self._tilt_period = tilt_period
    self._calibration_results = []

  def run(self):
    self._tilt_profile_publisher = rospy.Publisher(
        '~profile', parsec_msgs.LaserTiltProfile, latch=True)
    self._tilt_signal_subscriber = rospy.Subscriber(
        '~signal', parsec_msgs.LaserTiltSignal, self._on_tilt_signal)
    self._laser_subscriber = rospy.Subscriber(
        '~scan', sensor_msgs.LaserScan, self._on_laser_scan)
    self._tilt_profile_publisher.publish(parsec_msgs.LaserTiltProfile(
          min_angle=self._minimum_angle, max_angle=self._maximum_angle,
          period=self._tilt_period))
    rospy.spin()

  def _on_tilt_signal(self, signal):
    with self._lock:
      if signal.signal == parsec_msgs.LaserTiltSignal.ANGLE_DECREASING:
        # Reached top of the scan range.
        self._angle_decreasing_stamp = signal.header.stamp
      elif signal.signal == parsec_msgs.LaserTiltSignal.ANGLE_INCREASING:
        self._angle_increasing_stamp = signal.header.stamp

  def _on_laser_scan(self, scan):
    self._scans.add_scan(scan)
    self._maybe_calculate_calibration()

  def _calculate_calibration(self):
    with self._lock:
      if (self._angle_increasing_stamp is None or
          self._angle_decreasing_stamp is None):
        raise CalibrationError('Not enough signals received: increasing stamp: %r; decreasing stamp: %r' % (
            self._angle_increasing_stamp, self._angle_decreasing_stamp))
      if (self._scans.get_newest_scan().header.stamp < self._angle_decreasing_stamp or
          self._scans.get_newest_scan().header.stamp < self._angle_increasing_stamp):
        raise CalibrationError('No matching scans received that match the signal time stamps.')
      low_scan = self._scans.find_scan_at_time(self._angle_increasing_stamp)
      high_scan = self._scans.find_scan_at_time(self._angle_decreasing_stamp)
      if low_scan is None or high_scan is None:
        raise CalibrationError('Signal scans not found: low_scan: %r; high_scan: %r' % (low_scan, high_scan))
      if self._angle_increasing_stamp < self._angle_decreasing_stamp:
        scans = self._scans.get_scans_in_interval(self._angle_increasing_stamp, self._angle_decreasing_stamp)
      else:
        scans = self._scans.get_scans_in_interval(self._angle_decreasing_stamp, self._angle_increasing_stamp)
      low_scan_distance = laser_scans.calculate_laser_scan_range(low_scan)
      high_scan_distance = laser_scans.calculate_laser_scan_range(high_scan)
      closest_scan_distance = laser_scans.calculate_laser_scan_range(self._find_closest_scan(scans))
      if low_scan_distance <= closest_scan_distance or high_scan_distance <= closest_scan_distance:
        raise CalibrationError(
            'Scan range measurements insufficient for calibration: low scan: %r; middle scan: %r; high scan: %r' % (
                low_scan_distance, closest_scan_distance, high_scan_distance))
      low_scan_angle = math.acos(closest_scan_distance / low_scan_distance)
      high_scan_angle = math.acos(closest_scan_distance / high_scan_distance)
      result = CalibrationResult(low_scan_angle, abs(low_scan_angle / self._minimum_angle),
                                 high_scan_angle, abs(high_scan_angle / self._maximum_angle))
      result.write(sys.stdout)
      self._calibration_results.append(result)
      _print_results_mean(self._calibration_results, sys.stdout)
      sys.stdout.write('\n')
    self._reset()

  def _maybe_calculate_calibration(self):
    with self._lock:
      if (self._angle_increasing_stamp is None or
          self._angle_decreasing_stamp is None):
        return
      if (self._scans.get_newest_scan().header.stamp < self._angle_decreasing_stamp or
          self._scans.get_newest_scan().header.stamp < self._angle_increasing_stamp):
        return
    self._calculate_calibration()

  def _reset(self):
    with self._lock:
      self._scans.clear_scans()
      self._angle_increasing_stamp = None
      self._angle_decreasing_stamp = None

  def _find_closest_scan(self, scans):
    closest_distance = laser_scans.calculate_laser_scan_range(scans[0])
    closest_scan = scans[0]
    for scan in scans[1:]:
      distance = laser_scans.calculate_laser_scan_range(scan)
      if distance < closest_distance:
        closest_distance = distance
        closest_scan = scan
    return closest_scan


def main():
  if len(rospy.myargv()) < 4:
    sys.stdout.write('Usage: %s <minimum angle> <maximum angle> <tilt period>' % rospy.myargv()[0])
    return
  rospy.init_node('calibrate_tiltinig_servo')
  calibration_routine = ServoCalibrationRoutine(float(rospy.myargv()[1]), float(rospy.myargv()[2]),
                                                float(rospy.myargv()[3]))
  calibration_routine.run()


if __name__ == '__main__':
  main()
