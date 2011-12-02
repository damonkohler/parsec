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

import rospy

from parsec_calibration import laser_scans
from parsec_calibration import find_scan_parameters

import sensor_msgs.msg as sensor_msgs
import parsec_msgs.msg as parsec_msgs


_LASER_DISTANCE_FROM_ROTATION_AXIS = 0.033


def _mean(values):
  values = tuple(values)
  return sum(values) / len(values)


def _radians_to_degrees(angle):
  return angle * 180 / math.pi


class CalibrationError(Exception):
  pass


class CalibrationResult(object):

  def __init__(self, low_angle, low_multiplier, high_angle, high_multiplier, phase_offset):
    self.low_angle = low_angle
    self.high_angle = high_angle
    self.low_multiplier = low_multiplier
    self.high_multiplier = high_multiplier
    self.phase_offset = phase_offset

  def write(self, stream):
    stream.write(
      'Calculated low angle %r, %r degrees, multiplier %r\n' % (
          self.low_angle, _radians_to_degrees(self.low_angle), self.low_multiplier) +
      'Calculated high angle %r, %r degrees, multiplier %r\n' % (
          self.high_angle, _radians_to_degrees(self.high_angle), self.high_multiplier) +
      'Phase offset %r\n' % self.phase_offset)


class ServoCalibrationRoutine(object):
  """Calculates the minimal and maximal angles of the tilting servo"""

  def __init__(self, minimum_angle, maximum_angle, tilt_period):
    self._lock = threading.Lock()
    self._minimum_angle = minimum_angle
    self._maximum_angle = maximum_angle
    self._tilt_period = tilt_period
    self._tilt_signals = []
    self._scans = laser_scans.LaserScanQueue()
    self._last_calibration_time = None
    self._stream = None

  def run(self, stream=sys.stdout):
    self._stream = stream
    self._tilt_profile_publisher = rospy.Publisher(
        '~profile', parsec_msgs.LaserTiltProfile, latch=True)
    self._tilt_signal_subscriber = rospy.Subscriber(
        '~signal', parsec_msgs.LaserTiltSignal, self._on_tilt_signal)
    self._laser_subscriber = rospy.Subscriber(
        '~scan', sensor_msgs.LaserScan, self._on_laser_scan)
    self._tilt_profile_publisher.publish(parsec_msgs.LaserTiltProfile(
          min_angle=self._minimum_angle, max_angle=self._maximum_angle,
          increasing_duration=self._tilt_period/2,
          decreasing_duration=self._tilt_period/2))
    rospy.spin()

  def _on_tilt_signal(self, signal):
    with self._lock:
      # ignore the first signal if it is ANGLE_DECREASING because we
      # always want to start with an ANGLE_INCREASING signal to get
      # the phase offset parameter of the optimization right.
      if (not self._tilt_signals and
          signal.signal == parsec_msgs.LaserTiltSignal.ANGLE_DECREASING):
        return
      # Don't accept the signal if we didn't receive a laser scan that
      # is older than the signal yet.
      oldest_scan = self._scans.get_oldest_scan()
      if oldest_scan and oldest_scan.header.stamp > signal.header.stamp:
        return
      self._tilt_signals.append(signal)

  def _on_laser_scan(self, scan):
    self._scans.add_scan(scan)
    self._maybe_calculate_calibration()

  def _calculate_calibration(self):
    with self._lock:
      if len(self._tilt_signals) < 2:
        raise CalibrationError(
            'Invalid number of signals: %r. We need at exactly two signals.' %
            len(self._tilt_signals))
      if self._tilt_signals[0].signal != parsec_msgs.LaserTiltSignal.ANGLE_INCREASING:
        raise CalibrationError(
            'Invalid signals. The first signal needs to be ANGLE_INCREASING.')
      if self._scans.get_newest_scan().header.stamp < self._tilt_signals[-1].header.stamp:
        raise CalibrationError(
            'Newest scan is older than newest signal.')
      if self._scans.get_oldest_scan().header.stamp > self._tilt_signals[0].header.stamp:
        raise CalibrationError(
            'Oldest scan is newer than oldest signal.')
      start_time = self._scans.find_oldest_scan_after_time(
          self._tilt_signals[0].header.stamp).header.stamp
      end_time = self._scans.find_oldest_scan_after_time(
          self._tilt_signals[-1].header.stamp).header.stamp
      scans = self._scans.get_scans_in_interval(start_time, end_time)
      scan_parameter_finder = find_scan_parameters.FindScanParameters(
          sensor_distance_from_rotation_axis=_LASER_DISTANCE_FROM_ROTATION_AXIS)
      parameters = scan_parameter_finder.find_scan_parameters(
          [(scan.ranges[int(len(scan.ranges) / 2)],
            (scan.header.stamp - start_time).to_sec())
           for scan in scans],
          self._tilt_period)
      current_result = CalibrationResult(
          parameters.low_angle, parameters.low_angle / self._minimum_angle,
          parameters.high_angle, parameters.high_angle / self._maximum_angle,
          phase_offset = parameters.phase_offset * self._tilt_period)
      if self._stream:
        current_result.write(self._stream)
        self._stream.write('\n')
      self._last_calibration_time = end_time
      return current_result

  def _maybe_calculate_calibration(self):
    with self._lock:
      if len(self._tilt_signals) < 2:
        rospy.logdebug('Cannot calibrate. Not enough signals received. Need at least two.')
        return
      if self._scans.get_newest_scan().header.stamp < self._tilt_signals[-1].header.stamp:
        rospy.logdebug('Cannot calibrate. Newest scan older than newest signal.')
        return
      if self._scans.get_oldest_scan().header.stamp > self._tilt_signals[0].header.stamp:
        rospy.logdebug('Cannot calibrate. Oldest scan newer thanoldest signal.')
        return
      if (self._last_calibration_time is not None and
          self._last_calibration_time >= self._tilt_signals[-1].header.stamp):
        return
    self._calculate_calibration()
