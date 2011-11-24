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

"""Estimate odometry errors."""

__author__ = 'damonkohler@google.com (Damon Kohler)'

import math
import sys
import threading

import roslib; roslib.load_manifest('parsec_calibration')
import rospy
from rospy import timer

from parsec_calibration import laser_scans

import geometry_msgs.msg as geometry_msgs
import parsec_msgs.msg as parsec_msgs
import sensor_msgs.msg as sensor_msgs

_MIN_DISTANCE_TO_WALL = 2.0
_MAX_LINEAR_VELOCITY = 2.0
_TWIST_TIMER_PERIOD = rospy.Duration(0.1)
_MAX_TRAVEL_DISTANCE = 2.0


def _mean(values):
  return sum(values) / len(values)


class CalibrationResult(object):

  def __init__(self):
    self._laser_scan_messages = []
    self._odometry_messages = []
    self.odometry_distance_traveled = None
    self.laser_scan_distance_traveled = None

  def add_laser_message(self, data):
    self._laser_scan_messages.append(data)
    self._calculate_laser_scan_distance_traveled()

  def add_odometry_message(self, data):
    self._odometry_messages.append(data)
    self._calculate_odometry_distance_traveled()

  def _calculate_odometry_distance_traveled(self):
    start_x = self._odometry_messages[0].position_x
    start_y = self._odometry_messages[0].position_y
    end_x = self._odometry_messages[-1].position_x
    end_y = self._odometry_messages[-1].position_y
    self.odometry_distance_traveled = math.sqrt(
        (start_x - end_x) ** 2 + (start_y - end_y) ** 2)

  def _calculate_laser_scan_distance_traveled(self):
    start = laser_scans.calculate_laser_scan_range(self._laser_scan_messages[0])
    end = laser_scans.calculate_laser_scan_range(self._laser_scan_messages[-1])
    self.laser_scan_distance_traveled = abs(start - end)

  def calculate_odometry_error_multiplier(self):
    if self.odometry_distance_traveled > 0:
      return self.laser_scan_distance_traveled / self.odometry_distance_traveled

  def write(self, stream):
    if (self.odometry_distance_traveled is not None
        and self.laser_scan_distance_traveled is not None):
      stream.write('Odometry traveled: %.2f m\n' % self.odometry_distance_traveled)
      stream.write('Laser traveled: %.2f m\n' % self.laser_scan_distance_traveled)
      odometry_error_multiplier = self.calculate_odometry_error_multiplier()
      if odometry_error_multiplier is not None:
        stream.write('Error multiplier: %.2f\n' % odometry_error_multiplier)
    else:
      stream.write('No data collected.\n')


class CalibrationRoutine(object):

  def __init__(self):
    self._twist_publisher = rospy.Publisher('cmd_vel', geometry_msgs.Twist)
    self._odom_subscriber = rospy.Subscriber(
        'rosserial/odom_simple', parsec_msgs.Odometry, self._on_odometry)
    self._scan_subscriber = rospy.Subscriber(
        'parsec/base_scan', sensor_msgs.LaserScan, self._on_laser_scan)
    self._twist_timer = timer.Timer(_TWIST_TIMER_PERIOD, self._publish_twist)
    self._finished = threading.Event()
    self._last_laser_scan = None

  def _clear(self):
    self._linear_velocity = 0
    self._angular_velocity = 0
    self._result = CalibrationResult()
    self._finished.clear()

  def _publish_twist(self, unused_event):
    if (self._last_laser_scan is None or
        rospy.Time.now() - self._last_laser_scan > rospy.Duration(0.25)):
      rospy.logerr('Laser not coming in fast enough. '
                   'Last laser scan received at %s' % self._last_laser_scan)
      return
    twist = geometry_msgs.Twist()
    twist.linear.x = self._linear_velocity
    twist.angular.z = self._angular_velocity
    self._twist_publisher.publish(twist)

  def _on_odometry(self, data):
    if self._finished.is_set():
      return
    self._result.add_odometry_message(data)
    if self._result.odometry_distance_traveled >= _MAX_TRAVEL_DISTANCE:
      self._finish()

  def _on_laser_scan(self, data):
    self._last_laser_scan = rospy.Time.now()
    if self._finished.is_set():
      return
    self._result.add_laser_message(data)
    if (laser_scans.calculate_laser_scan_range(data) < _MIN_DISTANCE_TO_WALL and
        self._linear_velocity > 0):
      print 'Too close to wall, stopping.'
      self._finish()

  def _finish(self):
    self._linear_velocity = 0
    self._angular_velocity = 0
    self._finished.set()

  def run(self, linear_velocity, angular_velocity):
    self._clear()
    self._linear_velocity = linear_velocity
    self._angular_velocity = angular_velocity

  def wait_for_result(self):
    self._finished.wait()
    return self._result

  def shutdown(self):
    self._twist_timer.shutdown()
    self._twist_timer.join()


def main():
  rospy.init_node('calibrate_base_controller_node')

  calibration_routine = CalibrationRoutine()
  velocity = _MAX_LINEAR_VELOCITY
  results = []
  for index in range(8):
    calibration_routine.run(velocity, 0)
    results.append(calibration_routine.wait_for_result())
    velocity *= -1
    sys.stdout.write('Run %d\n' % index)
    results[-1].write(sys.stdout)
    rospy.sleep(5)  # HACK(damonkohler): Ensure that our velocity starts at 0.
  calibration_routine.shutdown()

  error_multipliers = []
  for result in results:
    error_multipliers.append(result.calculate_odometry_error_multiplier())
  sys.stdout.write('\nSummary\n')
  sys.stdout.write('Mean odometry error multiplier: %.2f\n' %
      _mean(filter(None, error_multipliers)))


if __name__ == '__main__':
  main()
