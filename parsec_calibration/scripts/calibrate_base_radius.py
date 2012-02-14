#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#  contributors may be used to endorse or promote products derived
#  from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import with_statement

"""Calibrates the base radius value."""

__author__ = 'Wim Meeussen and Damon Kohler (damonkohler@google.com)'

import math
import scipy.optimize
import threading

import roslib; roslib.load_manifest('parsec_calibration')
import rospy
import PyKDL

from parsec_calibration import twist_controller

import parsec_msgs.msg as parsec_msgs
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

_DEFAULT_VIEW_ANGLE = math.pi / 4


def _quaternion_to_angle(orientation_z, orientation_w):
  rotation = PyKDL.Rotation.Quaternion(0, 0, orientation_z, orientation_w)
  return rotation.GetRPY()[2]


def _normalize_angle(angle):
  normalized_angle = angle
  while normalized_angle > math.pi:
    normalized_angle -= 2.0 * math.pi
  while normalized_angle < -math.pi:
    normalized_angle += 2.0 * math.pi
  return normalized_angle


class _PointCloudToAngle(object):

  def __init__(self):
    self._view_angle = rospy.get_param('view_angle', _DEFAULT_VIEW_ANGLE)
    self._publisher = rospy.Publisher('cloud_angle', std_msgs.Float64)
    self._subscriber = rospy.Subscriber('base_cloud', sensor_msgs.PointCloud,
                                        self._point_cloud_callback)

  def _point_cloud_callback(self, data):
    points = [p for p in data.points if abs(math.atan(p.y / p.x)) < self._view_angle / 2]

    def function(parameters):
      wall_distance, slope = parameters
      return [((wall_distance + slope * p.y) - p.x) / math.sqrt(1 + slope**2) for p in points]

    result = scipy.optimize.leastsq(function, [1, 0])
    error = sum(function(result[0]))
    if error > 1e-4:
      rospy.logwarn('High wall angle error: %r' % error)

    cloud_angle_message = std_msgs.Float64()
    cloud_angle_message.data = math.atan(result[0][1])
    self._publisher.publish(cloud_angle_message)


class _CalibrationRoutine(object):

  def __init__(self):
    self._lock = threading.Lock()
    self._odometry_subscriber = rospy.Subscriber(
        'rosserial/odom_simple', parsec_msgs.Odometry, self._odom_callback)
    self._point_cloud_angle_subscriber = rospy.Subscriber(
        'cloud_angle', std_msgs.Float64, self._point_cloud_callback)
    self._odometry_time = rospy.Time()
    self._point_cloud_angle_time = rospy.Time()
    self._twist_controller = twist_controller.TwistController()
    self._max_wall_angle = rospy.get_param('max_wall_angle', 3e-2)
    self._point_cloud_angle = 0
    self._point_cloud_angle_time = rospy.Time.now()
    self._odometry_angle = 0
    self._odometry_time = rospy.Time.now()

  def run(self, speed):
    odometry_start_angle, laser_scan_start_angle = self._wait_for_angles()
    last_angle = odometry_start_angle
    turn_angle = 0
    self._twist_controller.go(0, speed)
    while turn_angle < 2 * math.pi:
      with self._lock:
        delta_angle = _normalize_angle(self._odometry_angle - last_angle)
      turn_angle += delta_angle
      last_angle = self._odometry_angle
      rospy.sleep(0.05)
    self._twist_controller.stop()
    rospy.loginfo('Rotation complete. Waiting for P controller to settle...')
    rospy.sleep(5)  # HACK(damonkohler): Allow the P controller to settle.
    odometry_end_angle, laser_scan_end_angle = self._wait_for_angles()
    laser_scan_delta = 2 * math.pi + _normalize_angle(laser_scan_end_angle - laser_scan_start_angle)
    odometry_delta = 2 * math.pi + _normalize_angle(odometry_end_angle - odometry_start_angle)
    rospy.loginfo('Base radius multiplier: %r' % (laser_scan_delta / odometry_delta))
    return laser_scan_delta / odometry_delta

  def align(self):
    # Allows for some lag in data acquisition.
    self._wait_for_angles()
    rospy.loginfo('Aligning base with wall...')
    with self._lock:
      angle = self._point_cloud_angle
    while abs(angle) > self._max_wall_angle:
      if angle > 0:
        self._twist_controller.step(0, -0.3)
      else:
        self._twist_controller.step(0, 0.3)
      rospy.sleep(0.05)
      with self._lock:
        angle = self._point_cloud_angle
    rospy.loginfo('Wall angle: %r' % angle)

  def _wait_for_angles(self):
    start_time = rospy.Time.now() + rospy.Duration(0.5)
    while not rospy.is_shutdown():
      rospy.sleep(0.3)
      with self._lock:
        if self._odometry_time < start_time:
          rospy.loginfo('Still waiting for odometry...')
        elif self._point_cloud_angle_time < start_time:
          rospy.loginfo('Still waiting for laser scanner...')
        else:
          return self._odometry_angle, self._point_cloud_angle

  def _odom_callback(self, data):
    with self._lock:
      self._odometry_angle = _quaternion_to_angle(data.orientation_z, data.orientation_w)
      self._odometry_time = data.header.stamp

  def _point_cloud_callback(self, data):
    with self._lock:
      self._point_cloud_angle = data.data
      self._point_cloud_angle_time = rospy.Time.now()


def main():
  rospy.init_node('calibrate_base_radius')
  # TODO(damonkohler): Fold this into _CalibrationRoutine.
  _ = _PointCloudToAngle()
  calibration_routine = _CalibrationRoutine()

  multipliers = []
  for speed in (0.3, 0.7, 1.0, 1.5):
    calibration_routine.align()
    rospy.sleep(5)  # HACK(damonkohler): Allow the P controller to settle.
    calibration_routine.align()  # Attempt to compensate for P controller overshoot.
    multiplier = calibration_routine.run(speed)
    multipliers.append(multiplier)

  mean = sum(multipliers) / len(multipliers)
  rospy.loginfo('Mean base radius multiplier: %r' % mean)


if __name__ == '__main__':
  main()
