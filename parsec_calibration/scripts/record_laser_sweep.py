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

"""Record laser range measurements of a full sweep to a file."""

import threading
import sys

import roslib; roslib.load_manifest('parsec_calibration')
import rospy

from parsec_calibration import laser_scans

import sensor_msgs.msg as sensor_msgs
import parsec_msgs.msg as parsec_msgs


class LaserSweepRecorder(object):
  """Records laser scans of a complete sweep to a stream."""

  def __init__(self, minimum_angle, maximum_angle, tilt_period):
    self._tilt_signal_condition = threading.Condition()
    self._scan_condition = threading.Condition()
    self._last_signal = None
    self._minimum_angle = minimum_angle
    self._maximum_angle = maximum_angle
    self._tilt_period = tilt_period
    self._scans = laser_scans.LaserScanQueue()

  def run(self, stream):
    tilt_profile_publisher = rospy.Publisher(
        '~profile', parsec_msgs.LaserTiltProfile, latch=True)
    tilt_signal_subscriber = rospy.Subscriber(
        '~signal', parsec_msgs.LaserTiltSignal, self._on_tilt_signal)
    laser_subscriber = rospy.Subscriber(
        '~scan', sensor_msgs.LaserScan, self._on_laser_scan)
    tilt_profile_publisher.publish(parsec_msgs.LaserTiltProfile(
          min_angle=self._minimum_angle, max_angle=self._maximum_angle,
          period=self._tilt_period))
    increasing_signal = self._wait_for_signal(parsec_msgs.LaserTiltSignal.ANGLE_INCREASING)
    decreasing_signal = self._wait_for_signal(parsec_msgs.LaserTiltSignal.ANGLE_DECREASING)
    increasing_time = self._scans.find_oldest_scan_after_time(increasing_signal.header.stamp).header.stamp
    decreasing_time = self._scans.find_newest_scan_before_time(decreasing_signal.header.stamp).header.stamp
    self._wait_for_scan_after(decreasing_signal.header.stamp)
    scans = self._scans.get_scans_in_interval(increasing_time, decreasing_time)
    self._write_scans(scans, stream)

  def _on_tilt_signal(self, data):
    with self._tilt_signal_condition:
      self._last_signal = data
      self._tilt_signal_condition.notify_all()

  def _on_laser_scan(self, data):
    self._scans.add_scan(data)
    with self._scan_condition:
      self._scan_condition.notify_all()

  def _wait_for_signal(self, signal):
    rospy.loginfo('waiting for signal %r' % signal)
    with self._tilt_signal_condition:
      while not self._last_signal or self._last_signal.signal != signal:
        self._tilt_signal_condition.wait()
      return self._last_signal

  def _wait_for_scan_after(self, time):
    rospy.loginfo('waiting for scan after %r' % time)
    with self._scan_condition:
      while True:
        newest_scan = self._scans.get_newest_scan() 
        if newest_scan and newest_scan.header.stamp < time:
          break
        self._scan_condition.wait()

  def _write_scans(self, scans, stream):
    for scan in scans:
      scan_count = len(scan.ranges)
      stream.write('%r\n' % min(scan.ranges[scan_count/2-25:scan_count/2+25]))


def main():
  if len(rospy.myargv()) < 4:
    sys.stdout.write(
        'Usage: %s <minimum angle> <maximum angle> <tilt period> [filename]\n' %
        rospy.myargv()[0])
    return
  rospy.init_node('calibrate_tiltinig_servo')
  recorder = LaserSweepRecorder(
      float(rospy.myargv()[1]), float(rospy.myargv()[2]),
      float(rospy.myargv()[3]))
  if len(rospy.myargv()) > 4:
    stream = open(rospy.myargv()[4], 'w')
  else:
    stream = sys.stdout
  recorder.run(stream)


if __name__ == '__main__':
  main()
