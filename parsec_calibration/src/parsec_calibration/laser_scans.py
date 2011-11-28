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

"""Utilities for handling laser scans."""

__author__ = 'damonkohler@google.com (Damon Kohler)'

import threading

_SCAN_SAMPLE_SIZE = 10


def calculate_laser_scan_range(data):
  center_index = len(data.ranges) / 2
  start_index = center_index - _SCAN_SAMPLE_SIZE / 2
  end_index = center_index + _SCAN_SAMPLE_SIZE / 2
  ranges = data.ranges[start_index:end_index]
  return sum(ranges) / len(ranges)


class IntervalInvalid(Exception):
  pass


class LaserScanQueue(object):
  """A thread safe queue of laser scans."""

  def __init__(self):
    self._lock = threading.Lock()
    self._scans = []

  def add_scan(self, scan):
    with self._lock:
      self._scans.append(scan)

  def clear_scans(self):
    with self._lock:
      self._scans = []

  def find_scan_at_time(self, time):
    scan_before = self.find_newest_scan_before_time(time)
    scan_after = self.find_oldest_scan_after_time(time)
    if scan_before is None:
      return scan_after
    elif scan_after is None:
      return scan_before
    elif time - scan_before.header.stamp < scan_after.header.stamp - time:
      return scan_before
    return scan_after

  def find_newest_scan_before_time(self, time):
    with self._lock:
      if not self._scans:
        return
      current_scan = self._scans[0]
      if current_scan.header.stamp > time:
        return
      for scan in self._scans[1:]:
        if scan.header.stamp >= time:
          break
        current_scan = scan
      return current_scan

  def find_oldest_scan_after_time(self, time):
    with self._lock:
      for scan in self._scans:
        if scan.header.stamp >= time:
          return scan

  def get_scans_in_interval(self, start_time, end_time):
    if start_time > end_time:
      raise IntervalInvalid('%f > %f' % (start_time.to_sec(), end_time.to_sec()))
    with self._lock:
      for first_scan_index, scan in enumerate(self._scans):
        if scan.header.stamp >= start_time:
          break
      else:
        raise IntervalInvalid('Start time not in interval.')
      for last_scan_index, scan in enumerate(self._scans[first_scan_index:],
                                             first_scan_index):
        if scan.header.stamp == end_time:
          return self._scans[first_scan_index:last_scan_index + 1]
        if scan.header.stamp > end_time:
          return self._scans[first_scan_index:last_scan_index]
      else:
        raise IntervalInvalid('End time not in interval.')


  def get_oldest_scan(self):
    if self._scans:
      return self._scans[0]

  def get_newest_scan(self):
    if self._scans:
      return self._scans[-1]
