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

"""Find scan parameters such as minimum angle, maximum angle and phase offset."""

import math
import scipy.optimize


class ScanParameters(object):
  """Represents the scan parameters."""

  def __init__(self, low_angle, high_angle, phase_offset, plane_distance, error):
    self.low_angle = low_angle
    self.high_angle = high_angle
    self.phase_offset = phase_offset
    self.plane_distance = plane_distance
    self.error = error

  def write(self, stream):
    stream.write('Low angle: %r (%r degrees)\n' % (
        self.low_angle, self.low_angle / math.pi * 180))
    stream.write('High angle:: %r (%r degrees)\n'% (
        self.high_angle, self.high_angle / math.pi * 180))
    stream.write('Phase offset: %r, distance from plane: %r\n' % (
        self.phase_offset, self.plane_distance))
    stream.write('Squared error: %r\n' % self.error)


class FindScanParameters(object):
  """Calculates scan parameters such as minimum angle, maximum angle and phase offset.

  This class takes a sequence numbers representing corresponding range
  measurements along a complete tilt period, i.e. all distances
  recorded while the laser rotates up and down exactly once. The
  algorithm assumes that the range measurements are taken by scanning
  a vertical plane, i.e. the robot was facing a wall.
  """

  def __init__(self, sensor_distance_from_rotation_axis,
               initial_low_angle=-1.0, initial_high_angle=1.0,
               initial_phase_offset=0.0):
    self._sensor_distance_from_rotation_axis = sensor_distance_from_rotation_axis
    self._initial_low_angle = initial_low_angle
    self._initial_high_angle = initial_high_angle
    self._initial_phase_offset = initial_phase_offset

  def find_scan_parameters(self, scans, periods=1.0):
    mean_distance_from_plane = sum(scans) / len(scans)
    initial_parameters = [mean_distance_from_plane,
                          self._initial_low_angle,
                          self._initial_high_angle,
                          self._initial_phase_offset]
    function = self._make_laser_error_function(scans, periods)
    result = scipy.optimize.leastsq(function, initial_parameters)
    distance_from_plane, low_angle, high_angle, phase_offset = result[0]
    return ScanParameters(low_angle, high_angle, phase_offset, distance_from_plane,
                          sum(v*v for v in function(list(result[0]))))

  def _make_laser_error_function(self, scans, periods=1.0):
    """Returns a function for optimization.

    Returns a function used by the optimization algorithm. The function
    gets one parameter which is a sequence of all parameters to be
    optimized and returns the squared error these parameters cause when
    compared to scans.
    """
    def function(x):
      distance_from_plane, low_angle, high_angle, phase_offset = x
      sawtooth_values = [math.modf((float(i) / (len(scans) - 1.0)) * periods + phase_offset + 1)[0]
                         for i in xrange(len(scans))]
      sweep_0_to_1 = [2 * x if x < 0.5 else 2 - 2 * x for x in sawtooth_values]
      angles = [x * high_angle + (1 - x) * low_angle for x in sweep_0_to_1]
      return [scans[i] -
              (distance_from_plane -
               self._sensor_distance_from_rotation_axis * math.sin(angles[i])) /
              math.cos(angles[i]) for i in xrange(len(scans))]
    return function
