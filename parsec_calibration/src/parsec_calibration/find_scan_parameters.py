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

__author__ = ('whess@google.com (Wolfgang Hess),'
              'moesenle@google.com (Lorenz Moesenlechner)'
              'damonkohler@google.com (Damon Kohler)')

import math
import scipy.optimize

import roslib; roslib.load_manifest('parsec_calibration')
import rospy


_MINIMUM_DISTANCE = 0  # meters
_MAXIMUM_DISTANCE = 0.3  # meters
_MAXIMUM_DISTANCE_DELTA = 0.05 # meters


class ScanParameters(object):
  """Represents the scan parameters."""

  def __init__(self, low_angle, high_angle,
      increasing_phase_offset, decreasing_phase_offset, plane_distance,
      stretch, predicted_distances, error):
    self.low_angle = low_angle
    self.high_angle = high_angle
    self.increasing_phase_offset = increasing_phase_offset
    self.decreasing_phase_offset = decreasing_phase_offset
    self.plane_distance = plane_distance
    self.stretch = stretch
    self.predicted_distances = predicted_distances
    self.error = error

  def write(self, stream):
    stream.write('Low angle: %r (%r degrees)\n' % (
        self.low_angle, self.low_angle / math.pi * 180))
    stream.write('High angle:: %r (%r degrees)\n'% (
        self.high_angle, self.high_angle / math.pi * 180))
    stream.write('Increasing phase offset: %r, distance from plane: %r\n' % (
        self.increasing_phase_offset, self.plane_distance))
    stream.write('Decreasing phase offset: %r, distance from plane: %r\n' % (
        self.decreasing_phase_offset, self.plane_distance))
    stream.write('Period stretch: %r\n' % self.stretch)
    stream.write('Squared error: %r\n' % self.error)


class FindScanParameters(object):
  """Calculates scan parameters such as minimum angle, maximum angle and phase offset.

  This class takes a sequence of numbers representing corresponding range
  measurements along a complete tilt period, i.e. all distances recorded while
  the laser tilts up and down exactly once. The algorithm assumes that the
  range measurements are taken by scanning a vertical plane, i.e. the robot is
  facing a wall.
  """

  def __init__(self, sensor_distance_from_rotation_axis,
               initial_low_angle=-1.0, initial_high_angle=1.0,
               initial_increasing_phase_offset=0.0,
               initial_decreasing_phase_offset=0.5):
    self._sensor_distance_from_rotation_axis = sensor_distance_from_rotation_axis
    self._initial_low_angle = initial_low_angle
    self._initial_high_angle = initial_high_angle
    self._initial_increasing_phase_offset = initial_increasing_phase_offset
    self._initial_decreasing_phase_offset = initial_decreasing_phase_offset

  def find_scan_parameters(self, scans):
    mean_distance_from_plane = sum(scan[0] for scan in scans) / len(scans)
    initial_parameters = [mean_distance_from_plane,
                          self._initial_low_angle,
                          self._initial_high_angle,
                          self._initial_increasing_phase_offset,
                          self._initial_decreasing_phase_offset,
                          1]  # stretch
    function = self._make_laser_error_function(scans)
    result, _, _, error_message, error_code = scipy.optimize.leastsq(function, initial_parameters, full_output=True)
    if error_code not in (1, 2, 3, 4):
      rospy.logerr(error_message)
      return
    (distance_from_plane, low_angle, high_angle, increasing_phase_offset,
        decreasing_phase_offset, stretch) = result
    predicted_angles = [self._angle_at_time(result, time) for _, time in scans]
    predicted_distances = [
        self._optimal_laser_distance(distance_from_plane, angle)
        for angle in predicted_angles]
    return ScanParameters(low_angle, high_angle,
                          increasing_phase_offset,
                          decreasing_phase_offset,
                          distance_from_plane,
                          stretch,
                          predicted_distances,
                          sum(v**2 for v in function(result)) / len(result))

  def _optimal_laser_distance(self, distance_from_plane, angle):
    """Evaluates the optimal laser function at angle and returns the
    corresponding distance.
    """
    return ((distance_from_plane - self._sensor_distance_from_rotation_axis * math.sin(angle)) /
            math.cos(angle))

  def _angle_at_time(self, parameters, time):
    """The angle function is a triangle starting at start_angle, increasing
    until time 0.5 and decreasing until time 1.0 at end_angle again.
    """
    low_angle, high_angle, increasing_phase_offset, decreasing_phase_offset, stretch = parameters[1:]
    x = math.fmod((time + increasing_phase_offset + 1) * stretch, 1)
    phase_offset = decreasing_phase_offset - increasing_phase_offset
    angle_delta = high_angle - low_angle
    if x > phase_offset:
      return high_angle - angle_delta * (x - phase_offset) / (1 - phase_offset)
    return low_angle + angle_delta * x / phase_offset

  def _make_laser_error_function(self, scans):
    """Returns a function for optimization.

    Returns a function used by the optimization algorithm. The function gets
    one parameter which is a sequence of all parameters to be optimized and
    returns the error for each scan when compared to the model.
    """

    def error_function(parameters):
      distance_from_plane = parameters[0]
      angles = []
      distances = []
      last_distance = scans[0][0]
      for distance, time in scans:
        if abs(distance - last_distance) > _MAXIMUM_DISTANCE_DELTA:
          continue
        last_distance = distance
        distances.append(distance)
        angles.append(self._angle_at_time(parameters, time))
      errors = [distance - self._optimal_laser_distance(distance_from_plane, angle)
                for distance, angle in zip(distances, angles)
                if _MINIMUM_DISTANCE < distance < _MAXIMUM_DISTANCE]
      errors.append((self._initial_low_angle - parameters[1]) * len(errors) * 1e-8)
      errors.append((self._initial_high_angle - parameters[2]) * len(errors) * 1e-8)
      return errors
    return error_function
