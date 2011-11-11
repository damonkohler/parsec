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

"""Correct odometry by using another source of pose information"""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import rospy
import threading
import math
from tf import transformations

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs


class InvalidReferencePose(Exception):
  pass


class TimestampsInvalid(Exception):
  pass


def normalize_angle(angle):
  """Returnes an angle between -PI and PI."""
  if angle > 0:
    while angle > math.pi:
      angle -= 2 * math.pi
  else:
    while angle < -math.pi:
      angle += 2 * math.pi
  return angle


class OdometryErrorCorrector(object):
  """Class for calculating and correcting the error on an odometry message.

  It integrates the pose estimated by, e.g. an ICP based algorithm.
  """

  def __init__(self, maximal_linear_correction, maximal_angular_correction):
    self._maximal_linear_correction = maximal_angular_correction
    self._maximal_angular_correction = maximal_angular_correction
    # reference_pose is the pose used for calculating the error
    # between the last known odometry message and the pose. This error
    # is then used to correct new odometry messages.
    self._reference_pose = None
    # Buffer of last n odometry messages that are used for calculating
    # the error.
    self._odometry_messages = []
    # The pose error as a 3-tuple (x, y, theta).
    self._pose_error = (0, 0, 0)
    self._lock = threading.RLock()

  def set_reference_pose(self, new_reference_pose):
    """Sets a new reference pose.

    This method should be called whenever a new reference pose is
    available. It re-calculates the current pose error on odometry.
    """
    with self._lock:
      if (self._reference_pose is not None and
          new_reference_pose.header.stamp < self._reference_pose.header.stamp):
        raise InvalidReferencePose()
      self._reference_pose = new_reference_pose
    self._update_error()

  def add_odometry_message(self, odometry_message):
    with self._lock:
      self._odometry_messages.append(odometry_message)
    
  def calculate_corrected_odometry(self, odometry_message):
    """Corrects the odometry message and returns a new, corrected message."""
    self._update_error()
    corrected_odometry = nav_msgs.Odometry(header=odometry_message.header,
                                           child_frame_id=odometry_message.child_frame_id,
                                           twist=odometry_message.twist)
    corrected_odometry.pose.covariance = odometry_message.pose.covariance
    error_x, error_y, error_yaw = self._pose_error
    corrected_odometry.pose.pose.position.x = odometry_message.pose.pose.position.x + error_x
    corrected_odometry.pose.pose.position.y = odometry_message.pose.pose.position.y + error_y
    corrected_odometry.pose.pose.position.z = odometry_message.pose.pose.position.z
    roll, pitch, yaw = transformations.euler_from_quaternion(
        (odometry_message.pose.pose.orientation.x,
         odometry_message.pose.pose.orientation.y,
         odometry_message.pose.pose.orientation.z,
         odometry_message.pose.pose.orientation.w))
    corrected_odometry.pose.pose.orientation = geometry_msgs.Quaternion(
        *transformations.quaternion_from_euler(roll, pitch, yaw + error_yaw))
    return corrected_odometry

  def _update_error(self):
    with self._lock:
      if self._reference_pose is None:
        return
      enclosing_indices = self._find_enclosing_odometry_message_indices(self._reference_pose.header.stamp)
      if enclosing_indices is None:
        return
      interpolated_x, interpolated_y, interpolated_theta = self._interpolate_odometry_pose(
          self._odometry_messages[enclosing_indices[0]],
          self._odometry_messages[enclosing_indices[1]],
          self._reference_pose.header.stamp)
      # We can throw away all odometry messages before the first
      # enclosing index because new reference poses are only valid if
      # they are newer than the previous message.
      self._odometry_messages = self._odometry_messages[enclosing_indices[0]:]
  
      _, _, reference_theta = transformations.euler_from_quaternion(
          (self._reference_pose.pose.orientation.x,
           self._reference_pose.pose.orientation.y,
           self._reference_pose.pose.orientation.z,
           self._reference_pose.pose.orientation.w))
      old_error_x, old_error_y, old_error_theta = self._pose_error
      error_x = self._reference_pose.pose.position.x - interpolated_x
      error_y = self._reference_pose.pose.position.y - interpolated_y
      error_theta = normalize_angle(reference_theta - interpolated_theta)
      # The maximal adjustment of the old error is controled by the
      # parameters maximal_angular_correction and
      # maximal_linear_correction. Our new correction values must not
      # differ from the old error by more than the corretion parameter.
      if abs(error_x - old_error_x) < self._maximal_linear_correction:
        correction_x = error_x
      elif old_error_x < error_x:
        correction_x = old_error_x + self._maximal_linear_correction
      else:
        correction_x = old_error_x - self._maximal_linear_correction
  
      if abs(error_y - old_error_y) < self._maximal_linear_correction:
        correction_y = error_y
      elif old_error_y < error_y:
        correction_y = old_error_y + self._maximal_linear_correction
      else:
        correction_y = old_error_y - self._maximal_linear_correction
  
      if abs(normalize_angle(error_theta - old_error_theta)) < self._maximal_angular_correction:
        correction_theta = error_theta
      elif old_error_theta < error_theta:
        correction_theta = old_error_theta + self._maximal_angular_correction
      else:
        correction_theta = old_error_theta - self._maximal_angular_correction
        
      self._pose_error = (correction_x, correction_y, correction_theta)

  def _find_enclosing_odometry_message_indices(self, time):
    """Returns the two odometry messages right before time and right after time.

    Returns None on error.
    """
    left_index = None
    if (self._odometry_messages and
        time < self._odometry_messages[0].header.stamp):
      return
    with self._lock:
      for index, message in enumerate(self._odometry_messages):
        if message.header.stamp > time:
          break
        else:
          left_index = index
      else:
        return
      if left_index + 1 == len(self._odometry_messages):
        return
      return left_index, left_index + 1

  def _interpolate_odometry_pose(self, odometry_1, odometry_2, time):
    """Returns a tuple (x, y, theta) with the interpolated pose at time.

    The timestamp of odometry_1 needs to be smaller than odometry_2's stamp.
    """
    if (odometry_1.header.stamp > odometry_2.header.stamp or
        time < odometry_1.header.stamp or time >= odometry_2.header.stamp):
      print 'stamps invalid', odometry_1.header.stamp, time, odometry_2.header.stamp
      raise TimestampsInvalid()
    delta_t = ((time.to_sec() - odometry_1.header.stamp.to_sec()) /
               (odometry_2.header.stamp.to_sec() - odometry_1.header.stamp.to_sec()))
    _, _, yaw_1 = transformations.euler_from_quaternion((odometry_1.pose.pose.orientation.x,
                                                         odometry_1.pose.pose.orientation.y,
                                                         odometry_1.pose.pose.orientation.z,
                                                         odometry_1.pose.pose.orientation.w))
    _, _, yaw_2 = transformations.euler_from_quaternion((odometry_2.pose.pose.orientation.x,
                                                         odometry_2.pose.pose.orientation.y,
                                                         odometry_2.pose.pose.orientation.z,
                                                         odometry_2.pose.pose.orientation.w))
    return (odometry_1.pose.pose.position.x + (odometry_2.pose.pose.position.x - odometry_1.pose.pose.position.x) * delta_t,
            odometry_1.pose.pose.position.y + (odometry_2.pose.pose.position.x - odometry_1.pose.pose.position.x) * delta_t,
            normalize_angle(yaw_1 + (yaw_2 - yaw_1) * delta_t))
