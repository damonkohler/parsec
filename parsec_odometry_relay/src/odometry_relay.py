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

"""Republish parsec_msgs/ParsecOdometry as nav_msgs/Odometry."""

__author__ = 'damonkohler@google.com (Damon Kohler)'

import roslib
roslib.load_manifest('parsec_odometry_relay')

import rospy
import math

import odometry_error_corrector

from parsec_msgs.msg import Odometry as ParsecOdometry
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, PoseStamped


class OdometryRelay(object):
  """Class for relaying /odom_simple to /odom.

  The resulting odometry message (currently) has an empty covarience
  matrix.

  ROS Parameters:

    base_frame_id: name of the robot's base frame, default is 'base_link'
    publish_tf: when true, publishes the transform from /odom to
        base_frame_id, default is true
  """

  def __init__(self):
    self._odometry_publisher = rospy.Publisher('odom', Odometry)
    self._tf_publisher = rospy.Publisher('/tf', tfMessage)
    self._child_frame_id = rospy.get_param('~base_frame_id', 'base_link')
    self._publish_tf = rospy.get_param('~publish_tf', True)
    maximal_linear_correction = rospy.get_param('~maximal_linear_correction', 0.1)
    maximal_angular_correction = rospy.get_param('~maximal_angular_correction', 10.0 * math.pi / 180)
    rospy.loginfo('Using base frame %s' % self._child_frame_id)
    self._odometry_subscriber = rospy.Subscriber(
        'odom_simple', ParsecOdometry, self._relay_odometry_callback)
    self._odometry_error_corrector = odometry_error_corrector.OdometryErrorCorrector(
        maximal_linear_correction, maximal_angular_correction)
    self._correction_pose_subscriber = rospy.Subscriber(
        '~correction_pose', PoseStamped, self._correction_pose_callback)

  def _relay_odometry_callback(self, data):
    odometry = Odometry()
    odometry.header = data.header
    odometry.child_frame_id = self._child_frame_id
    odometry.pose.pose.position.x = data.position_x
    odometry.pose.pose.position.y = data.position_y
    odometry.pose.pose.orientation.z = data.orientation_z
    odometry.pose.pose.orientation.w = data.orientation_w
    odometry.twist.twist.linear.x = data.linear_x
    odometry.twist.twist.linear.y = data.linear_y
    odometry.twist.twist.angular.z = data.angular_z
    corrected_odometry = self._odometry_error_corrector.update_odometry(odometry)
    self._odometry_publisher.publish(corrected_odometry)

    if self._publish_tf:
      transform = TransformStamped()
      transform.header = data.header
      transform.child_frame_id = self._child_frame_id
      transform.transform.translation = corrected_odometry.pose.pose.position
      transform.transform.rotation = corrected_odometry.pose.pose.orientation
      
      transform_uncorrected = TransformStamped()
      transform_uncorrected.header = data.header
      transform_uncorrected.child_frame_id = self._child_frame_id + '_uncorrected'
      transform_uncorrected.transform.translation = odometry.pose.pose.position
      transform_uncorrected.transform.rotation = odometry.pose.pose.orientation

      self._tf_publisher.publish(tfMessage(transforms=[transform, transform_uncorrected]))

  def _correction_pose_callback(self, pose):
    self._odometry_error_corrector.set_reference_pose(pose)


if __name__ == '__main__':
  rospy.init_node('parsec_odometry_relay')
  relay = OdometryRelay()
  rospy.spin()
