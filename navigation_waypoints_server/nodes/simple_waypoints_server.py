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

"""Provides a simple PoseStamped interface to navigation_waypoitns_server."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import roslib; roslib.load_manifest('navigation_waypoints_server')
import rospy
import actionlib

import geometry_msgs.msg as geometry_msgs
import navigation_waypoints_server.msg


class SimpleWaypointsServer(object):
  """Simple node that calls the navigation_waypoints_server.

  Receives a goal from a pose_stamped topic and calls the
  navigation_waypoints_server with it."""

  def __init__(self):
    self._navigation_waypoints_server = actionlib.SimpleActionClient(
        '~execute_path', navigation_waypoints_server.msg.ExecutePathAction)
    self._goal_subscriber = rospy.Subscriber(
        '~goal_pose', geometry_msgs.PoseStamped, self._goal_callback)

  def _goal_callback(self, goal):
    action_goal = navigation_waypoints_server.msg.ExecutePathGoal(
        waypoints=[goal])
    self._navigation_waypoints_server.send_goal(action_goal)


if __name__ == '__main__':
  rospy.init_node('simple_waypoints_server')
  simple_waypoints_server = SimpleWaypointsServer()
  rospy.spin()
