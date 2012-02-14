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

"""Coordinates movement through a mutable queue of waypoints."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import roslib; roslib.load_manifest('navigation_waypoints_server')
import rospy

from navigation_waypoints_server import waypoints_server


if __name__ == '__main__':
  rospy.init_node('navigation_waypoints_server')
  waypoints_server = waypoints_server.WaypointsServer()
  rospy.spin()
