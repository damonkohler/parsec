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

"""Action server providing an action for setting and changing waypoints."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

from threading import Lock

import rospy
import actionlib

from navigation_waypoints_server import move_base_proxy

import navigation_waypoints_server.msg
import navigation_waypoints_server.srv


class WaypointFailed(Exception):
  """Exception raised when we failed driving to a waypoint,
  i.e. when move_base reported a failure."""
  pass


class WaypointsServer(object):
  """Server for sending a driving to a sequence of waypoints."""

  def __init__(self):
    # List of pending waypoints. Required to be a member variable
    # because we need to be able to change it while executing the path
    # without preempting the action.
    self._pending = []
    self._lock = Lock()
        
    self._params = self._parse_params()
    self._execute_path = actionlib.SimpleActionServer(
        '~execute_path',
        navigation_waypoints_server.msg.ExecutePathAction,
        self._execute_action)
    self._update_waypoints_service = rospy.Service(
        '~update_waypoints', navigation_waypoints_server.srv.UpdateWaypoints, self._update_waypoints)
    self._move_base_proxies = [
        move_base_proxy.MoveBaseProxy(param['action'], param.get('check_plan'))
        for param in self._params['move_base_actions']]

  def _parse_params(self):
    """Currently, we support the following parameters:

    - base_frame: the name of the robot's base_link

    - move_base_actions: A list of maps that describe which move_base
      actions to use and, if required, a nav_msgs/GetPlan service to
      check if we actually want to call this service. The check_plan
      parameter is optional, the acton parameter mandatory. Example:
      [{'action': 'move_base_1', 'check_plan': 'move_base_1/make_plan'}]

    This function returns a dictionary of parameters.
    """
    return {
      'base_frame': rospy.get_param('~base_frame', 'base_link'),
      'move_base_actions': rospy.get_param('~move_base_actions', [{'action': '/move_base'}]),
      }

  def _execute_action(self, goal):
    visited = []
    invalid = []
    # We cannot acquire the lock for the complete method. If we
    # acquired the lock at the method's top-level, we would be unable
    # to update teh waypoints while we are executing.
    with self._lock:
      self._pending = goal.waypoints
    try:
      while not rospy.is_shutdown():
        try:
          with self._lock:
            if not self._pending:
              break
            current = self._pending[0]
            self._pending = self._pending[1:]
          self._execute_path.publish_feedback(
              navigation_waypoints_server.msg.ExecutePathFeedback(current=current,
                                                                  visited=visited,
                                                                  invalid=invalid,
                                                                  pending=self._pending))
          for proxy in self._move_base_proxies:
            self._execute_path.register_preempt_callback(proxy.interrupt)
            if proxy.maybe_check_plan(current) and proxy.execute(current):
              break
          else:
            raise WaypointFailed()
          visited += [current]

        except WaypointFailed, e:
          invalid += [current]
          if not goal.continue_on_error:
            raise e
      self._execute_path.set_succeeded(
          navigation_waypoints_server.msg.ExecutePathResult(visited=visited,
                                                            invalid=invalid,
                                                            pending=self._pending))
    except move_base_proxy.PreemptRequested:
      self._execute_path.set_preempted(
          navigation_waypoints_server.msg.ExecutePathResult(visited=visited,
                                                            invalid=invalid,
                                                            pending=[current] + self._pending))
    except WaypointFailed:
      self._execute_path.set_aborted(
          navigation_waypoints_server.msg.ExecutePathResult(visited=visited,
                                                            invalid=invalid,
                                                            pending=self._pending))

  def _update_waypoints(self, request):
    self._pending = request.waypoints
    return []
