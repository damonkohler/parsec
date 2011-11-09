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
import actionlib

from threading import Lock

import navigation_waypoints_server.msg
import navigation_waypoints_server.srv
import nav_msgs.msg
import nav_msgs.srv
import move_base_msgs.msg as move_base_msgs
import geometry_msgs.msg as geometry_msgs

MOVE_BASE_POLL_TIMEOUT = 0.1
WAIT_FOR_MOTION_PLAN_SERVICE_TIMEOUT = 2.0
PREEMPT_WAIT_FOR_TERMINATION_TIMEOUT = 2.0


class PreemptRequested(Exception):
  """Exception raised when the service client requests preemption."""
  pass


class WaypointFailed(Exception):
  """Exception raised when we failed driving to a waypoint,
  i.e. when move_base reported a failure."""
  pass


class MoveBaseProxy(object):
  """Proxy object for calling move_base

  It also provides the method check_motion_plan to find out if the
  corresponding move_base instance actually will be able to find a
  valid motion plan.
  """

  def __init__(self, action_name, check_motion_plan_service_name=None):
    self.interrupted = False
    self.check_motion_plan_service = None
    self.move_base_action = actionlib.SimpleActionClient(
        action_name, move_base_msgs.MoveBaseAction)
    if check_motion_plan_service_name is not None:
      rospy.wait_for_service(check_motion_plan_service_name,
                             WAIT_FOR_MOTION_PLAN_SERVICE_TIMEOUT)
      self.check_motion_plan_service = rospy.ServiceProxy(
          check_motion_plan_service_name, nav_msgs.srv.GetPlan)

  def interrupt(self):
    """Interrupts execution of the current goal.

    This triggers termination of the execute method.
    """
    self.interrupted = True

  def execute(self, goal, interrupt_poll_timeout=MOVE_BASE_POLL_TIMEOUT):
    """Executes the (move_base-)action while checking for cancellation.

    In case of cancellation, raises an exception. If the goal cannot
    be reached, returns False, otherwise True.
    """
    self.interrupted = False
    goal_msg = move_base_msgs.MoveBaseGoal(target_pose=goal)
    self.move_base_action.send_goal(goal_msg)
    while not self.move_base_action.wait_for_result(rospy.Duration(interrupt_poll_timeout)):
      if self.interrupted:
        self.move_base_action.cancel_goal()
        self.move_base_action.wait_for_result(
            rospy.Duration(PREEMPT_WAIT_FOR_TERMINATION_TIMEOUT))
        raise PreemptRequested()
    self.move_base_action.get_result()
    return self.move_base_action.get_state == actionlib.GoalStatus.SUCCEEDED

  def maybe_check_plan(self, goal):
    """Returns True if move_base will be able to find a global plan.

    This method executes the service passed to the constructor. If no
    service name has been passed, always returns True.
    """
    if not self.check_motion_plan_service:
      return True
    else:
      # We pass an empty PoseStampt as start here. According to the
      # move_base sourcecode, an empty frame_id means to use the
      # robot's current pose in move_base's reference frame which is
      # exactly what we want.
      plan = self.check_motion_plan_service(start=geometry_msgs.PoseStamped(),
                                            goal=goal, tolerance=0.0)
      return len(plan.plan.poses) > 0


class NavWaypointsServer(object):
  """Server for sending a driving to a sequence of waypoints."""

  def __init__(self, action_name):
    # List of pending waypoints. Required to be a member variable
    # because we need to be able to change it while executing the path
    # without preempting the action.
    self.pending = []
    self.lock = Lock()

    self.params = self._parse_params()
    self.execute_path = actionlib.SimpleActionServer(
        action_name,
        navigation_waypoints_server.msg.ExecutePathAction,
        self._execute_action)
    self.update_waypoints_service = rospy.Service(
        '~update_waypoints', navigation_waypoints_server.srv.UpdateWaypoints,
        self._update_waypoints)
    self.move_base_proxies = [MoveBaseProxy(param['action'], param.get('check_plan'))
                              for param in self.params['move_base_actions']]


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

    def make_interrupt_proxy_callback(move_base_proxy):
      def callback():
        move_base_proxy.interrupt()
      return callback

    visited = []
    invalid = []
    # We cannot acquire the lock for the complete method. If we
    # acquired the lock at the method's top-level, we would be unable
    # to update teh waypoints while we are executing.
    with self.lock:
      self.pending = goal.waypoints
    try:
      while True:
        try:
          with self.lock:
            if not self.pending:
              break
            current = self.pending[0]
            self.pending = self.pending[1:]
          self.execute_path.publish_feedback(
              navigation_waypoints_server.msg.ExecutePathFeedback(
                  current=current, visited=visited, invalid=invalid, pending=self.pending))
          for move_base_proxy in self.move_base_proxies:
            self.execute_path.register_preempt_callback(
                make_interrupt_proxy_callback(move_base_proxy))
            if move_base_proxy.maybe_check_plan(current) and move_base_proxy.execute(current):
              break
          else:
            raise WaypointFailed()
          visited += [current]

        except WaypointFailed, e:
          invalid += [current]
          if not goal.continue_on_error:
            raise e
      self.execute_path.set_succeeded(
          navigation_waypoints_server.msg.ExecutePathResult(
              visited=visited, invalid=invalid, pending=self.pending))
    except PreemptRequested, e:
      self.execute_path.set_preempted(
          navigation_waypoints_server.msg.ExecutePathResult(
              visited=visited, invalid=invalid, pending=[current] + self.pending))
    except WaypointFailed, e:
      self.execute_path.set_aborted(
          navigation_waypoints_server.msg.ExecutePathResult(
              visited=visited, invalid=invalid, pending=self.pending))

  def _update_waypoints(self, request):
    self.pending = request.waypoints
    return []


if __name__ == '__main__':
    rospy.init_node('navigation_waypoints_server')
    waypoints_server = NavWaypointsServer('~execute_path')
    rospy.spin()
