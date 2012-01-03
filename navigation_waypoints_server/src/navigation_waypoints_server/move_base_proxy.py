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

"""Provides a proxy class for calling move_base."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import rospy
import actionlib

import nav_msgs.msg
import nav_msgs.srv
import move_base_msgs.msg as move_base_msgs
import geometry_msgs.msg as geometry_msgs


MOVE_BASE_POLL_TIMEOUT = 0.1
WAIT_FOR_PLAN_SERVICE_TIMEOUT = 2.0
PREEMPT_WAIT_FOR_TERMINATION_TIMEOUT = 2.0


class PreemptRequested(Exception):
  """Exception raised when the service client requests preemption."""
  pass


class MoveBaseProxy(object):
  """Proxy object for calling move_base.

  It also provides the method check_plan to find out if the
  corresponding move_base instance actually will be able to find a
  valid motion plan.
  """

  def __init__(self, action_name, check_plan_service_name=None):
    self._interrupted = False
    self._check_plan_service = None
    self._check_plan_service_name = check_plan_service_name
    self._move_base_action = actionlib.SimpleActionClient(
        action_name, move_base_msgs.MoveBaseAction)

  def interrupt(self):
    """Interrupts execution of the current goal.

    This triggers termination of the execute method.
    """
    self._interrupted = True
    
  def execute(self, goal, interrupt_poll_timeout=MOVE_BASE_POLL_TIMEOUT):
    """Executes the (move_base-)action while checking for cancellation.

    In case of cancellation, raises an exception. If the goal cannot
    be reached, returns False, otherwise True.
    """
    self._interrupted = False
    goal_msg = move_base_msgs.MoveBaseGoal(target_pose=goal)
    self._move_base_action.send_goal(goal_msg)
    while not (self._move_base_action.wait_for_result(rospy.Duration(interrupt_poll_timeout)) or
               rospy.is_shutdown()):
      if self._interrupted:
        self._move_base_action.cancel_goal()
        self._move_base_action.wait_for_result(
            rospy.Duration(PREEMPT_WAIT_FOR_TERMINATION_TIMEOUT))
        raise PreemptRequested()
    self._move_base_action.get_result()
    return self._move_base_action.get_state == actionlib.GoalStatus.SUCCEEDED

  def maybe_check_plan(self, goal):
    """Returns True if move_base will be able to find a global plan.

    This method executes the service passed to the constructor. If no
    service name has been passed, always returns True. If the service
    call fails for some reason, also returns False.
    """
    self._maybe_initialize_check_plan_service()
    if not self._check_plan_service:
      return True
    else:
      # We pass an empty PoseStampt as start here. According to the
      # move_base sourcecode, an empty frame_id means to use the
      # robot's current pose in move_base's reference frame which is
      # exactly what we want.
      try:
        plan = self._check_plan_service(start=geometry_msgs.PoseStamped(),
                                               goal=goal, tolerance=0.0)
      except rospy.ServiceException:
        rospy.logwarn('Service call failed: %r' % self._check_plan_service_name)
        return
      return len(plan.plan.poses) > 0

  def _maybe_initialize_check_plan_service(self):
    if (self._check_plan_service_name is not None and
        self._check_plan_service is None):
      try:
        rospy.wait_for_service(self._check_plan_service_name,
                               WAIT_FOR_PLAN_SERVICE_TIMEOUT)
      except rospy.ROSException:
        rospy.logfatal('Could not connect to service: %r' % self._check_plan_service_name)
        rospy.signal_shutdown('Fatal error')
      self._check_plan_service = rospy.ServiceProxy(
          self._check_plan_service_name, nav_msgs.srv.GetPlan)
