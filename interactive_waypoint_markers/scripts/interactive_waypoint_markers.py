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

"""Provides interactive markers to control navigation between waypoints."""

__author__ = 'moesenle@google.com (Lorenz Moesenlechner)'

import threading

import roslib; roslib.load_manifest("interactive_waypoint_markers")
import rospy
import actionlib

import navigation_waypoints_server.msg
import navigation_waypoints_server.srv
import geometry_msgs.msg

from interactive_markers import interactive_marker_server
from interactive_markers import menu_handler


class Waypoint(object):
  """Represents a waypoint.

  Attributes:
    name: the name of this waypoint
    pose: the pose of this waypoint
    active: is this the currently active waypoint?
  """

  def __init__(self, name, pose, active):
    self.name = name
    self.pose = pose
    self.active = active


class WaypointQueue(object):
  """Queue containing waypoints to be driven to

  This class manages the queue of waypoints we want to drive to. It
  also manages activation of a waypoint. Only the first waypoint in
  the queue is supposed to be active.

  Attributes:
    waypoints: sequence of waypoints
  """

  def __init__(self):
    self._waypoints = []
    self._lock = threading.Lock()

  def Add(self, name, pose):
    """Adds a waypoint to the end of the sequence of waypoints."""
    with self._lock:
      if not self._waypoints:
        new_waypoint = Waypoint(name, pose, active=True)
      else:
        new_waypoint = Waypoint(name, pose, active=False)
      self._waypoints.append(new_waypoint)
      return new_waypoint

  def Remove(self, name):
    """Removes a waypoint from our lists of waypoints and makes sure
    that a the new waypoint is activated. Also returns the removed
    waypoint to allow the user to handle deactivation of that waypoint
    and activation of the new first waypoint properly.
    """
    new_waypoints = []
    removed_waypoint = None
    with self._lock:
      for waypoint in self._waypoints:
        if waypoint.name != name:
          new_waypoints.append(waypoint)
        else:
          removed_waypoint = waypoint
      self._waypoints = new_waypoints
      if self._waypoints:
        self._waypoints[0].active = True
    return removed_waypoint

  def Get(self, name):
    with self._lock:
      for waypoint in self._waypoints:
        if waypoint.name == name:
          return waypoint

  def GetByPoseStamped(self, pose):
    with self._lock:
      for waypoint in self._waypoints:
        if self._PoseStampedEqual(pose, waypoint.pose):
          return waypoint

  def MarkActiveAndRemovePrevious(self, name):
    """Removes all waypoints before the waypoint to be marked.

    Returns: the list of removed waypoints
    """
    with self._lock:
      if not self._waypoints:
        return
      index = 0
      new_active = None
      for waypoint in self._waypoints:
        if waypoint.name == name:
          new_active = waypoint
          break
        index += 1
      if new_active:
        new_active.active = True
      removed = self._waypoints[:index]
      self._waypoints = self._waypoints[index:]
    return new_active, removed

  def Update(self, name, new_pose):
    waypoint = self.Get(name)
    waypoint.pose = new_pose
    return waypoint

  def GetWaypoints(self):
    with self._lock:
      return self._waypoints[:]

  def _PoseStampedEqual(self, p1, p2):
    return (p1.pose.position.x == p2.pose.position.x and
            p1.pose.position.y == p2.pose.position.y and
            p1.pose.position.z == p2.pose.position.z and
            p1.pose.orientation.x == p2.pose.orientation.x and
            p1.pose.orientation.y == p2.pose.orientation.y and
            p1.pose.orientation.z == p2.pose.orientation.z and
            p1.pose.orientation.w == p2.pose.orientation.w)


class InteractiveWaypointMarkers(object):
  """Interactive waypoint handling.

  This class creates an interactive waypoints server to create
  waypoints and an handles calling the action for following these
  waypoints.
  """

  def __init__(self):
    # Sequence number of goals.
    self._current_waypoint_index = 0

    # Flag to indicate if we are following a closed path, i.e. that
    # instead of deleting waypoints, we move them to the end of the
    # list.
    self._closed_path = False
    self._waypoints = WaypointQueue()

    self.server = interactive_marker_server.InteractiveMarkerServer('~markers')
    self.server.applyChanges()

    self.menu_handler = menu_handler.MenuHandler()
    self.menu_handler.insert("Delete", callback=self._DeleteWaypointCallback)
    self.menu_handler.setCheckState(
        self.menu_handler.insert(
            'Closed path', callback=self._ToggleClosedPath),
        menu_handler.MenuHandler.UNCHECKED)

    self.add_waypoint_sub = rospy.Subscriber(
        '~add_waypoint', geometry_msgs.msg.PoseStamped, self._AddWaypointCallback)

    self.execute_path = actionlib.SimpleActionClient(
        'navigation_waypoints_server/execute_path',
        navigation_waypoints_server.msg.ExecutePathAction)

    self.update_waypoints_srv = rospy.ServiceProxy(
        'navigation_waypoints_server/update_waypoints',
        navigation_waypoints_server.srv.UpdateWaypoints)

  def _AddWaypointCallback(self, pose):
    name = 'waypoint_%d' % self._current_waypoint_index
    description = 'waypoint %d' % self._current_waypoint_index
    self._current_waypoint_index += 1
    self._AddWaypoint(name, pose)

    waypoint_int_marker = interactive_marker_server.InteractiveMarker()
    waypoint_int_marker.header.frame_id = pose.header.frame_id
    waypoint_int_marker.name = name
    waypoint_int_marker.description = description
    waypoint_int_marker.pose = pose.pose

    waypoint_marker = interactive_marker_server.Marker()
    waypoint_marker.type = interactive_marker_server.Marker.ARROW
    waypoint_marker.scale.x = 0.8
    waypoint_marker.scale.y = 0.8
    waypoint_marker.scale.z = 0.8
    waypoint_marker.color.r = 0.0
    waypoint_marker.color.g = 0.8
    waypoint_marker.color.b = 0.0
    waypoint_marker.color.a = 1.0

    waypoint_control = interactive_marker_server.InteractiveMarkerControl()
    waypoint_control.always_visible = True
    waypoint_control.markers.append(waypoint_marker)
    waypoint_int_marker.controls.append(waypoint_control)

    translate_control = interactive_marker_server.InteractiveMarkerControl()
    translate_control.name = 'move_rotate_plane'
    translate_control.interaction_mode = interactive_marker_server.InteractiveMarkerControl.MOVE_ROTATE
    translate_control.orientation.x = 0
    translate_control.orientation.y = 1
    translate_control.orientation.z = 0
    translate_control.orientation.w = 1
    translate_control.always_visible = True
    waypoint_int_marker.controls.append(translate_control)

    self.server.insert(waypoint_int_marker, self._MarkerFeedbackCallback)
    self.menu_handler.apply(self.server, name)
    self.server.applyChanges()

  def _DeleteWaypointCallback(self, feedback):
    self._RemoveWaypoint(feedback.marker_name)
    self._EraseWaypointMarker(feedback.marker_name)

  def _ToggleClosedPath(self, feedback):
    state = self.menu_handler.getCheckState(feedback.menu_entry_id)
    if state == menu_handler.MenuHandler.CHECKED:
      self.menu_handler.setCheckState(feedback.menu_entry_id, menu_handler.MenuHandler.UNCHECKED)
      self._closed_path = False
    else:
      self._closed_path = True
      self.menu_handler.setCheckState(feedback.menu_entry_id, menu_handler.MenuHandler.CHECKED)
      try:
        self.menu_handler.reApply(self.server)
      except RuntimeError:
        # reApply has a bug. It tries to remove elements from a
        # set it iterates over. Catch and ignore the error. This
        # is ugly since we might lose some deletions for a while.
        # See https://code.ros.org/trac/ros-pkg/ticket/5203
        pass
      self.server.applyChanges()

  def _MarkerFeedbackCallback(self, feedback):
    if feedback.event_type == feedback.MOUSE_UP:
      self._UpdateWaypointWithPose(feedback.marker_name, feedback.pose)

  def _SendNewOrUpdateCurrentNavGoal(self, waypoint):
    if waypoint.active:
      self._SendNewNavGoal()
    else:
      self._UpdateCurrentNavGoal()

  def _AddWaypoint(self, name, pose):
    new_waypoint = self._waypoints.Add(name, pose)
    self._SendNewOrUpdateCurrentNavGoal(new_waypoint)

  def _UpdateWaypointWithPose(self, name, new_pose):
    old_pose_stamped = self._waypoints.Get(name).pose
    new_pose_stamped = geometry_msgs.msg.PoseStamped(
        header=old_pose_stamped.header,
        pose=new_pose)
    updated_waypoint = self._waypoints.Update(name, new_pose_stamped)
    self._SendNewOrUpdateCurrentNavGoal(updated_waypoint)

  def _RemoveWaypoint(self, name):
    removed_waypoint = self._waypoints.Remove(name)
    self._SendNewOrUpdateCurrentNavGoal(removed_waypoint)

  def _EraseWaypointMarker(self, name):
    # _very_ ugly hack. It seems like interactive_markers has a
    # bug. It's erase method is broken. To still be able to erase,
    # we handle this by hand. We just set the UpdateContext type
    # to erase after re-inserting the marker again. See ticket at
    # https://code.ros.org/trac/ros-pkg/ticket/5193
    marker = self.server.get(name)
    self.server.insert(marker)
    self.server.pending_updates[name].update_type = interactive_marker_server.UpdateContext.ERASE
    self.server.applyChanges()
    try:
      self.menu_handler.reApply(self.server)
    except RuntimeError:
      # reApply has a bug. It tries to remove elements from a
      # set it iterates over. Catch and ignore the error. This
      # is ugly since we might lose some deletions for a while.
      pass

  def _SendNewNavGoal(self):
    """Sends a new waypoints goal and cancels the current one if we
    are still runnging.
    """
    if self.execute_path.simple_state != actionlib.SimpleGoalState.DONE:
      self.execute_path.cancel_goal()
      # We need to wait until the goal was really canceled
      if not self.execute_path.wait_for_result(rospy.Duration(2)):
        rospy.logwarn('Cancellation of goal took more than 2 seconds. Continuing anyway.')
    self.execute_path.send_goal(
      navigation_waypoints_server.msg.ExecutePathGoal(
        waypoints=[wp.pose for wp in self._waypoints.GetWaypoints()],
        continue_on_error=True),
      done_cb=self._NavigationDoneCallback,
      feedback_cb=self._NavigationFeedbackCallback)

  def _UpdateCurrentNavGoal(self):
    """Updates all waypoints on the server that are not active at the moment"""
    self.update_waypoints_srv(waypoints=[wp.pose for wp in self._waypoints.GetWaypoints() if not wp.active])
  
  def _NavigationDoneCallback(self, state, result):
    if state != actionlib.GoalStatus.PREEMPTED:
      for wp in self._waypoints.GetWaypoints():
        self._EraseWaypointMarker(wp.name)
        self._waypoints.Remove(wp.name)

  def _NavigationFeedbackCallback(self, feedback):
    current = self._waypoints.GetByPoseStamped(feedback.current)
    _, removed_waypoints = self._waypoints.MarkActiveAndRemovePrevious(current.name)
    if self._closed_path:
      for removed in removed_waypoints:
        self._waypoints.Add(removed.name, removed.pose)
      self._UpdateCurrentNavGoal()
    else:
      for removed in removed_waypoints:
        self._EraseWaypointMarker(removed.name)


if __name__=="__main__":
    rospy.init_node("interactive_waypoint_markers")
    markers = InteractiveWaypointMarkers()
    rospy.spin()
