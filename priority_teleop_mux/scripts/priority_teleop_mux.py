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

"""Multiplexes teleop topics based on priority."""

__author__ = 'damonkohler@google.com (Damon Kohler)'

import sys
import threading

import roslib; roslib.load_manifest('priority_teleop_mux')

import rospy

import geometry_msgs.msg as geometry_msgs

_TIMER_PERIOD = rospy.Duration(0.1)
_PRIORITY_TIMEOUT = rospy.Duration(3)


class _PrioritizedTopic(object):

  def __init__(self, priority):
    self.priority = priority


class PriorityTeleopMux(object):

  def __init__(self, outgoing_topic):
    self._publisher = rospy.Publisher(outgoing_topic, geometry_msgs.Twist)
    self._topics = []
    self._publisher_count = 0
    self._priority = None
    self._timeout = rospy.Duration(0)
    self._timer = rospy.Timer(_TIMER_PERIOD, self._update_timeout)

  def _update_timeout(self, unused_event):
    self._timeout -= _TIMER_PERIOD
    if self._timeout.to_sec() < 0:
      self._priority = None
      self._timeout = rospy.Duration(0)

  def add_topic(self, incoming_topic):
    topic = _PrioritizedTopic(self._publisher_count)
    self._topics.append(topic)
    self._publisher_count += 1

    def callback(data):
      if (self._priority is None or
          self._priority >= topic.priority):
        self._priority = topic.priority
        self._timeout = _PRIORITY_TIMEOUT
        self._publisher.publish(data)
      else:
        print 'dropped ' + str(data)

    subscriber = rospy.Subscriber(incoming_topic, geometry_msgs.Twist, callback)


def main():
  rospy.init_node('priority_teleop_mux')
  outgoing_topic = rospy.get_param('~outgoing_topic')
  incoming_topics = rospy.get_param('~incoming_topics')
  priority_teleop_mux = PriorityTeleopMux(outgoing_topic)
  for incoming_topic in incoming_topics:
    priority_teleop_mux.add_topic(incoming_topic)
  rospy.spin()


if __name__ == '__main__':
  main()
