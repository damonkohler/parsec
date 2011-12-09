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

import roslib; roslib.load_manifest('priority_teleop_mux')

import rospy

import geometry_msgs.msg as geometry_msgs
import priority_mux_msgs.msg as priority_mux_msgs

_TIMEOUT_TIMER_PERIOD = rospy.Duration(0.1)
_PRIORITY_TIMEOUT = rospy.Duration(3)
_LOG_TIMER_PERIOD = rospy.Duration(60)


class _PrioritizedTopic(object):

  def __init__(self, name, priority):
    self.name = name
    self.priority = priority
    self.duration = rospy.Duration()
    self._mark = None

  def mark(self):
    if self._mark is None:
      self._mark = rospy.Time.now()

  def reset(self):
    if self._mark is not None:
      self.duration += rospy.Time.now() - self._mark
      self._mark = None


class PriorityTeleopMux(object):

  def __init__(self, outgoing_topic):
    self._twist_publisher = rospy.Publisher(outgoing_topic, geometry_msgs.Twist)
    self._log_publisher = rospy.Publisher('~log', priority_mux_msgs.LogEntry)
    self._topics = []
    self._publisher_count = 0
    self._current_topic = None
    self._timeout = rospy.Duration(0)
    self._timeout_timer = rospy.Timer(_TIMEOUT_TIMER_PERIOD, self._update_timeout)
    self._log_timer = rospy.Timer(_LOG_TIMER_PERIOD, self._publish_log)

  def _publish_log(self, unused_event):
    log_entry = priority_mux_msgs.LogEntry()
    for topic in self._topics:
      topic_entry = priority_mux_msgs.TopicEntry()
      topic_entry.name = topic.name
      topic_entry.priority = topic.priority
      topic_entry.duration = topic.duration
      log_entry.topic_entries.append(topic_entry)
    self._log_publisher.publish(log_entry)

  def _update_timeout(self, unused_event):
    self._timeout -= _TIMEOUT_TIMER_PERIOD
    if self._timeout.to_sec() < 0:
      self._current_topic = None
      self._timeout = rospy.Duration(0)

  def add_topic(self, name):
    topic = _PrioritizedTopic(name, self._publisher_count)
    self._topics.append(topic)
    self._publisher_count += 1

    def callback(data):
      if (self._current_topic is None or
          self._current_topic.priority >= topic.priority):
        if self._current_topic != topic:
          topic.mark()
        self._current_topic = topic
        self._timeout = _PRIORITY_TIMEOUT
        self._twist_publisher.publish(data)
      else:
        topic.reset()
        rospy.logdebug('Dropped message on topic %r.' % topic.name)

    rospy.Subscriber(name, geometry_msgs.Twist, callback)


def main():
  rospy.init_node('priority_teleop_mux')
  outgoing_topic = rospy.get_param('~outgoing_topic')
  incoming_topics = rospy.get_param('~incoming_topics')
  priority_teleop_mux = PriorityTeleopMux(outgoing_topic)
  for name in incoming_topics:
    priority_teleop_mux.add_topic(name)
  rospy.spin()


if __name__ == '__main__':
  main()
