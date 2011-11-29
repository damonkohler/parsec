// Copyright 2011 Google Inc.
// Author: moesenle@google.com (Lorenz Moesenlechner)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "priority_mux/priority_mux.h"

#include <priority_mux_msgs/LogEntry.h>

namespace priority_mux {

PriorityMux::PriorityMux(const ros::NodeHandle &node_handle)
    : private_node_handle_(node_handle) {
  double timeout;
  private_node_handle_.param("timeout", timeout, kDefaultTimeout);
  timeout_ = ros::Duration(timeout);

  log_publisher_ = private_node_handle_.advertise<priority_mux_msgs::LogEntry>(
      "log", 10);
  double log_rate;
  private_node_handle_.param("log_rate", log_rate, kDefaultLogRate);
  log_timer_ = private_node_handle_.createTimer(
      ros::Duration(log_rate),
      boost::bind(&PriorityMux::LogTimerCallback, this, _1));
}

void PriorityMux::AddTopic(const std::string &topic) {
  size_t priority = expiring_subscriptions_.size();
  boost::mutex::scoped_lock lock(mutex_);
  ros::Subscriber subscriber = global_node_handle_.subscribe<topic_tools::ShapeShifter>(
      topic, 10, boost::bind(&PriorityMux::TopicCallback, this, priority, _1));
  expiring_subscriptions_.push_back(ExpiringSubscription(topic, priority, timeout_, subscriber));
}

void PriorityMux::TopicCallback(
    size_t priority, const topic_tools::ShapeShifter::ConstPtr &message) {
  if (priority > FindActivePriority()) {
    return;
  }
  boost::mutex::scoped_lock lock(mutex_);
  expiring_subscriptions_[priority].Ping();
  Republish(message);
}

void PriorityMux::LogTimerCallback(const ros::TimerEvent &) {
  boost::mutex::scoped_lock lock(mutex_);
  priority_mux_msgs::LogEntry log;
  log.header.stamp = ros::Time::now();
  for (size_t i = 0; i < expiring_subscriptions_.size(); i++) {
    priority_mux_msgs::TopicEntry entry;
    entry.name = expiring_subscriptions_[i].name();
    entry.priority = expiring_subscriptions_[i].priority();
    entry.duration = expiring_subscriptions_[i].runtime();
    log.topic_entries.push_back(entry);
  }
  log_publisher_.publish(log);
}

void PriorityMux::Republish(const topic_tools::ShapeShifter::ConstPtr &message) {
  if (!output_publisher_) {
    output_publisher_ = message->advertise(private_node_handle_, "output", 10);
  }
  output_publisher_.publish(message);
}

size_t PriorityMux::FindActivePriority() {
  boost::mutex::scoped_lock lock(mutex_);
  size_t i;
  for(i = 0; i < expiring_subscriptions_.size(); i++) {
    if (!expiring_subscriptions_[i].IsExpired()) {
      break;
    }
  }
  return i;
}

}  // namespace priority_mux
