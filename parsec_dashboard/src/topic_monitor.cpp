// Copyright 2012 Google Inc.
// Author: duhadway@google.com (Charles DuHadway)
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

#include "parsec_dashboard/topic_monitor.h"

#include <diagnostic_updater/update_functions.h>
#include <topic_tools/shape_shifter.h>

namespace topic_monitor {

using ros::Subscriber;
using diagnostic_updater::FrequencyStatusParam;
using diagnostic_updater::TimeStampStatusParam;
using diagnostic_updater::TopicDiagnostic;

TopicMonitor::TopicMonitor(const ros::NodeHandle &node_handle)
  : node_handle_(node_handle), diagnostic_updater_() {
  diagnostic_updater_.setHardwareID("none");
}

TopicMonitor::~TopicMonitor() {
  boost::mutex::scoped_lock lock(topics_mutex_);
  for (size_t i = 0; i < topics_.size(); ++i) {
    MonitoredTopic *topic = topics_[i];
    delete topic->diagnostic;
    delete topic;
  }
}

void TopicMonitor::AddTopic(
    const std::string &topic_name, double min_frequency, double max_frequency) {
  MonitoredTopic *topic = new MonitoredTopic();

  topic->min_frequency = min_frequency;
  topic->max_frequency = max_frequency;
  topic->diagnostic = new TopicDiagnostic(
      topic_name,
      diagnostic_updater_,
      FrequencyStatusParam(&topic->min_frequency, &topic->max_frequency),
      TimeStampStatusParam(-1, 1.0));
  topic->subscriber = node_handle_.subscribe<topic_tools::ShapeShifter>(
      topic_name, 10, boost::bind(&TopicMonitor::MessageCallback, this, topic, _1));

  boost::mutex::scoped_lock lock(topics_mutex_);
  topics_.push_back(topic);
}

void TopicMonitor::MessageCallback(MonitoredTopic *topic,
    const topic_tools::ShapeShifter::ConstPtr &message) {
  topic->diagnostic->tick(ros::Time::now());
}

void TopicMonitor::Run() {
  while (node_handle_.ok()) {
    diagnostic_updater_.update();
    ros::WallDuration(1.0).sleep();
  }
}

}  // namespace topic_monitor
