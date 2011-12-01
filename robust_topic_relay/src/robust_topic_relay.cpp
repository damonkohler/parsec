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

#include "robust_topic_relay/robust_topic_relay.h"

#include <ros_check/ros_check.h>
#include <topic_tools/shape_shifter.h>

namespace robust_topic_relay {

RobustTopicRelay::RobustTopicRelay(const ros::NodeHandle &node_handle)
  : node_handle_(node_handle) {
}

void RobustTopicRelay::AddTopic(
    const std::string &input_topic_name, const std::string &output_topic_name,
    double expected_frequency) {
  RobustTopicRelay::RelayedTopic relayed_topic(
      input_topic_name, output_topic_name, ros::Duration(1 / expected_frequency));

  boost::mutex::scoped_lock lock(mutex_);
  CHECK(relayed_topics_.find(input_topic_name) == relayed_topics_.end());
  relayed_topics_[input_topic_name] = relayed_topic;
  ConnectRelayedTopic(input_topic_name);
}

void RobustTopicRelay::Run() {
  while (ros::ok()) {
    ros::Duration expire_duration;
    if (!FindNextExpirationDuration(&expire_duration)) {
        ros::Duration(0.1).sleep();
        continue;
    }
    if (expire_duration < ros::Duration(0)) {
      ReconnectExpiredTopics();
      continue;
    }
    expire_duration.sleep();
  }
}

void RobustTopicRelay::MessageCallback(
    const std::string topic_name, const topic_tools::ShapeShifter::ConstPtr &message) {
  RobustTopicRelay::RelayedTopic &relayed_topic = relayed_topics_[topic_name];
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (relayed_topic.expiration_time == ros::Time()) {
      ROS_INFO("Connected to topic: %s", topic_name.c_str());
    }
    ros::Time next_expiration_time = ros::Time::now() + relayed_topic.expected_delay;
    expiring_topics_.erase(relayed_topic.expiration_time);
    relayed_topic.expiration_time = next_expiration_time;
    expiring_topics_[next_expiration_time] = topic_name;
  }
  if (!relayed_topic.publisher) {
    relayed_topic.publisher = message->advertise(
        node_handle_, relayed_topic.output_topic_name, 10);
  }
  relayed_topic.publisher.publish(message);
}

bool RobustTopicRelay::FindNextExpirationDuration(ros::Duration *expire_duration) {
  boost::mutex::scoped_lock lock(mutex_);

  if (expiring_topics_.empty() ||
      expiring_topics_.begin()->first == ros::Time()) {
    return false;
  }
  *expire_duration = expiring_topics_.begin()->first - ros::Time::now();
  return true;
}

void RobustTopicRelay::ReconnectExpiredTopics() {
  boost::mutex::scoped_lock lock(mutex_);

  while (!expiring_topics_.empty() &&
         expiring_topics_.begin()->first < ros::Time::now()) {
    std::string topic_name = expiring_topics_.begin()->second;
    ROS_INFO("Reconnecting expired topic: %s", topic_name.c_str());
    relayed_topics_[topic_name].expiration_time = ros::Time();
    relayed_topics_[topic_name].subscriber.shutdown();
    expiring_topics_.erase(expiring_topics_.begin());
    ConnectRelayedTopic(topic_name);
  }
}

void RobustTopicRelay::ConnectRelayedTopic(const std::string &topic_name) {
  std::map<std::string, RobustTopicRelay::RelayedTopic>::iterator relayed_topic =
      relayed_topics_.find(topic_name);
  CHECK(relayed_topic != relayed_topics_.end());
  relayed_topic->second.subscriber = node_handle_.subscribe<topic_tools::ShapeShifter>(
      topic_name, 10, boost::bind(&RobustTopicRelay::MessageCallback, this, topic_name, _1));
}

}  // namespace robust_topic_relay
