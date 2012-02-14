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

#ifndef ROBUST_TOPIC_RELAY_ROBUST_TOPIC_RELAY_H
#define ROBUST_TOPIC_RELAY_ROBUST_TOPIC_RELAY_H

#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace robust_topic_relay {

class RobustTopicRelay {
 public:
  RobustTopicRelay(const ros::NodeHandle &node_handle);
  void AddTopic(
      const std::string &input_topic_name, const std::string &output_topic_name,
      double expected_frequency, double reconnect_frequency);
  void Run();

 private:
  struct RelayedTopic {
    std::string input_topic_name;
    std::string output_topic_name;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::Time expiration_time;
    ros::Duration expected_delay;
    ros::Duration reconnect_delay;
    bool connected;

    RelayedTopic() {}
    RelayedTopic(const std::string &input_topic_name,
                 const std::string &output_topic_name,
                 const ros::Duration &expected_delay,
                 const ros::Duration &reconnect_delay)
      : input_topic_name(input_topic_name),
        output_topic_name(output_topic_name),
        expected_delay(expected_delay),
        reconnect_delay(reconnect_delay),
        connected(false) {}
  };

  ros::NodeHandle node_handle_;
  std::map<std::string, RelayedTopic> relayed_topics_;
  std::map<ros::Time, std::string> expiring_topics_;
  boost::mutex mutex_;
  
  void MessageCallback(
      const std::string topic_name, const topic_tools::ShapeShifter::ConstPtr &message);
  bool FindNextExpirationDuration(ros::Duration *expire_duration);
  void ReconnectExpiredTopics();
  void ConnectRelayedTopic(const std::string &topic_name);
};

}  // namespace robust_topic_relay

#endif  // ROBUST_TOPIC_RELAY_ROBUST_TOPIC_RELAY_H
