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

#include <list>
#include <string>

#include <ros/ros.h>

static const int kDefaultSpinThreadCount = 10;

namespace robust_topic_relay {

struct TopicConfiguration {
  std::string input_topic;
  std::string output_topic;
  double expected_frequency;
  
  TopicConfiguration()
    : expected_frequency(0.0) {}
  TopicConfiguration(const std::string &input_topic,
                     const std::string &output_topic,
                     double expected_frequency)
    : input_topic(input_topic),
      output_topic(output_topic),
      expected_frequency(expected_frequency) {}
};

static bool ParseParams(
    ros::NodeHandle &node_handle, std::list<TopicConfiguration> *relay_configuration) {
  XmlRpc::XmlRpcValue relayed_topics;
  if (!node_handle.getParam("relayed_topics", relayed_topics)) {
    ROS_FATAL("Parameter not found: relayed_topics");
    return false;
  }
  if (relayed_topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Parameter must be a list: relayed_topics");
    return false;
  }

  for (int i = 0; i < relayed_topics.size(); i++) {
    if (relayed_topics[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_FATAL("List element must be a map: relayed_topics[%d]", i);
      return false;
    }
    
    std::string input_topic;
    if (relayed_topics[i]["input_topic"].getType() !=
        XmlRpc::XmlRpcValue::TypeString) {
      ROS_FATAL("Invalid type. Expected string: relayed_topics[%d]/input_topic", i);
      return false;
    }
    input_topic = static_cast<std::string>(relayed_topics[i]["input_topic"]);

    std::string output_topic;
    if (relayed_topics[i]["output_topic"].getType() !=
        XmlRpc::XmlRpcValue::TypeString) {
      ROS_FATAL("Invalid type. Expected string: relayed_topics[%d]/output_topic", i);
      return false;
    }
    output_topic = static_cast<std::string>(relayed_topics[i]["output_topic"]);

    double expected_frequency;
    if (relayed_topics[i]["expected_frequency"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble) {
      expected_frequency = static_cast<double>(relayed_topics[i]["expected_frequency"]);
    } else if (relayed_topics[i]["expected_frequency"].getType() ==
               XmlRpc::XmlRpcValue::TypeInt) {
      expected_frequency = static_cast<int>(relayed_topics[i]["expected_frequency"]);
    } else {
      ROS_FATAL("Invalid type. Expected number: "
                "relayed_topics[%d]/expected_frequency", i);
      return false;
    }
    relay_configuration->push_back(
      TopicConfiguration(input_topic, output_topic, expected_frequency));
  }
  return true;
}

}  // namespace robust_topic_relay

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robust_topic_relay");
  
  ros::AsyncSpinner spinners(kDefaultSpinThreadCount);
  spinners.start();

  ros::NodeHandle node_handle("~");
  std::list<robust_topic_relay::TopicConfiguration> relay_configuration;
  if (!robust_topic_relay::ParseParams(node_handle, &relay_configuration)) {
    return 1;
  }
  robust_topic_relay::RobustTopicRelay robust_topic_relay(node_handle);
  for (std::list<robust_topic_relay::TopicConfiguration>::iterator it =
           relay_configuration.begin();
       it != relay_configuration.end(); it++) {
    robust_topic_relay.AddTopic(it->input_topic, it->output_topic, it->expected_frequency);
  }
  robust_topic_relay.Run();

  return 0;
}
