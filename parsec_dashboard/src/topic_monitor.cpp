// Copyright 2011 Google Inc.
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

#include "parsec_dashboarh/topic_monitor.h"

#include <list>
#include <string>

#include <ros/ros.h>

static const int kDefaultSpinThreadCount = 10;

namespace topic_monitor {

struct TopicConfiguration {
  std::string input_topic;
  double expected_frequency;
  
  TopicConfiguration()
    : expected_frequency(0.0) {}
  TopicConfiguration(const std::string &input_topic,
                     double expected_frequency)
    : input_topic(input_topic),
      expected_frequency(expected_frequency) {}
};

static bool ParseParams(
    ros::NodeHandle &node_handle, std::list<TopicConfiguration> *relay_configuration) {
  XmlRpc::XmlRpcValue topics;
  if (!node_handle.getParam("topics", topics)) {
    ROS_FATAL("Parameter not found: topics");
    return false;
  }
  if (relayed_topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Parameter must be a list: topics");
    return false;
  }

  for (int i = 0; i < topics.size(); i++) {
    if (topics[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_FATAL("List element must be a map: topics[%d]", i);
      return false;
    }
    
    std::string input_topic;
    if (relayed_topics[i]["name"].getType() !=
        XmlRpc::XmlRpcValue::TypeString) {
      ROS_FATAL("Invalid type. Expected string: topics[%d]/name", i);
      return false;
    }
    input_topic = static_cast<std::string>(relayed_topics[i]["name"]);

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
        TopicConfiguration(input_topic, expected_frequency));
  }
  return true;
}

}  // namespace topic_monitor

using namespace topic_monitor;
using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "topic_monitor");
  
  ros::AsyncSpinner spinner(kDefaultSpinThreadCount);
  spinner.start();

  ros::NodeHandle node_handle("~");
  std::list<TopicConfiguration> configuration;
  if (!ParseParams(node_handle, &configuration)) {
    return 1;
  }
  TopicRelay topic_monitor(node_handle);
  for (std::list<TopicConfiguration>::iterator it = topic_monitor.begin();
       it != topic_monitor.end(); it++) {
    ROS_INFO("Adding topic: %s", it->input_topic.c_str());
    topic_monitor.AddTopic(it->input_topic, it->expected_frequency);
  }
  topic_monitor.Run();

  return 0;
}
