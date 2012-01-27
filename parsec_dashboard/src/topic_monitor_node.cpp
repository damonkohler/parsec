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

#include "parsec_dashboard/topic_monitor.h"

#include <list>
#include <string>

#include <ros/ros.h>

static const int kDefaultSpinThreadCount = 10;

namespace topic_monitor {

struct TopicConfiguration {
  std::string topic;
  double min_frequency;
  double max_frequency;
  
  TopicConfiguration()
    : min_frequency(0.0) {}
  TopicConfiguration(const std::string &topic,
                     double min_frequency,
                     double max_frequency)
    : topic(topic),
      min_frequency(min_frequency),
      max_frequency(max_frequency) {}
};

static bool ParseParams(
    ros::NodeHandle &node_handle, std::list<TopicConfiguration> *configuration) {
  XmlRpc::XmlRpcValue topics;
  if (!node_handle.getParam("topics", topics)) {
    ROS_FATAL("Parameter not found: topics");
    return false;
  }
  if (topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Parameter must be a list: topics");
    return false;
  }

  for (int i = 0; i < topics.size(); i++) {
    if (topics[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_FATAL("List element must be a map: topics[%d]", i);
      return false;
    }
    
    std::string topic;
    if (topics[i]["name"].getType() !=
        XmlRpc::XmlRpcValue::TypeString) {
      ROS_FATAL("Invalid type. Expected string: topics[%d]/name", i);
      return false;
    }
    topic = static_cast<std::string>(topics[i]["name"]);

    double min_frequency;
    if (topics[i]["min_frequency"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble) {
      min_frequency = static_cast<double>(topics[i]["min_frequency"]);
    } else if (topics[i]["min_frequency"].getType() ==
               XmlRpc::XmlRpcValue::TypeInt) {
      min_frequency = static_cast<int>(topics[i]["min_frequency"]);
    } else {
      ROS_FATAL("Invalid type. Expected number: "
                "topics[%d]/min_frequency", i);
      return false;
    }

    double max_frequency;
    if (topics[i]["max_frequency"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble) {
      max_frequency = static_cast<double>(topics[i]["max_frequency"]);
    } else if (topics[i]["max_frequency"].getType() ==
               XmlRpc::XmlRpcValue::TypeInt) {
      max_frequency = static_cast<int>(topics[i]["max_frequency"]);
    } else {
      ROS_FATAL("Invalid type. Expected number: "
                "topics[%d]/max_frequency", i);
      return false;
    }

    configuration->push_back(
        TopicConfiguration(topic, min_frequency, max_frequency));
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
  TopicMonitor topic_monitor(node_handle);
  for (std::list<TopicConfiguration>::iterator it = configuration.begin();
       it != configuration.end(); it++) {
    ROS_INFO("Adding topic: %s", it->topic.c_str());
    topic_monitor.AddTopic(it->topic, it->min_frequency, it->max_frequency);
  }
  topic_monitor.Run();

  return 0;
}
