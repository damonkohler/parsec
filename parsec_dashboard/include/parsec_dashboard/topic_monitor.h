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

#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H

#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace topic_monitor {

class TopicMonitor {
 public:
  TopicMonitor(const ros::NodeHandle &node_handle);
  virtual ~TopicMonitor();
  void AddTopic(const std::string &topic_name, 
                double min_frequency, 
                double max_frequency);
  void Run();

 private:
  struct MonitoredTopic {
    double min_frequency;
    double max_frequency;
    diagnostic_updater::TopicDiagnostic *diagnostic;
    ros::Subscriber subscriber;
  };

  ros::NodeHandle node_handle_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::vector<MonitoredTopic* > topics_;
  boost::mutex topics_mutex_;
 
  void MessageCallback(MonitoredTopic *topic,
                       const topic_tools::ShapeShifter::ConstPtr &message);
  void ConnectTopic(const std::string &topic_name);
};

}  // namespace topic_monitor

#endif  // TOPIC_MONITOR_H
