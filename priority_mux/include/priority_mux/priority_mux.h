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

#ifndef PRIORITY_MUX_PRIORITY_MUX_H
#define PRIORITY_MUX_PRIORITY_MUX_H

#include <vector>

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace priority_mux {

class PriorityMux {
 public:
  PriorityMux(const ros::NodeHandle &node_handle);
  void AddTopic(const std::string &topic);

 private:
  class PriorizedTopic {
   public:
    PriorizedTopic(const std::string name, int priority,
                   ros::Duration timeout,
                   const ros::Subscriber &subscriber)
        : name_(name), priority_(priority), timeout_(timeout),
          subscriber_(subscriber),
          runtime_(0.0) {}
    
    bool IsExpired() {
      return last_ping_time_ != ros::Time() &&
          ros::Time::now() - last_ping_time_ > timeout_;
    }

    void Ping() {
      ros::Time now = ros::Time::now();
      if (!IsExpired() && last_ping_time_ != ros::Time()) {
        runtime_ += now - last_ping_time_;
      }
      last_ping_time_ = now;
    }

    const std::string &name() const {
      return name_;
    }
    size_t priority() const {
      return priority_;
    }
    const ros::Duration &runtime() const {
      return runtime_;
    }

    std::string name_;
    size_t priority_;
    ros::Duration timeout_;
    ros::Subscriber subscriber_;
    ros::Time last_ping_time_;
    ros::Duration runtime_;
  };

  static const double kDefaultTimeout = 3.0;
  static const double kDefaultLogRate = 5.0;

  ros::NodeHandle global_node_handle_;
  ros::NodeHandle private_node_handle_;
  std::vector<PriorizedTopic> priorized_topics_;
  ros::Publisher output_publisher_;
  ros::Publisher log_publisher_;
  ros::Timer log_timer_;
  
  boost::mutex mutex_;
  ros::Duration timeout_;

  void TopicCallback(
      size_t priority, const topic_tools::ShapeShifter::ConstPtr &message);
  void LogTimerCallback(const ros::TimerEvent &);
  void Republish(const topic_tools::ShapeShifter::ConstPtr &message);
  size_t FindActivePriority();
};

}  // namespace priority_mux

#endif  // PRIORITY_MUX_PRIORITY_MUX_H
