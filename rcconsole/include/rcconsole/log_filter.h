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

#ifndef RCCONSOLE_LOG_FILTER_NODELET_H
#define RCCONSOLE_LOG_FILTER_NODELET_H

#include <boost/regex.hpp>
#include <nodelet/nodelet.h>

#include <rosgraph_msgs/Log.h>

namespace rcconsole {

class LogFilter : public nodelet::Nodelet {
 public:
  virtual void onInit();

 private:
  boost::regex regex_;
  ros::Subscriber log_subscriber_;
  ros::Publisher filtered_log_publisher_;

  void LogCallback(const rosgraph_msgs::Log::ConstPtr &log);

 protected:
  const boost::regex &regex() const {
    return regex_;
  }
  /**
   * Returns true if the log message should be filtered, i.e. if it
   * should _not_ be republished.
   */
  virtual bool FilterLog(const rosgraph_msgs::Log &log) = 0;
};

}  // namespace rcconsole

#endif  // RCCONSOLE_LOG_FILTER_NODELET_H
