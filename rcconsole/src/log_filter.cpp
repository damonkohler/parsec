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

#include "rcconsole/log_filter.h"

namespace rcconsole {

void LogFilter::onInit() {
  std::string regex_string;
  if (!getPrivateNodeHandle().getParam("regex", regex_string)) {
    ROS_FATAL("Required parameter '%s' not found.",
              getPrivateNodeHandle().resolveName("regex").c_str());
    return;
  }
  regex_ = boost::regex(regex_string);
  log_subscriber_ = getPrivateNodeHandle().subscribe<rosgraph_msgs::Log>(
      "rosout", 100, boost::bind(&LogFilter::LogCallback, this, _1));
  filtered_log_publisher_ =
      getPrivateNodeHandle().advertise<rosgraph_msgs::Log>("rosout_filtered", 100);
}

void LogFilter::LogCallback(const rosgraph_msgs::Log::ConstPtr &log) {
  if (!FilterLog(*log)) {
    filtered_log_publisher_.publish(log);
  }
}

}  // namespace rcconsole
