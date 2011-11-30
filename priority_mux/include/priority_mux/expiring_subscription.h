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

#ifndef PRIORITY_MUX_EXPIRING_SUBSCRIPTION_H
#define PRIORITY_MUX_EXPIRING_SUBSCRIPTION_H

#include <string>

#include <ros/ros.h>

namespace priority_mux {

class ExpiringSubscription {
 public:
  ExpiringSubscription(
      const std::string name, int priority, ros::Duration timeout,
      const ros::Subscriber &subscriber);
  bool IsExpired();
  void Ping();

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

}  // namespace priority_mux

#endif  // PRIORITY_MUX_EXPIRING_SUBSCRIPTION_H
