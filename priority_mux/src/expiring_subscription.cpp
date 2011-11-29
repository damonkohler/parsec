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

#include "priority_mux/expiring_subscription.h"

namespace priority_mux {

ExpiringSubscription::ExpiringSubscription(
    const std::string name, int priority, ros::Duration timeout,
    const ros::Subscriber &subscriber)
    : name_(name), priority_(priority), timeout_(timeout),
      subscriber_(subscriber),
      runtime_(0.0) {
}

bool ExpiringSubscription::IsExpired() {
  return last_ping_time_ != ros::Time() &&
      ros::Time::now() - last_ping_time_ > timeout_;
}

void ExpiringSubscription::Ping() {
  ros::Time now = ros::Time::now();
  if (!IsExpired() && last_ping_time_ != ros::Time()) {
    runtime_ += now - last_ping_time_;
  }
  last_ping_time_ = now;
}

}  // namespace priority_mux
