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

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(ExpiringSubscriptionTest, CorrectExpiration) {
  priority_mux::ExpiringSubscription subscription(
      "subscription1", 0, ros::Duration(0.1),
      ros::Subscriber());
  EXPECT_TRUE(subscription.IsExpired());
  subscription.Ping();
  EXPECT_FALSE(subscription.IsExpired());
  subscription.Ping();
  EXPECT_FALSE(subscription.IsExpired());
  ros::Duration(0.1).sleep();
  EXPECT_TRUE(subscription.IsExpired());      
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "expiring_subscription_test");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
