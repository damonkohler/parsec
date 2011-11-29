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

#include "priority_mux/priority_mux.h"

#include <list>
#include <sstream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <std_msgs/Int32.h>

class PriorityMuxTest : public testing::Test {
 public:
  PriorityMuxTest()
      : node_handle_("~"),
        spinner_(1),
        priority_mux_(node_handle_) {}

 protected:
  static const size_t kNumberOfPublishers = 3;
  static const double kExpirationTimeout = 1.0;

  ros::NodeHandle node_handle_;
  ros::AsyncSpinner spinner_;
  priority_mux::PriorityMux priority_mux_;
  std::vector<ros::Publisher> priority_publishers_;
  ros::Subscriber priority_output_subscriber_;
  std::vector<int> received_messages_;
  boost::mutex mutex_;
  boost::condition_variable condition_;

  virtual void SetUp() {
    spinner_.start();
    node_handle_.setParam("timeout", kExpirationTimeout);
    for (size_t i = 0; i < kNumberOfPublishers; i++) {
      std::stringstream topic_name;
      topic_name << "topic_" << i;
      priority_publishers_.push_back(
          node_handle_.advertise<std_msgs::Int32>(topic_name.str(), 10));
      priority_mux_.AddTopic(node_handle_.resolveName(topic_name.str()));
    }
    priority_output_subscriber_ = node_handle_.subscribe<std_msgs::Int32>(
        "output", 10, boost::bind(&PriorityMuxTest::OutputCallback, this, _1));
  }

  void OutputCallback(const std_msgs::Int32::ConstPtr &data) {
    boost::mutex::scoped_lock lock(mutex_);
    received_messages_.push_back(data->data);
    condition_.notify_all();
  }

  void PublishWithPriority(size_t priority, int message) {
    std_msgs::Int32 data;
    data.data = message;
    ASSERT_LT(priority, priority_publishers_.size());
    priority_publishers_[priority].publish(data);
  }

  bool PublishWithPriorityAndWait(
      size_t priority, int message, const ros::Duration &timeout) {
    size_t received_messages_size = received_messages_.size();
    PublishWithPriority(priority, message);
    return WaitForMessagesWithTimeout(received_messages_size + 1, timeout);
  }

  bool WaitForMessagesWithTimeout(size_t n, const ros::Duration &timeout) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    while (received_messages_.size() < n) {
      if (!condition_.timed_wait(lock, timeout.toBoost())) {
        return false;
      }
    }
    return true;
  }
};

TEST_F(PriorityMuxTest, Topic1Relayed) {
  EXPECT_TRUE(PublishWithPriorityAndWait(0, 1, ros::Duration(0.1)));
  EXPECT_EQ(received_messages_[0], 1);
}

TEST_F(PriorityMuxTest, Topic2Relayed) {
  EXPECT_TRUE(PublishWithPriorityAndWait(1, 1, ros::Duration(0.1)));  
  EXPECT_EQ(received_messages_[0], 1);
}

TEST_F(PriorityMuxTest, Topic3Relayed) {
  EXPECT_TRUE(PublishWithPriorityAndWait(2, 1, ros::Duration(0.1)));    
  EXPECT_EQ(received_messages_[0], 1);
}

TEST_F(PriorityMuxTest, Priorities) {
  PublishWithPriorityAndWait(2, 2, ros::Duration(0.1));
  PublishWithPriorityAndWait(0, 0, ros::Duration(0.1));
  PublishWithPriorityAndWait(1, 1, ros::Duration(0.1));
  PublishWithPriorityAndWait(0, 0, ros::Duration(0.1));
  ros::Duration(kExpirationTimeout).sleep();
  PublishWithPriorityAndWait(2, 2, ros::Duration(0.1));
  EXPECT_EQ(received_messages_.size(), 4);
  EXPECT_EQ(received_messages_[0], 2);
  EXPECT_EQ(received_messages_[1], 0);
  EXPECT_EQ(received_messages_[2], 0);
  EXPECT_EQ(received_messages_[3], 2);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "priority_mux_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
