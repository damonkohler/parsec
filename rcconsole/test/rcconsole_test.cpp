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

#include "rcconsole/rcconsole.h"

#include <string>
#include <sstream>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>

class RcConsoleTest : public testing::Test {
 public:
  RcConsoleTest()
      : node_handle_("~"),
        spinner_(1),
        rcconsole_(node_handle_) {
  }

 protected:
  ros::NodeHandle node_handle_;
  ros::Publisher log_publisher_;
  ros::AsyncSpinner spinner_;
  rcconsole::RcConsole rcconsole_;
  
  virtual void SetUp() {
    spinner_.start();
    log_publisher_ = node_handle_.advertise<rosgraph_msgs::Log>(
        "/rosout_agg", 10);
    
  }

  void PublishLogMessage(
      uint8_t level, const std::string &name, const std::string &message) {
    rosgraph_msgs::Log log;
    log.header.stamp = ros::Time::now();
    log.level = level;
    log.name = name;
    log.msg = message;
    log_publisher_.publish(log);
  }

  void ProcessRosMessages() {
    ros::Duration(0.1).sleep();
  }
};

TEST_F(RcConsoleTest, IncludeNameFilter) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::INCLUDE_NAME_FILTER, "logger1");
  rcconsole_.SetOutputStream("%n", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "");
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "logger1\n");
}

TEST_F(RcConsoleTest, IncludeMessageFilter) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::INCLUDE_MESSAGE_FILTER, "message");
  rcconsole_.SetOutputStream("%m", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "message");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message2");
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(),
            "message\n"
            "message\n");
}

TEST_F(RcConsoleTest, ExcludeNameFilter) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::EXCLUDE_NAME_FILTER, "logger1");
  rcconsole_.SetOutputStream("%n", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "");
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "logger2\n");
}

TEST_F(RcConsoleTest, ExcludeMessageFilter) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::EXCLUDE_MESSAGE_FILTER, "message");
  rcconsole_.SetOutputStream("%m", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "message");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message2");
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "message2\n");
}

TEST_F(RcConsoleTest, FormatString) {
  std::stringstream log_output_;
  rcconsole_.SetOutputStream("%n %l %m", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger", "message");
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "logger INFO message\n");
}

TEST_F(RcConsoleTest, FilterChain) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::INCLUDE_NAME_FILTER, "logger1");
  rcconsole_.AddFilter(rcconsole::RcConsole::EXCLUDE_MESSAGE_FILTER, "message1");
  rcconsole_.SetOutputStream("%n %m", log_output_);
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message1");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "message1");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message2");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "message2");  
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "logger1 message2\n");
}

TEST_F(RcConsoleTest, SetOutputStreamOrder) {
  std::stringstream log_output_;
  rcconsole_.AddFilter(rcconsole::RcConsole::INCLUDE_MESSAGE_FILTER, ".*message.*");
  rcconsole_.SetOutputStream("%m", log_output_);
  rcconsole_.AddFilter(rcconsole::RcConsole::INCLUDE_MESSAGE_FILTER, ".*1.*");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger1", "message2");
  PublishLogMessage(rosgraph_msgs::Log::INFO, "logger2", "message1");
  // Give the system time to send the log message and execute all
  // callbacks.
  ProcessRosMessages();
  EXPECT_EQ(log_output_.str(), "message1\n");
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rcconsole_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
