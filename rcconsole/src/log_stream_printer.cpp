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

#include "rcconsole/log_stream_printer.h"

#include <ros_check/ros_check.h>

namespace rcconsole {

LogStreamPrinter::LogStreamPrinter(
    const ros::NodeHandle &node_handle, const std::string &topic,
    std::ostream &stream, const std::string &format_string)
    : node_handle_(node_handle),
      stream_(stream), format_string_(format_string) {
  log_subscriber_ = node_handle_.subscribe<rosgraph_msgs::Log>(
      topic, 100, boost::bind(&LogStreamPrinter::LogCallback, this, _1));
}

void LogStreamPrinter::LogCallback(const rosgraph_msgs::Log::ConstPtr &log) {
  PrintLog(format_string_, *log, stream_);
}

std::ostream &LogStreamPrinter::PrintLog(
    const std::string &format_string, const rosgraph_msgs::Log &log,
    std::ostream &stream) {
  for (size_t i = 0; i < format_string.size(); i++ ) {
    if (format_string[i] == '%') {
      i++;
      CHECK_LT(i, format_string.size());
      PrintFormatFlag(format_string[i], log, stream);
    } else {
      stream << format_string[i];
    }
  }
  stream << std::endl;
  return stream;
}

std::ostream &LogStreamPrinter::PrintFormatFlag(
    char flag, const rosgraph_msgs::Log &log, std::ostream &stream) {
  switch (flag) {
    case 't':
      return PrintTime(log, stream);
    case 'l':
      return PrintLevel(log, stream);
    case 'n':
      return PrintName(log, stream);
    case 'o':
      return PrintTopics(log, stream);
    case 'm':
      return PrintMessage(log, stream);
    default:
      ROS_FATAL("Invalid format flag: %c", flag);
      CHECK(false);
  }
  return stream;
}

std::ostream &LogStreamPrinter::PrintTime(const rosgraph_msgs::Log &log, std::ostream &stream) {
  return stream << std::setiosflags(std::ios::fixed) << std::setprecision(9)
                << log.header.stamp.toSec();
}

std::ostream &LogStreamPrinter::PrintName(const rosgraph_msgs::Log &log, std::ostream &stream) {
  return stream << log.name;
}

std::ostream &LogStreamPrinter::PrintMessage(const rosgraph_msgs::Log &log, std::ostream &stream) {
  return stream << log.msg;
}

std::ostream &LogStreamPrinter::PrintLevel(const rosgraph_msgs::Log &log, std::ostream &stream) {
  switch (log.level) {
    case rosgraph_msgs::Log::DEBUG:
      return stream << "DEBUG";
    case rosgraph_msgs::Log::INFO:
      return stream << "INFO";
    case rosgraph_msgs::Log::WARN:
      return stream << "WARN";
    case rosgraph_msgs::Log::ERROR:
      return stream << "ERROR";
    case rosgraph_msgs::Log::FATAL:
      return stream << "FATAL";
  }
  return stream << log.level << " (Unknown)";
}

std::ostream &LogStreamPrinter::PrintTopics(const rosgraph_msgs::Log &log, std::ostream &stream) {
  for (size_t i = 0; i < log.topics.size(); i++) {
    stream << log.topics[i];
    if (i < log.topics.size() - 1) {
      stream << ", ";
    }
  }
  return stream;
}

}  // namespace rcconsole
