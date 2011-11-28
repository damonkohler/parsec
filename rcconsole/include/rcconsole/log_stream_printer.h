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

#ifndef RCCONSOLE_LOG_STREAM_PRINTER_H
#define RCCONSOLE_LOG_STREAM_PRINTER_H

#include <ostream>
#include <string>

#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>

namespace rcconsole {

// Note(moesenle): We cannot make the printer a nodelet because it
// needs a stream as input.

/**
 * Class for printing rosconsole messages to the console, based on a
 * format string. The default format string is "%t %n: %m". Supported
 * format flag characters are:
 *
 *  t  time stamp
 *  n  logger name
 *  m  log message
 *  o  topics
 *  l  log level
 */
class LogStreamPrinter {
 public:
  LogStreamPrinter(
      const ros::NodeHandle &node_handle, const std::string &topic,
      std::ostream &stream, const std::string &format_string);

 private:
  ros::NodeHandle node_handle_;
  std::ostream &stream_;
  std::string format_string_;
  ros::Subscriber log_subscriber_;

  // This class should not be copyable. ROS subscriber callbacks are
  // bound to their instance by using boost bind which means that
  // copied subscribers stoped working as soon as the original class
  // is destructed.
  LogStreamPrinter(const LogStreamPrinter &other);

  void LogCallback(const rosgraph_msgs::Log::ConstPtr &log);
  std::ostream &PrintLog(
      const std::string &format_string, const rosgraph_msgs::Log &log,
      std::ostream &stream);
  std::ostream &PrintFormatFlag(char flag, const rosgraph_msgs::Log &log,
                                std::ostream &stream);
  std::ostream &PrintTime(const rosgraph_msgs::Log &log, std::ostream &stream);
  std::ostream &PrintName(const rosgraph_msgs::Log &log, std::ostream &stream);
  std::ostream &PrintMessage(const rosgraph_msgs::Log &log, std::ostream &stream);
  std::ostream &PrintLevel(const rosgraph_msgs::Log &log, std::ostream &stream);
  std::ostream &PrintTopics(const rosgraph_msgs::Log &log, std::ostream &stream);
};

}  // namespace rcconsole

#endif  // RCCONSOLE_LOG_STREAM_OUTPUT_H
