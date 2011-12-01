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

#ifndef RCCONSOLE_SET_LOGGER_LEVEL_H
#define RCCONSOLE_SET_LOGGER_LEVEL_H

#include <list>
#include <string>

#include <ros/ros.h>

#include <roscpp/Logger.h>

namespace rcconsole {

class Logger {
 public:
  Logger(const ros::NodeHandle &node_handle, const std::string &node_name,
         ros::Duration repeat_duration);
  bool SetLoggerLevel(const std::string &logger_name, uint8_t log_level);
  bool SetLoggerLevelName(
      const std::string &logger_name, const std::string &log_level);
  bool GetLoggerLevel(const std::string &logger_name, uint8_t *log_level);

  static std::list<std::string> ListNodes();
  static std::list<std::string> ListLoggers(const std::string &node_name);
  
 private:
  std::string node_name_;
  std::map<std::string, uint8_t> log_levels_;
  ros::ServiceClient set_logger_level_client_;
  ros::NodeHandle node_handle_;
  ros::Timer update_logger_levels_timer_;

  void UpdateLoggerLevels(const ros::TimerEvent &);
  bool UpdateLoggerLevel(const std::string &logger_name, uint8_t log_level);

  static bool GetLoggerInfo(
      const std::string &node_name, std::list<roscpp::Logger> *loggers);
  static std::string EncodeLoggerLevel(uint8_t level);
  static uint8_t DecodeLoggerLevel(const std::string &level);
};

}  // namespace rcconsole

#endif  // RCCONSOLE_SET_LOGGER_LEVEL_H
