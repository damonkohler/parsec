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

#include "rcconsole/logger.h"

#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>
#include <rosgraph_msgs/Log.h>

namespace rcconsole {

Logger::Logger(const ros::NodeHandle &node_handle, const std::string &node_name,
               ros::Duration repeat_duration)
    : node_name_(node_name),
      node_handle_(node_handle) {
  set_logger_level_client_ = ros::service::createClient<roscpp::SetLoggerLevel>(
      node_name_ + "/set_logger_level");
  update_logger_levels_timer_ = node_handle_.createTimer(
      ros::Duration(repeat_duration), boost::bind(&Logger::UpdateLoggerLevels, this, _1));
}

bool Logger::SetLoggerLevel(const std::string &logger_name, uint8_t log_level) {
  log_levels_[logger_name] = log_level;
  return UpdateLoggerLevel(logger_name, log_level);
}

bool Logger::SetLoggerLevelName(
    const std::string &logger_name, const std::string &log_level) {
  return SetLoggerLevel(logger_name, DecodeLoggerLevel(log_level));
}

bool Logger::GetLoggerLevel(const std::string &logger_name, uint8_t *log_level) {
  std::map<std::string, uint8_t>::iterator logger = log_levels_.find(logger_name);
  if (logger != log_levels_.end()) {
    *log_level = logger->second;
    return true;
  }
  std::list<roscpp::Logger> loggers;
  GetLoggerInfo(node_name_, &loggers);
  for (std::list<roscpp::Logger>::const_iterator it = loggers.begin();
       it != loggers.end(); it++) {
    if (it->name == logger_name) {
      *log_level = DecodeLoggerLevel(it->level);
      return true;
    }
  }
  return false;
}

std::list<std::string> Logger::ListNodes() {
  std::vector<std::string> node_names;
  ros::master::getNodes(node_names);
  std::list<std::string> result;
  for (size_t i = 0; i < node_names.size(); i++) {
    if (ros::service::exists(node_names[i] + "/get_loggers", false)) {
      result.push_back(node_names[i]);
    }
  }
  return result;
}

std::list<std::string> Logger::ListLoggers(const std::string &node_name) {
  std::list<roscpp::Logger> loggers;
  std::list<std::string> result;
  GetLoggerInfo(node_name, &loggers);
  for (std::list<roscpp::Logger>::const_iterator it = loggers.begin();
       it != loggers.end(); it++) {
    result.push_back(it->name);
  }
  return result;
}

void Logger::UpdateLoggerLevels(const ros::TimerEvent &) {
  for (std::map<std::string, uint8_t>::const_iterator it = log_levels_.begin();
       it != log_levels_.end(); it++) {
    SetLoggerLevel(it->first, it->second);
  }
}

bool Logger::UpdateLoggerLevel(const std::string &logger_name, uint8_t log_level) {
  if (!set_logger_level_client_.isValid()) {
    return false;
  }
  roscpp::SetLoggerLevel set_logger_level;
  set_logger_level.request.logger = logger_name;
  set_logger_level.request.level = EncodeLoggerLevel(log_level);
  return set_logger_level_client_.call(set_logger_level);
}

bool Logger::GetLoggerInfo(
    const std::string &node_name, std::list<roscpp::Logger> *loggers) {
  roscpp::GetLoggers get_loggers;
  std::string service_name = node_name + "/get_loggers";
  if (!ros::service::exists(service_name, false)) {
    return false;
  }
  if (!ros::service::call(service_name, get_loggers)) {
    return false;
  }
  for (size_t i = 0; i < get_loggers.response.loggers.size(); i++) {
    loggers->push_back(get_loggers.response.loggers[i]);
  }
  return true;
}

std::string Logger::EncodeLoggerLevel(uint8_t level) {
  switch (level) {
    case rosgraph_msgs::Log::DEBUG:
      return "DEBUG";
    case rosgraph_msgs::Log::INFO:
      return "INFO";
    case rosgraph_msgs::Log::WARN:
      return "WARN";
    case rosgraph_msgs::Log::ERROR:
      return "ERROR";
    case rosgraph_msgs::Log::FATAL:
      return "FATAL";
  }
  return "UNKNOWN";
}

uint8_t Logger::DecodeLoggerLevel(const std::string &level) {
  if (level == "DEBUG") {
    return rosgraph_msgs::Log::DEBUG;
  } else if (level == "INFO") {
    return rosgraph_msgs::Log::INFO;
  } else if (level == "WARN") {
    return rosgraph_msgs::Log::WARN;
  } else if (level == "ERROR") {
    return rosgraph_msgs::Log::WARN;
  } else if (level == "FATAL") {
    return rosgraph_msgs::Log::FATAL;
  }
  return 0;
}

}  // namespace rcconsole
