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

#ifndef RCCONSOLE_RCCONSOLE_H
#define RCCONSOLE_RCCONSOLE_H

#include <list>
#include <string>
#include <ostream>

#include <nodelet/loader.h>
#include <ros/ros.h>

#include "rcconsole/include_name_filter.h"
#include "rcconsole/include_message_filter.h"
#include "rcconsole/exclude_name_filter.h"
#include "rcconsole/exclude_message_filter.h"
#include "rcconsole/log_stream_printer.h"

namespace rcconsole {

class RcConsole {
 public:
  typedef enum {
    INCLUDE_NAME_FILTER, INCLUDE_MESSAGE_FILTER,
    EXCLUDE_NAME_FILTER, EXCLUDE_MESSAGE_FILTER
  } LogFilterType;

  RcConsole(const ros::NodeHandle &node_handle);
  void AddFilter(LogFilterType type, const std::string &regex);
  void SetOutputStream(const std::string format_string, std::ostream &stream);
  bool HasOutputStream();

  const std::string &current_output_topic() {
    return current_output_topic_;
  }

 private:
  // Index used to create unique filter names.
  unsigned int current_index_;
  ros::NodeHandle node_handle_;
  nodelet::Loader nodelet_loader_;
  std::string current_output_topic_;
  boost::shared_ptr<LogStreamPrinter> stream_printer_;

  RcConsole(const RcConsole &);
  void SetupFilterParameters(const std::string &name, const std::string &regex);
  std::string MakeFilterName(LogFilterType type);
  std::string GetFilterNodeletFullName(LogFilterType type);
  std::string GetFilterNodeletClassName(LogFilterType type);
};

}  // namespace rcconsole

#endif  // RCCONSOLE_RCCONSOLE_H
