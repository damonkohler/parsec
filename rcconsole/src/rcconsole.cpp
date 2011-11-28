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

#include <map>
#include <sstream>
#include <vector>

#include <ros_check/ros_check.h>

namespace rcconsole {

RcConsole::RcConsole(const ros::NodeHandle &node_handle)
    : current_index_(0),
      node_handle_(node_handle),
      nodelet_loader_(node_handle_),
      current_output_topic_("/rosout_agg") {
}

void RcConsole::AddFilter(LogFilterType type, const std::string &regex) {
  std::string name = node_handle_.resolveName(MakeFilterName(type));
  SetupFilterParameters(name, regex);
  std::string nodelet_type_name = GetFilterNodeletFullName(type);
  std::map<std::string, std::string> remappings;
  std::vector<std::string> argv;
  remappings[name + "/rosout"] = current_output_topic_;
  CHECK(nodelet_loader_.load(name, nodelet_type_name, remappings, argv));
  current_output_topic_ = name + "/rosout_filtered";
  if (stream_printer_) {
    stream_printer_->Reconnect(current_output_topic_);
  }
}

void RcConsole::SetOutputStream(
    const std::string format_string, std::ostream &stream) {
  stream_printer_.reset(
      new LogStreamPrinter(node_handle_, current_output_topic_,
                           stream, format_string));
}

bool RcConsole::HasOutputStream() {
  return stream_printer_;
}

void RcConsole::SetupFilterParameters(
    const std::string &name, const std::string &regex) {
  ros::NodeHandle filter_node_handle(node_handle_, name);
  filter_node_handle.setParam("regex", regex);
}

std::string RcConsole::MakeFilterName(LogFilterType type) {
  std::stringstream name_stream;
  name_stream << GetFilterNodeletClassName(type) << "_" << current_index_;
  current_index_++;
  return name_stream.str();
}

std::string RcConsole::GetFilterNodeletFullName(LogFilterType type) {
  return "rcconsole/" + GetFilterNodeletClassName(type);
}

std::string RcConsole::GetFilterNodeletClassName(LogFilterType type) {
  switch (type) {
    case INCLUDE_NAME_FILTER:
      return "IncludeNameFilter";
    case INCLUDE_MESSAGE_FILTER:
      return "IncludeMessageFilter";
    case EXCLUDE_NAME_FILTER:
      return "ExcludeNameFilter";
    case EXCLUDE_MESSAGE_FILTER:
      return "ExcludeMessageFilter";
  }
}

}  // namespace rcconsole
