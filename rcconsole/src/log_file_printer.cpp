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

#include "rcconsole/log_file_printer.h"

#include <pluginlib/class_list_macros.h>

namespace rcconsole {

const std::string LogFilePrinter::kDefaultFormatString = "[%t %l %n]: %m";

void LogFilePrinter::onInit() {
  std::string filename;
  if (!getPrivateNodeHandle().getParam("filename", filename)) {
    ROS_FATAL("LogFilePrinter needs a file name.");
    return;
  }
  file_.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
  if (!file_) {
    ROS_FATAL("Unable to open file: %s", filename.c_str());
    return;
  }
  std::string format_string;
  getPrivateNodeHandle().param(
      "format_string", format_string, kDefaultFormatString);
  stream_printer_.reset(
      new LogStreamPrinter(
          getPrivateNodeHandle(), "rosout", file_, format_string));
}

}

PLUGINLIB_DECLARE_CLASS(rcconsole, LogFilePrinter, rcconsole::LogFilePrinter, nodelet::Nodelet);
