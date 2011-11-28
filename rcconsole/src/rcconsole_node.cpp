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

#include <getopt.h>

#include <iostream>
#include <list>
#include <string>

#include <ros/ros.h>

#include "rcconsole/rcconsole.h"

namespace rcconsole {

static const std::string kDefaultFormatString = "[%t %l %n] %m";
static const int kFormatStringArgument = RcConsole::EXCLUDE_MESSAGE_FILTER + 1;

static option program_options[] = {
  {"include-name", required_argument, NULL, RcConsole::INCLUDE_NAME_FILTER},
  {"include-message", required_argument, NULL, RcConsole::INCLUDE_MESSAGE_FILTER},
  {"exclude-name", required_argument, NULL, RcConsole::EXCLUDE_NAME_FILTER},
  {"exclude-message", required_argument, NULL, RcConsole::EXCLUDE_MESSAGE_FILTER},
  {"format-string", required_argument, NULL, kFormatStringArgument},
  {"help", no_argument, NULL, 'h'}
};

static void PrintUsage(const std::string &program_name) {
  std::cout << "Usage: " << program_name << " [OPTION]" << std::endl;
  std::cout << std::endl <<
      "  --include-name=regex" << std::endl <<
      "\t only print if logger name matches regex" << std::endl <<
      "  --include-message=regex" << std::endl <<
      "\t only print if message matches regex" << std::endl <<
      "  --exlude-name=regex" << std::endl <<
      "\t don't print loggers that match regex" << std::endl <<
      "  --exlude-message=regex" << std::endl <<
      "\t don't print messages that match regex" << std::endl <<
      "  --format_string=string" << std::endl <<
      "\t print using the format string."<< std::endl <<
      std::endl <<
      "Include and exclude patterns are process in-order to build up a "
      "filter chain." << std::endl <<
      "Special format flags that can be used in the format string are:" <<
      std::endl <<
      "  %t    time stamp" << std::endl <<
      "  %n    logger name" << std::endl <<
      "  %m    log message" << std::endl <<
      "  %o    topics" << std::endl <<
      "  %l    log level" << std::endl <<
      std::endl;
}

static bool SetupFromCommandLine(
    int argc, char *argv[], RcConsole &rcconsole) {
  while (true) {
    int option = getopt_long(argc, argv, "", program_options, NULL);
    switch (option) {
      case -1:
        if (!rcconsole.HasOutputStream()) {
          rcconsole.SetOutputStream(kDefaultFormatString, std::cout);
        }
        return true;
      case RcConsole::INCLUDE_NAME_FILTER:
      case RcConsole::INCLUDE_MESSAGE_FILTER:
      case RcConsole::EXCLUDE_NAME_FILTER:
      case RcConsole::EXCLUDE_MESSAGE_FILTER:
        rcconsole.AddFilter(static_cast<RcConsole::LogFilterType>(option), optarg);
        break;
      case kFormatStringArgument:
        rcconsole.SetOutputStream(optarg, std::cout);
        break;
      case 'h':
        PrintUsage(argv[0]);
        return false;
      default:
        return false;
    }
  }
}

}  // namespace rcconsole

int main(int argc, char *argv[]) {
  ros::init(
      argc, argv, "rcconsole",
      ros::init_options::AnonymousName | ros::init_options::NoRosout);
  ros::NodeHandle node_handle("~");
  rcconsole::RcConsole rcconsole(node_handle);
  if (!SetupFromCommandLine(argc, argv, rcconsole)) {
    return -1;
  }
  ros::spin();
  return 0;
}
