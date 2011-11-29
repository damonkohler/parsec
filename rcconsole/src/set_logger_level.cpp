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

#include <getopt.h>

#include <algorithm>
#include <iostream>
#include <iterator>

#include <ros/ros.h>

static const option program_options[] = {
  {"repeat", required_argument, NULL, 'r'},
  {"list-nodes", no_argument, NULL, 'n'},
  {"list-loggers", required_argument, NULL, 'l'},
  {"help", no_argument, NULL, 'h'},
  {0, 0, 0, 0}  
};

void PrintUsage(const std::string &program_name) {
  std::cout << "Usage: " << program_name <<
      " [OPTION] [node] [logger] [level]" << std::endl;
  std::cout << std::endl << "Program options:" << std::endl <<
      "  -r, --repeat=duration" << std::endl <<
      "\t repeat setting the level every 'duration' seconds" << std::endl <<
      " -n, --list-nodes" << std::endl <<
      "\t lists all nodes for which logger levels can be set" << std::endl <<
      " -l, --list-loggers" << std::endl <<
      "\t lists all loggers of a node" << std::endl <<
      " -h, --help" << std::endl <<
      "\t print this message" <<std::endl;
}

void ListNodes() {
  std::list<std::string> nodes = rcconsole::Logger::ListNodes();
  std::copy(nodes.begin(), nodes.end(),
            std::ostream_iterator<std::string>(std::cout, "\n"));
}

void ListLoggers(const std::string &node_name) {
  std::list<std::string> nodes = rcconsole::Logger::ListLoggers(node_name);
  std::copy(nodes.begin(), nodes.end(),
            std::ostream_iterator<std::string>(std::cout, "\n"));  
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "set_logger_level", ros::init_options::AnonymousName);
  double repeat_duration = 0;
  std::string node_name;
  std::string logger_name;
  std::string level;

  while (true) {
    int option = getopt_long(argc, argv, "r:nl:h", program_options, NULL);
    if (option == -1) {
      if (argc - optind < 3) {
        std::cout << "Not enough arguments." << std::endl;
        PrintUsage(argv[0]);
        return 1;
      }
      node_name = argv[optind];
      logger_name = argv[optind+1];
      level = argv[optind+2];
      break;
    }
    switch (option) {
      case 'r':
        repeat_duration = atof(optarg);
        break;
      case 'n':
        ListNodes();
        return 0;
      case 'l':
        ListLoggers(optarg);
        return 0;
      case 'h':
        PrintUsage(argv[0]);
        return 0;
      default:
        PrintUsage(argv[0]);
        return 1;
    }
  }

  rcconsole::Logger logger(
      ros::NodeHandle("~"), node_name, ros::Duration(repeat_duration));
  logger.SetLoggerLevelName(logger_name, level);
  if (repeat_duration != 0) {
    ros::spin();
  }
  return 0;
}
