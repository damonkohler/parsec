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

#ifndef ROS_CHECK_H
#define ROS_CHECK_H

#include <cstdio>

#include <boost/typeof/typeof.hpp>
#include <ros/console.h>
#include <ros/ros.h>

// We cannot create a function or class to prevent evaluation of
// rhs and lhs because we would lose line and file information in
// ROS_FATAL. Instead, we will bind lhs and rhs to variables
// inside the while loop.
#define CHECK_OP(name, value1, value2, operation) \
    do { \
      BOOST_AUTO(__ros_check_lhs_evaluated, value1); \
      BOOST_AUTO(__ros_check_rhs_evaluated, value2); \
      if (!(__ros_check_lhs_evaluated operation \
            __ros_check_rhs_evaluated)) { \
        ROS_FATAL_STREAM(__FILE__ ":" << __LINE__ << \
                         " CHECK" #name " failed: " \
                         #value1 " " #operation " " #value2 << \
                         " (" << __ros_check_lhs_evaluated << " vs. " << \
                         __ros_check_rhs_evaluated << ")"); \
        if (ros::isInitialized()) { \
          ros::shutdown(); \
        } \
        ros_check::PrintStacktraceAndDie(stderr); \
      } \
    } while (0)

#define CHECK(condition) \
    do { \
      if (!(condition)) { \
        ROS_FATAL(__FILE__ ":%d Check " #condition " failed", __LINE__); \
        if (ros::isInitialized()) { \
          ros::shutdown(); \
        } \
        ros_check::PrintStacktraceAndDie(stderr); \
      } \
    } while (0)

#define CHECK_EQ(lhs, rhs) CHECK_OP(_EQ, lhs, rhs, ==)
#define CHECK_NE(lhs, rhs) CHECK_OP(_NE, lhs, rhs, !=)
#define CHECK_LE(lhs, rhs) CHECK_OP(_LE, lhs, rhs, <=)
#define CHECK_LT(lhs, rhs) CHECK_OP(_LT, lhs, rhs, <)
#define CHECK_GE(lhs, rhs) CHECK_OP(_GE, lhs, rhs, >=)
#define CHECK_GT(lhs, rhs) CHECK_OP(_GT, lhs, rhs, >)

namespace ros_check {

void PrintStacktrace(FILE *stream, int skip);
void PrintStacktraceAndDie(FILE *stream);

}  // namespace ros_check

#endif  // ROS_CHECK_H
