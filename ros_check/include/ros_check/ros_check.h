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

#include <stdio.h>

#include <ros/console.h>

#define CHECK_OP(lhs, rhs, op) CHECK(lhs op rhs)

#define CHECK(condition) \
    do {\
      if (!(condition)) { \
        ROS_FATAL("Check failed: " #condition); \
        ros_check::PrintStacktraceAndDie(stderr); \
      } \
    } while (0)
#define CHECK_EQ(lhs, rhs) CHECK_OP(lhs, rhs, ==)
#define CHECK_NE(lhs, rhs) CHECK_OP(lhs, rhs, !=)
#define CHECK_LE(lhs, rhs) CHECK_OP(lhs, rhs, <=)
#define CHECK_LT(lhs, rhs) CHECK_OP(lhs, rhs, <)
#define CHECK_GE(lhs, rhs) CHECK_OP(lhs, rhs, >=)
#define CHECK_GT(lhs, rhs) CHECK_OP(lhs, rhs, >)

namespace ros_check {

void PrintStacktrace(FILE *stream, int skip);
void PrintStacktraceAndDie(FILE *stream);

}

#endif  // ROS_CHECK_H
