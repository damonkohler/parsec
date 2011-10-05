// Copyright 2011 Google Inc.
// Author: whess@google.com (Wolfgang Hess)
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

#ifndef PARSECLIB_ODOMETRY_
#define PARSECLIB_ODOMETRY_

#include <math.h>

#include "ros.h"
#include "parsec_msgs/Odometry.h"

class Odometry {
 public:
  Odometry() : phi(0.0f), x(0.0f), y(0.0f) {}

  // Updates the odometry by a left and right wheel movement, separated by
  // a known distance. All distances in meters.
  void UpdateFromWheels(
      float left, float right, float wheel_distance, float time_delta) {
    // Assuming constant velocity and $sin x \approx x$.
    float phi_delta = (right - left) / wheel_distance;
    phi += phi_delta;
    phi_dot = phi_delta / time_delta;
    float position_delta = (left + right) / 2.0f;
    float x_delta = cos(phi) * position_delta;
    float y_delta = sin(phi) * position_delta;
    x += x_delta;
    y += y_delta;
    x_dot = x_delta / time_delta;
    y_dot = y_delta / time_delta;
  }

  // Puts the current odometry state into a message.
  void ToMessage(const ros::NodeHandle& node_handle, parsec_msgs::Odometry *message) {
    message->header.stamp = const_cast<ros::NodeHandle&>(node_handle).now();
    message->header.frame_id = "odom";
    message->pose.position.x = x;
    message->pose.position.y = y;
    message->pose.position.z = 0.0f;
    message->pose.orientation.x = 0.0f;
    message->pose.orientation.y = sin(0.5f * phi);
    message->pose.orientation.z = 0.0f;
    message->pose.orientation.w = cos(0.5f * phi);
    message->child_frame_id = "base_link";
    message->twist.linear.x = x_dot;
    message->twist.linear.y = y_dot;
    message->twist.angular.z = phi_dot;
  }

 private:
  float phi;
  float x;
  float y;
  float phi_dot;
  float x_dot;
  float y_dot;
};

#endif  // PARSECLIB_ODOMETRY_

