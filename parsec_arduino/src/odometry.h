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
  Odometry()
    : phi(0.0f), x(0.0f), y(0.0f), x_dot_t(0.0f),
      velocity_measurement_interval(0.0f) {}

  // Updates the odometry by a left and right wheel movement, separated by
  // a known distance. All distances in meters.
  void UpdateFromWheels(
      float left, float right, float wheel_distance, float time_delta) {
    // Assuming constant velocity and $sin x \approx x$.
    float phi_delta = (right - left) / wheel_distance;
    static const float two_pi = 6.283185307f;
    phi += phi_delta;
    while (phi < 0.0f) {
      phi += two_pi;
    }
    while (phi >= two_pi) {
      phi -= two_pi;
    }
    phi_dot_t += phi_delta;
    float position_delta = (left + right) / 2.0f;
    float x_delta = cos(phi) * position_delta;
    float y_delta = sin(phi) * position_delta;
    x += x_delta;
    y += y_delta;
    x_dot_t += position_delta;
    velocity_measurement_interval += time_delta;
  }

  // Puts the current odometry state into a message.
  void ToMessage(const ros::NodeHandle &node_handle, parsec_msgs::Odometry *message) {
    message->header.stamp = node_handle.now();
    static char frame_id[] = "odom";
    message->header.frame_id = frame_id;
    message->position_x = x;
    message->position_y = y;
    message->orientation_z = sin(0.5f * phi);
    message->orientation_w = cos(0.5f * phi);
    message->linear_x = x_dot_t / velocity_measurement_interval;
    message->linear_y = 0.0f;
    message->angular_z = phi_dot_t / velocity_measurement_interval;

    // reset for new velocity measurement
    x_dot_t = phi_dot_t = velocity_measurement_interval = 0.0f;
  }

 private:
  float phi;
  float x;
  float y;
  
  float phi_dot_t;
  float x_dot_t;
  float velocity_measurement_interval;
};

#endif  // PARSECLIB_ODOMETRY_

