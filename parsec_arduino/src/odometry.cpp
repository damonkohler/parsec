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

#include "odometry.h"

#include <math.h>

#include "ros.h"
#include "parsec_msgs/Odometry.h"

Odometry::Odometry()
  : phi_(0.0f), x_(0.0f), y_(0.0f), cumulated_phi_delta_(0.0f),
    cumulated_position_delta_(0.0f), velocity_measurement_interval_(0.0f) {}

void Odometry::UpdateFromWheels(
    float left, float right, float wheel_distance, float time_delta) {
  // Assuming constant velocity and $sin x \approx x$.
  float phi_delta = (right - left) / wheel_distance;
  static const float two_pi = 6.283185307f;
  phi_ += phi_delta;
  while (phi_ < 0.0f) {
    phi_ += two_pi;
  }
  while (phi_ >= two_pi) {
    phi_ -= two_pi;
  }
  cumulated_phi_delta_ += phi_delta;
  float position_delta = (left + right) / 2.0f;
  float x_delta = cos(phi_) * position_delta;
  float y_delta = sin(phi_) * position_delta;
  x_ += x_delta;
  y_ += y_delta;
  cumulated_position_delta_ += position_delta;
  velocity_measurement_interval_ += time_delta;
}

void Odometry::ToMessage(
    const ros::NodeHandle &node_handle, parsec_msgs::Odometry *message) {
  message->header.stamp = node_handle.now();
  static char frame_id[] = "odom";
  message->header.frame_id = frame_id;
  message->position_x = x_;
  message->position_y = y_;
  message->orientation_z = sin(0.5f * phi_);
  message->orientation_w = cos(0.5f * phi_);
  message->linear_x =
      cumulated_position_delta_ / velocity_measurement_interval_;
  message->linear_y = 0.0f;
  message->angular_z = cumulated_phi_delta_ / velocity_measurement_interval_;

  // Reset for new velocity measurement.
  cumulated_phi_delta_ = 0.0f;
  cumulated_position_delta_ = 0.0f;
  velocity_measurement_interval_ = 0.0f;
}
