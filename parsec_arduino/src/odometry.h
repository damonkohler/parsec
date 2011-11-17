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
  Odometry();

  // Updates the odometry by a left and right wheel movement, separated by
  // a known distance. All distances in meters.
  void UpdateFromWheels(
      float left, float right, float wheel_distance, float time_delta);

  // Puts the current odometry state into a message.
  void ToMessage(
      const ros::NodeHandle &node_handle, parsec_msgs::Odometry *message);

 private:
  float phi_;
  float x_;
  float y_;

  float cumulated_phi_delta_;
  float cumulated_position_delta_;
  float velocity_measurement_interval_;
};

#endif  // PARSECLIB_ODOMETRY_
