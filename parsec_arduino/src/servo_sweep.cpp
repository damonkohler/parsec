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

#include "servo_sweep.h"

#include <math.h>

ServoSweep::ServoSweep(int servo_pin, OnSignalCallback callback)
  : servo_pin_(servo_pin),
    increasing_duration_(0),
    decreasing_duration_(0),
    min_pwm_period_(0),
    max_pwm_period_(0),
    min_servo_pwm_period_(0),
    max_servo_pwm_period_(0),
    min_servo_angle_(0.0f),
    max_servo_angle_(0.0f),
    pwm_period_per_radian_(0),
    direction_(ANGLE_INCREASING),
    on_signal_(callback) {}

void ServoSweep::Attach() {
  servo_.attach(servo_pin_);
}

void ServoSweep::SetParameters(
    unsigned int min_pwm_period, unsigned int max_pwm_period,
    float min_angle, float max_angle) {
  // CHECK(min_angle >= 0);
  // CHECK(max_angle >= min_angle);
  min_servo_pwm_period_ = min_pwm_period;
  max_servo_pwm_period_ = max_pwm_period;
  min_servo_angle_ = min_angle;
  max_servo_angle_ = max_angle;
  pwm_period_per_radian_ = (max_servo_pwm_period_ - min_servo_pwm_period_) /
      (max_servo_angle_ - min_servo_angle_);
  // Move to the perpendicular position for a visual sanity check of the
  // parameters.
  SetPosition(0.0f);
}

void ServoSweep::SetPosition(float angle) {
  unsigned int pwm_period =
      min_servo_pwm_period_ - min_servo_angle_ * pwm_period_per_radian_;
  servo_.writeMicroseconds(pwm_period);
}

void ServoSweep::SetProfile(float min_angle, float max_angle,
                            float increasing_duration, float decreasing_duration) {
  increasing_duration_ = increasing_duration * 1e6;
  decreasing_duration_ = decreasing_duration * 1e6;
  // CHECK(max_angle >= min_angle);
  // CHECK(min_angle >= min_servo_angle_);
  // CHECK(max_angle <= max_servo_angle_);
  min_pwm_period_ = (min_angle - min_servo_angle_) * pwm_period_per_radian_
      + min_servo_pwm_period_;
  max_pwm_period_ = (max_angle - min_servo_angle_) * pwm_period_per_radian_
      + min_servo_pwm_period_;

  if (min_pwm_period_ < min_servo_pwm_period_) {
    min_pwm_period_ = min_servo_pwm_period_;
  }
  if (max_pwm_period_ > max_servo_pwm_period_) {
    max_pwm_period_ = max_servo_pwm_period_;
  }
}

void ServoSweep::Update() {
  // Do nothing if tilting is disabled.
  if (increasing_duration_ == 0 || decreasing_duration_ == 0) {
    return;
  }

  // Map the current time into the interval [0, period).
  unsigned long long position =
      micros() % (increasing_duration_ + decreasing_duration_);

  if (position > increasing_duration_) {
    position = increasing_duration_ + decreasing_duration_ - position;
    SetDirection(ANGLE_DECREASING);
  } else {
    SetDirection(ANGLE_INCREASING);
  }

  unsigned long current_duration;
  if (direction_ == ANGLE_INCREASING) {
    current_duration = increasing_duration_;
  } else {
    current_duration = decreasing_duration_;
  }
  // Map the variable position into the interval [min_pwm_period_, max_pwm_period_).
  unsigned int pwm_range = max_pwm_period_ - min_pwm_period_;
  unsigned int pwm_period =
      min_pwm_period_ + position * pwm_range / current_duration;
  // We use writeMicroseconds for increased precision.
  servo_.writeMicroseconds(pwm_period);
}

void ServoSweep::SetDirection(ServoDirection new_direction) {
  if (new_direction != direction_) {
    direction_ = new_direction;
    if (on_signal_) {
      on_signal_(direction_);
    }
  }
}
