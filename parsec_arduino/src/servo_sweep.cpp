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
  : servo_(), servo_pin_(servo_pin),
    period_(0),
    min_pwm_period_(min_servo_pwm_period_),
    max_pwm_period_(max_servo_pwm_period_),
    direction_(ANGLE_INCREASING),
    on_signal_(callback) {}

void ServoSweep::Attach() {
  servo_.attach(servo_pin_);
}

void ServoSweep::SetParameters(
    unsigned int min_pwm_period, unsigned int max_pwm_period,
    float min_angle, float max_angle) {
  min_servo_pwm_period_ = min_pwm_period;
  max_servo_pwm_period_ = max_pwm_period;
  min_servo_angle_ = min_angle;
  max_servo_angle_ = max_angle;
}

void ServoSweep::SetProfile(float min_angle, float max_angle, float period) {
  period_ = period * 1e6;
  unsigned int pwm_period_per_radian = (max_servo_pwm_period_ - min_servo_pwm_period_) /
      (max_servo_angle_ - min_servo_angle_);
  // CHECK(max_angle >= min_angle);
  // CHECK(min_angle >= min_servo_angle_);
  // CHECK(max_angle <= max_servo_angle_);
  min_pwm_period_ = (min_angle - min_servo_angle_) * pwm_period_per_radian + min_servo_pwm_period_;
  max_pwm_period_ = (max_angle - min_servo_angle_) * pwm_period_per_radian + min_servo_pwm_period_;

  if (min_pwm_period_ < min_servo_pwm_period_) {
    min_pwm_period_ = min_servo_pwm_period_;
  }
  if (max_pwm_period_ > max_servo_pwm_period_) {
    max_pwm_period_ = max_servo_pwm_period_;
  }
}

void ServoSweep::Update() {
  // Do nothing if tilting is disabled.
  if (period_ == 0) {
    return;
  }

  // Map the current time into the interval [0, period_).
  unsigned long long position = micros() % period_;

  if (position > period_ / 2) {
    position = period_ - position;
    SetDirection(ANGLE_DECREASING);
  } else {
    SetDirection(ANGLE_INCREASING);
  }

  // Map the variable position into the interval [min_pwm_period_, max_pwm_period_).
  unsigned int pwm_period = min_pwm_period_ + position * (max_pwm_period_ - min_pwm_period_) / (period_ / 2);

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
