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

#include "ros.h"

#include "servo_sweep.h"

extern ros::NodeHandle node_handle;

ServoSweep::ServoSweep(int servo_pin)
  : servo_(), servo_pin_(servo_pin),
    period_(0),
    min_pwm_period_(SERVO_MIN_PWM_PERIOD),
    max_pwm_period_(SERVO_MAX_PWM_PERIOD) {}

void ServoSweep::Init() {
  servo_.attach(servo_pin_);
}

void ServoSweep::SetProfile(float min_angle, float max_angle, float period) {
  period_ = period * 1e6;

  min_pwm_period_ = (SERVO_MIN_PWM_PERIOD + SERVO_MAX_PWM_PERIOD) / 2 +
    min_angle/1.5707963268 * (SERVO_MAX_PWM_PERIOD - SERVO_MIN_PWM_PERIOD)/2;

  max_pwm_period_ = (SERVO_MIN_PWM_PERIOD + SERVO_MAX_PWM_PERIOD) / 2 +
    max_angle/1.5707963268 * (SERVO_MAX_PWM_PERIOD - SERVO_MIN_PWM_PERIOD)/2;
  
  // If min and max are swaped, re-swap
  if(max_pwm_period_ < min_pwm_period_)
  {
    unsigned tmp = max_pwm_period_;
    max_pwm_period_ = min_pwm_period_;
    min_pwm_period_ = tmp;
  }

  if(min_pwm_period_ < SERVO_MIN_PWM_PERIOD)
    min_pwm_period_ = SERVO_MIN_PWM_PERIOD;
  if(max_pwm_period_ > SERVO_MAX_PWM_PERIOD)
    max_pwm_period_ = SERVO_MAX_PWM_PERIOD;
}

void ServoSweep::Update() {
  if(period_ == 0)
    return;

  // Map the current time into the interval [-period_/2, period/2)
  unsigned long position = micros() % period_;

  if(position > period_/2)
    position = period_ - position;
  // Map the variable position into the interval [min_pwm_period_, max_pwm_period_)
  position = min_pwm_period_ + position * (max_pwm_period_ - min_pwm_period_) / (period_/2);
  
  // We use writeMicroseconds for increased precision.
  servo_.writeMicroseconds(position);
}
