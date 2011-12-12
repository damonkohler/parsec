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

bool ServoSweep::attached_ = false;
const int ServoSweep::kPrescaler = 8;

ServoSweep::ServoSweep(OnSignalCallback callback)
  : increasing_duration_(0),
    decreasing_duration_(0),
    min_pwm_period_(0),
    max_pwm_period_(0),
    min_servo_pwm_period_(0),
    max_servo_pwm_period_(0),
    pwm_period_per_radian_(0),
    min_servo_angle_(0.0f),
    max_servo_angle_(0.0f),
    increasing_phase_offset_(0),
    decreasing_phase_offset_(0),
    direction_(ANGLE_INCREASING),
    on_signal_(callback) {}

void ServoSweep::Attach() {
  // CHECK(!attached_);
  attached_ = true;

  // We use PWM pin 11, a.k.a. PB5, timer 1.
  pinMode(11, OUTPUT);
  // Prescaler 8 means the 16-bit timer can run for at most 32.768 ms before
  // overflowing at 16 MHz.
  unsigned long long frequency = F_CPU;
  // Update the servo every 10 ms.
  unsigned int max_timer_value = frequency * 10 / 1000 / kPrescaler - 1;
  ICR1 = max_timer_value;
  TCNT1 = 0;
  // Initialize timer 1 Fast PWM with ICR1 as TOP (mode 14 = WGM13|WGM12|WGM11).
  TCCR1B = _BV(CS11) | _BV(WGM13) | _BV(WGM12);  // kPrescaler == 8.
  TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Non-inverting mode.
}

void ServoSweep::UpdateMicroseconds(unsigned int pulse_width) {
  unsigned long long frequency = F_CPU;
  unsigned int time_value = frequency * pulse_width / 1000000ul / kPrescaler;
  OCR1A = time_value;
}

void ServoSweep::SetParameters(
    unsigned int min_pwm_period, unsigned int max_pwm_period,
    float min_angle, float max_angle,
    int increasing_phase_offset, int decreasing_phase_offset) {
  // CHECK(min_angle >= 0);
  // CHECK(max_angle >= min_angle);
  min_servo_pwm_period_ = min_pwm_period;
  max_servo_pwm_period_ = max_pwm_period;
  min_servo_angle_ = min_angle;
  max_servo_angle_ = max_angle;
  increasing_phase_offset_ = increasing_phase_offset;
  decreasing_phase_offset_ = decreasing_phase_offset;
  pwm_period_per_radian_ = (max_servo_pwm_period_ - min_servo_pwm_period_) /
      (max_servo_angle_ - min_servo_angle_);
  // Move to the perpendicular position for a visual sanity check of the
  // parameters.
  unsigned int pwm_period =
      min_servo_pwm_period_ - min_servo_angle_ * pwm_period_per_radian_;
  UpdateMicroseconds(pwm_period);
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

float ServoSweep::Update() {
  // Do nothing if tilting is disabled.
  if (increasing_duration_ == 0 || decreasing_duration_ == 0) {
    return 0.0f;
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

  UpdateMicroseconds(pwm_period);
  return static_cast<float>(pwm_period - min_servo_pwm_period_) / pwm_period_per_radian_ + min_servo_angle_;
}

int ServoSweep::GetPhaseOffset() {
  if (direction_ == ANGLE_INCREASING) {
    return increasing_phase_offset_;
  }
  return decreasing_phase_offset_;
}

void ServoSweep::SetDirection(ServoDirection new_direction) {
  if (new_direction != direction_) {
    direction_ = new_direction;
    if (on_signal_) {
      on_signal_(direction_);
    }
  }
}
