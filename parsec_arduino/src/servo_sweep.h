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

#ifndef PARSECLIB_SERVO_SWEEP_
#define PARSECLIB_SERVO_SWEEP_

#include <Servo.h>
#include <WProgram.h>

#include "ros.h"

class ServoSweep {
 public:
  typedef void(*OnSignalCallback)(int);
  
  ServoSweep(int servo_pin, OnSignalCallback callback=NULL);

  void Init();

  /**
   * min_angle and max_angle must be between -PI/2 and +PI/2.
   *
   * \param period the length of a complete period in seconds
   */
  void SetProfile(float min_angle, float max_angle, float period);
  void Update();

 private:
  // Set values in the enum here to match the signal in the
  // corresponding ROS message parsec_msgs/LaserTiltSignal.
  typedef enum {DIRECTION_DOWN=0, DIRECTION_UP=1} ServoDirection;
  // For the HSR-5990TG servo, 1500 us is neutral, and
  // +/- 900 us for +/- 90 degrees.
  static const unsigned SERVO_MIN_PWM_PERIOD = 600;
  static const unsigned SERVO_MAX_PWM_PERIOD = 2400;
  
  int servo_pin_;
  unsigned long period_;      // period in microseconds
  unsigned min_pwm_period_;   // minimal period in microseconds
  unsigned max_pwm_period_;   // maximal period in microseconds
  Servo servo_;
  ServoDirection direction_;  // 0 for going up, 1 for going down
  OnSignalCallback on_signal_;
};

#endif  // PARSECLIB_SERVO_SWEEP_
