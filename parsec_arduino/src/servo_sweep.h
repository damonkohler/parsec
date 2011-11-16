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

  /**
   * Attaches the servo to the configured pin.
   */
  void Attach();

  /**
   * Configure servo parameters.
   *
   * @param min_pwm_period the minimum PWM period supported by the servo
   * @param max_pwm_period the maximum PWM period supported by the servo
   * @param min_angle the angle assocated with the minimum PWM period
   * @param max_angle the angle associated with the maximum PWM period
   */
  void SetParameters(
      unsigned int min_pwm_period, unsigned int max_pwm_period,
      float min_angle, float max_angle);

  /**
   * min_angle and max_angle must be between -PI/2 and +PI/2.
   *
   * @param period the length of a complete period in seconds
   */
  void SetProfile(float min_angle, float max_angle, float period);

  /**
   * Update the servo's position in accordance with the configured profile.
   */
  void Update();

 private:
  // Set values in the enum here to match the signal in the
  // corresponding ROS message parsec_msgs/LaserTiltSignal.
  typedef enum {ANGLE_DECREASING=0, ANGLE_INCREASING=1} ServoDirection;

  int servo_pin_;
  unsigned long period_;  // period in microseconds
  unsigned int min_pwm_period_;  // minimal period in microseconds
  unsigned int max_pwm_period_;  // maximal period in microseconds
  unsigned int min_servo_pwm_period_;  // minimum servo PWM period in radians
  unsigned int max_servo_pwm_period_;  // maximum servo PWM period in radians
  float min_servo_angle_;  // minimum servo angle in radians
  float max_servo_angle_;  // maximum servo angle in radians
  Servo servo_;
  ServoDirection direction_;
  OnSignalCallback on_signal_;

  void SetDirection(ServoDirection new_direction);
};

#endif  // PARSECLIB_SERVO_SWEEP_
