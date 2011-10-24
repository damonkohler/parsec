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

ServoSweep::ServoSweep(int servo_pin)
  : servo_(), servo_pin_(servo_pin) {}

void ServoSweep::Init() {
  servo_.attach(servo_pin_);
}

void ServoSweep::Update() {
  // About 2 seconds period.
  long position = micros() & ((1L << 21) - 1);
  if (position >= (1L << 20)) {
   position = (1L << 21) - position;
  }
  // We use writeMicroseconds for increased precision.
  // For the HSR-5990TG servo, 1500 us is neutral, and
  // +/- 900 us for +/- 90 degrees.
  servo_.writeMicroseconds(600 + ((position * 1800) >> 20));
}
