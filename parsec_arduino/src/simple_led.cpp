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

#include "simple_led.h"

#include <WProgram.h>

SimpleLED::SimpleLED(int led_pin) : led_pin_(led_pin) {
  pinMode(led_pin_, OUTPUT);
}

void SimpleLED::DoSoftwarePWM(int value, int milliseconds) {
  if (value < 0) {
    value = 0;
  }
  if (value > 100) {
    value = 100;
  }
  for (int i = 0; i < 2 * milliseconds; ++i) {
    if (value > 0) {
      digitalWrite(led_pin_, HIGH);
      delayMicroseconds(5 * value);
    }
    if (value < 100) {
      digitalWrite(led_pin_, LOW);
      delayMicroseconds(5 * (100 - value));
    }
  }
  digitalWrite(led_pin_, LOW);
}

void SimpleLED::EnterCrashLoop(int id) {
  for (;;) {
    for (int value = 1; value != 101; ++value) {
      DoSoftwarePWM(value, 5);
    }
    for (int value = 99; value != 0; --value) {
      DoSoftwarePWM(value, 5);
    }
    DoSoftwarePWM(0, 300);
    for (int i = 0; i < id; ++i) {
      DoSoftwarePWM(100, 300);
      DoSoftwarePWM(0, 300);
    }
  }
}
