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

#ifndef PARSECLIB_SIMPLE_LED_
#define PARSECLIB_SIMPLE_LED_

// Class to connect to an LED using a digital pin.
class SimpleLED {
 public:
  // Constructs an SimpleLED object and sets the pin appropriately. 
  explicit SimpleLED(int led_pin);

  // Shows a brightness of value 0--100 for a number of milliseconds.
  // After that time, the LED is turned off, and the function returns.
  void DoSoftwarePWM(int value, int milliseconds);

  // This function never returns and is intended as a debugging tool. It loops,
  // in each cycle first flashing the LED slowly, and then the id number.
  void EnterCrashLoop(int id);

 private:
  int led_pin_;
};

#endif  // PARSECLIB_SIMPLE_LED_
