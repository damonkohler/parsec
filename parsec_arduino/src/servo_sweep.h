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

class ServoSweep {
 public:
  ServoSweep(int servo_pin);

  void Update();

 private:
  Servo servo_;
};

#endif  // PARSECLIB_SERVO_SWEEP_
