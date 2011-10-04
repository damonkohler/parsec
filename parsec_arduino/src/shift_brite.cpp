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

#include "shift_brite.h"

#include <SPI.h>
#include <WProgram.h>

ShiftBrite::ShiftBrite(int latch_pin)
    : latch_pin_(latch_pin) {
  pinMode(latch_pin_, OUTPUT);
  digitalWrite(latch_pin_, LOW);
}

void ShiftBrite::Initialize(int n) {
  for (int i = 0; i != n; ++i) {
    // All dot corrections to full current. Clock mode to 800 kHz.
    unsigned long current = 127;  // From 0 (36.5 %) to 127 (100 %).
    SendData(0x40000000ul | current << 20 | current << 10 | current);
  }
  Latch(n);
}

void ShiftBrite::Enable(int enable_pin) {
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
}

void ShiftBrite::UpdateColors(int n, int red[], int green[], int blue[]) {
  for (int i = 0; i != n; ++i) {
    UpdateColor(red[i], green[i], blue[i]);
  }
  Latch(n);
}

void ShiftBrite::UpdateColor(int red, int green, int blue) {
  SendData(
      static_cast<unsigned long>(blue) << 20 |
      static_cast<unsigned long>(red) << 10 |
      static_cast<unsigned long>(green));
}

void ShiftBrite::SendData(unsigned long data) {
  SPI.transfer(data >> 24);
  SPI.transfer((data >> 16) & 0xff);
  SPI.transfer((data >> 8) & 0xff);
  SPI.transfer(data & 0xff);
}

void ShiftBrite::Latch(int n) {
  delayMicroseconds(20 + 5 * n);
  digitalWrite(latch_pin_, HIGH);
  delayMicroseconds(20);
  digitalWrite(latch_pin_, LOW);
}
