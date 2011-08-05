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

#include <parallax_ping.h>

#include <WProgram.h>

void Check(bool assertion, const char *format, ...);  // TODO(whess): Move this.

volatile Ultrasonic* Ultrasonic::measuring_ = NULL;
bool Ultrasonic::last_level_ = false;

Ultrasonic::Ultrasonic()
    : last_micros_(0), value_(32767) {}

bool Ultrasonic::IsReady() {
  noInterrupts();
  bool ready = (measuring_ == NULL &&
                micros() - last_micros_ > 200);
  interrupts();
  return ready;
}

void Ultrasonic::SendTriggerPulse() {
  delayMicroseconds(5);
  Check(measuring_ == NULL, "US: meas != NULL");
  Check(micros() - last_micros_ >= 200, "US: delta < 200");
  measuring_ = this;
  digitalWrite(kPulsePin, HIGH);
  pinMode(kPulsePin, OUTPUT);
  delayMicroseconds(5);
  pinMode(kPulsePin, INPUT);
  digitalWrite(kPulsePin, LOW);
  last_micros_ = micros();
  attachInterrupt(kPulsePinInterrupt, &Ultrasonic::HandleInputChange, CHANGE);
}

int Ultrasonic::QueryValue() {
  noInterrupts();
  int value = value_;
  interrupts();
  return value;
}

float Ultrasonic::QueryDistance() {
  return 1.72e-4f * QueryValue();
}

int Ultrasonic::DebugTime() {
  noInterrupts();
  int value = last_micros_ / 1000000;
  interrupts();
  return value;
}

void Ultrasonic::HandleInputChange() {
  Check(measuring_ != NULL, "US: meas == NULL");
  unsigned long delta_micros = micros() - measuring_->last_micros_;
  measuring_->last_micros_ += delta_micros;
  if (digitalRead(kPulsePin) == HIGH) {
    // The sensors answer is expected after 750 us.
    Check(delta_micros > 700 && delta_micros < 800,
          "US: at %lu", delta_micros);
    last_level_ = true;
  } else if (last_level_) {
    // Answer should be between 115 us and 18500 us.
    Check(delta_micros > 100 && delta_micros < 25000,
          "US: is %lu", delta_micros);
    measuring_->value_ = delta_micros;
    measuring_ = NULL;
    detachInterrupt(kPulsePinInterrupt);
    last_level_ = false;
  }
}
