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

#include "parallax_ping.h"

#include <WProgram.h>

void SendLogMessage(const char *message);  // TODO(whess): Move elsewhere.

Ultrasonic* Ultrasonic::measuring_ = NULL;
volatile char Ultrasonic::state_ = Ultrasonic::kStateReady;
unsigned long Ultrasonic::triggered_micros_ = 0;
unsigned long Ultrasonic::receiving_micros_ = 0;
unsigned long Ultrasonic::done_micros_ = 0;
int Ultrasonic::debug_id_ = -1;
int Ultrasonic::error_count_ = 0;

Ultrasonic::Ultrasonic()
    : last_micros_(0), value_(32767) {}

bool Ultrasonic::IsReady() {
  if (state_ != kStateReady) {
    // Still measuring, so we check for timeout.
    LogIfCheck(micros() - triggered_micros_ < 30000,
               "Ping%d timeout", debug_id_);
    return false;
  }
  if (measuring_ != NULL) {
    // A measurement completed, so compute and store a new value.
    measuring_->UpdateValue();
    measuring_ = NULL;
  }
  return micros() - last_micros_ > 200;
}

void Ultrasonic::SendTriggerPulse(int debug_id) {
  LogIfCheck(measuring_ == NULL, "Ping%d measuring", debug_id_);
  LogIfCheck(state_ == kStateReady, "Ping%d busy", debug_id_);
  LogIfCheck(micros() - done_micros_ >= 200, "Ping%d+1 lowdelta", debug_id_);
  debug_id_ = debug_id;
  measuring_ = this;
  digitalWrite(kPulsePin, HIGH);
  pinMode(kPulsePin, OUTPUT);
  delayMicroseconds(5);
  pinMode(kPulsePin, INPUT);
  digitalWrite(kPulsePin, LOW);
  triggered_micros_ = micros();
  state_ = kStateTriggered;
  attachInterrupt(kPulsePinInterrupt, &Ultrasonic::HandleInputChange, CHANGE);
}

int Ultrasonic::QueryValue() {
  return value_;
}

float Ultrasonic::QueryDistance() {
  return 1.72e-4f * QueryValue();
}

int Ultrasonic::DebugTime() {
  return last_micros_ / 1000000;
}

int Ultrasonic::GetErrorCount() {
  return error_count_;
}

void Ultrasonic::HandleInputChange() {
  unsigned long current_micros = micros();
  if (digitalRead(kPulsePin) == HIGH) {
    // State is kStateTriggered.
    receiving_micros_ = current_micros;
    state_ = kStateReceiving;
  } else if (state_ == kStateReceiving) {
    detachInterrupt(kPulsePinInterrupt);
    done_micros_ = current_micros;
    state_ = kStateReady;
  }
}

void Ultrasonic::UpdateValue() {
  unsigned long delta_micros = receiving_micros_ - triggered_micros_;
  // Answer is expected after 750 us.
  LogIfCheck(delta_micros > 650 && delta_micros < 850,
             "Ping%d at %lu", debug_id_, delta_micros);
  value_ = done_micros_ - receiving_micros_;
  // Answer should be between 115 us and 18500 us.
  LogIfCheck(value_ > 100 && value_ < 25000,
             "Ping%d is %lu", debug_id_, value_);
  last_micros_ = done_micros_;
}

void Ultrasonic::LogIfCheck(bool assertion, const char *format, ...) {
  if (!assertion) {
    ++error_count_;
    static char error_message[18];
    va_list ap;
    va_start(ap, format);
    vsnprintf(error_message, 18, format, ap);
    va_end(ap);
    SendLogMessage(error_message);
  }
}
