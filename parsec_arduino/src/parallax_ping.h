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

#ifndef PARSECLIB_PARALLAX_PING_
#define PARSECLIB_PARALLAX_PING_

#include "median_queue.h"

// Class for Parallax Inc. PING))) devices.
class Ultrasonic {
 public:
  static const int kPulsePin = 2;  // Pulses are sent through pin 2.
  static const int kPulsePinInterrupt = 0;  // The interrupt for kInputPin.
  static const int kKeepValues = 5;  // Median computed of last few values.

  // Creates an ultrasonic object. All sensors use the same pin for their
  // pulses and are expected to be multiplexed.
  Ultrasonic();

  // Whether this sensor is ready to be triggered.
  bool IsReady();

  // Sends a 5 us trigger pulse. You must connect this sensor to the pulse pin
  // before calling this function, and maintain the connection until IsReady()
  // returns true again, e.g., only change connections when triggering pulses.
  // The new data is returned after IsReady() returned true.
  void SendTriggerPulse(int debug_id);

  // Queries the current value in us. The current value is the median of the
  // last kKeepValues readings.
  int QueryValue();

  // Queries the current value in meters, i.e., 1.72e-4 * QueryValue().
  float QueryDistance();

  // Returns the number of errors that we have seen so far.
  static int GetErrorCount();

 private:
  static const char kStateReady = 0;
  static const char kStateTriggered = 1;
  static const char kStateReceiving = 2;

  static void HandleInputChange();
  inline void UpdateValue();
  static void LogIfCheck(bool assertion, const char *format, ...);

  unsigned long last_micros_;
  MedianQueue<int, kKeepValues> values_;

  static Ultrasonic *measuring_;
  static volatile char state_;
  static unsigned long triggered_micros_;
  static unsigned long receiving_micros_;
  static unsigned long done_micros_;

  static int debug_id_;
  static int error_count_;
};

#endif  // PARSECLIB_PARALLAX_PING_
