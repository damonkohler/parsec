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

#include "position_controller.h"

#include <WProgram.h>

void Check(bool assertion, const char *format, ...);  // TODO(whess): Move this.

PositionController::PositionController(
    int (*read)(), void (*write)(unsigned char),
    unsigned char address, float wheel_radius, Pid *pid)
    : pid_(pid),
      read_(read), write_(write), address_(address),
      wheel_radius_(wheel_radius), last_position_(0), travel_goal_(0),
      last_position_time_(0), last_velocity_position_(0), last_velocity_(0.0f),
      last_velocity_cmd_(0.0f), last_control_time_(0.0f) {}

void PositionController::Initialize(bool reverse) {
  // Three CLRPs are kind of a soft reset to ensure all position
  // controllers are in a sane state.
  ClearPosition();
  ClearPosition();
  ClearPosition();
  // We choose this long delay, so that we have time to set RXEN, even
  // when we are interrupted immediately after sending a query command.
  SetTXDelay(300);
  if (reverse) {
    SetOrientationAsReversed();
  }
  // We want to control the speed, so we set the speed ramp to the maximum
  // possible.
  SetSpeedRampRate(255);
}

float PositionController::UpdateVelocity(float velocity) {
  int delta;
  unsigned long current_time = micros();
  last_velocity_cmd_ += pid_->update(last_velocity_, velocity, (current_time - last_control_time_) * 1e-6);
  float speed = last_velocity_cmd_ * 2.864788975f /* $9/\pi$ */ / wheel_radius_;
  if (speed < 0) {
    delta = TravelFromHere(-100);
    speed = -speed;
  } else if (speed > 0) {
    delta = TravelFromHere(100);
  } else if (speed == 0) {
    delta = TravelFromHere(0);
  }
  RecalculateVelocity();
  last_control_time_ = current_time;
  SetSpeedMaximum(speed < kMaximumSpeed ? floor(speed + .5f) : kMaximumSpeed);
  return 1.745329252e-1f /* \pi/18 */ * wheel_radius_ * delta;
}

void PositionController::SoftwareEmergencyStop(void (*write)(unsigned char)) {
  const unsigned char CLRP = 0x28;
  // We do not wait before the first CLRP broadcast to stop as soon as possible.
  // We wait for the following 3 CLRPs to make sure they are received.
  for (int i = 0; i != 4; ++i) {
    write(CLRP);
    delayMicroseconds(150);
  }
}

unsigned char PositionController::ReadSafely() {
  int result = read_();
  Check(result != -1, "Posctrl %d failed", int(address_));
  unsigned char data = result;
  return data;
}

void PositionController::ClearPosition() {
  const unsigned char CLRP = 0x28;
  write_(CLRP | address_);
  last_position_ = 0;
  travel_goal_ = 0;
  pid_->reset();
}

void PositionController::SetTXDelay(int usec) {
  const unsigned char STXD = 0x38;
  write_(STXD | address_);
  write_(int((usec - 40) * 100l / 434));
}

void PositionController::SetOrientationAsReversed() {
  const unsigned char SREV = 0x30;
  write_(SREV | address_);
}

void PositionController::SetSpeedRampRate(unsigned char rate) {
  const unsigned char SSRR = 0x48;
  write_(SSRR | address_);
  write_(rate);
}

void PositionController::SetSpeedMaximum(unsigned int speed) {
  const unsigned char SMAX = 0x40;
  write_(SMAX | address_);
  write_(speed >> 8);
  write_(speed & 0xff);
}

void PositionController::TravelNumberOfPositions(int distance) {
  const unsigned char TRVL = 0x20;
  write_(TRVL | address_);
  write_((distance >> 8) & 0xff);
  write_(distance & 0xff);
}

unsigned int PositionController::QueryPosition() {
  const unsigned char QPOS = 0x08;
  write_(QPOS | address_);
  unsigned char msb = ReadSafely();
  unsigned char lsb = ReadSafely();
  // We wait 150 microseconds for the position controller to set RXEN, so that
  // it can receive the next command.
  delayMicroseconds(150);
  return msb << 8 | lsb;
}

unsigned int PositionController::QueryPositionDelta() {
  unsigned int delta = QueryPosition() - last_position_;
  last_position_ += delta;
  return delta;
}

void PositionController::RecalculateVelocity() {
  unsigned long current_time = micros();
  long time_delta = current_time - last_position_time_;
  // Only recalculate when enough time since the last measurement has
  // passed.
  if (time_delta > 20000) {
    last_velocity_ = 1.745329252e-1f /* \pi/18 */ * wheel_radius_
      * int(last_position_ - last_velocity_position_)
      / (time_delta * 1e-6);
    last_position_time_ = current_time;
    last_velocity_position_ = last_position_;
  }
}

int PositionController::TravelFromHere(int distance_from_here) {
  int delta = QueryPositionDelta();
  travel_goal_ -= delta;
  int distance_change = distance_from_here - travel_goal_;
  if (distance_change != 0) {
    TravelNumberOfPositions(distance_change);
    travel_goal_ += distance_change;
  }
  return delta;
}
