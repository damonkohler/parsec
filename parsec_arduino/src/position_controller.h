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

#ifndef PARSECLIB_POSITION_CONTROLLER_
#define PARSECLIB_POSITION_CONTROLLER_

// Class for the Parallax Position Controller Device.
class PositionController {
 public:
  // Constructs a position controller object. Communication is done by
  // receiving and sending bytes via the given functions. Reads may return -1
  // on error, or an unsigned char.
  //
  // Since bytes are transmitted and received over the same wire, changing
  // back from transmitting to receiving is critical, both for ourselves and the
  // position controller we talk to, to make sure we do not miss any bytes.
  //
  // After reading, we wait 150 microseconds before writing to give the
  // position controller enough time to set RXEN, even when currently
  // handling its interrupts.
  //
  // Writing is not an issue. Assume that writing a byte takes about 50 us,
  // e.g., 19200 baud, and is done when the write function returns. This means
  // it takes about 150 microseconds to send three bytes. Therefore, if the
  // interrupts of the position controller are faster, we will not overflow the
  // receive buffer (2 bytes in the receive FIFO and one in the shift register).
  //
  // Setting RXEN after transmitting also has to happen on our side. We set the
  // transmit delay of the position controller to 300 microseconds, so all
  // interrupts running here have to be faster.
  PositionController(
      int (*read)(), void (*write)(unsigned char),
      unsigned char address, float wheel_radius);

  // Initializes a position controller, possibly reversing its orientation.
  // It will stay reversed until power is lost, even across soft resets.
  void Initialize(bool reverse);

  // Updates the velocity, given in m/s. Returns the travelled distance in m
  // since the last call to UpdateVelocity() for odometry. This is combined to
  // use a single position query for efficiency.
  float UpdateVelocity(float velocity);

  // Returns the last position reading.
  unsigned int GetLastPosition() { return last_position_; }

  // Returns the current target velocity.
  float GetTargetVelocity() { return target_velocity_; }

  // Tries to broadcast a software reset to all position controllers in the
  // hope they will stop the motors.
  static void SoftwareEmergencyStop(void (*write)(unsigned char));

  void SetGain(float gain);

  void SetAcceleration(float acceleration);

 private:
  static const unsigned int kMaximumSpeed;
  static const float kMaximumVelocity;
  static const float kMinimumVelocity;

  // Read successfully or crash.
  inline unsigned char ReadSafely();

  // Soft reset.
  inline void ClearPosition();

  // Sets the transmit delay between 40 and 1147 microseconds.
  inline void SetTXDelay(int usec);

  // Reverse the orientation for a controller. It will stay reversed until
  // power is lost, even across soft resets.
  inline void SetOrientationAsReversed();

  // Sets the speed ramp rate between 1 and 255.
  inline void SetSpeedRampRate(unsigned char rate);

  // Sets the maximum speed in positions per half-second, which is equal to
  // 10/3 rpm.
  inline void SetSpeedMaximum(unsigned int speed);

  // Adds positions to travel. 0 has the special meaning of stop.
  inline void TravelNumberOfPositions(int distance);

  // Queries the current position.
  inline unsigned int QueryPosition();

  // Queries the change in position since the last call.
  inline unsigned int QueryPositionDelta();

  // Limits acceleration toward the target velocity.
  void LimitAcceleration(float velocity, float time_delta);

  // Set the travel destination to a distance relative from the current
  // position. Returns the change in positions since the last call.
  inline int TravelFromHere(int distance_from_here);

  int (*read_)();
  void (*write_)(unsigned char);
  unsigned char address_;
  float wheel_radius_;
  unsigned int last_position_;
  int travel_goal_;
  unsigned long last_update_time_;
  float target_velocity_;
  float distance_error_;
  float gain_;
  float acceleration_;
};

#endif  // PARSECLIB_POSITION_CONTROLLER_
