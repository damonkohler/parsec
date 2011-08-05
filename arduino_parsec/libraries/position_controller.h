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
  // receiving and sending bytes via the given functions.
  PositionController(
      unsigned char (*read)(), void (*write)(unsigned char),
      unsigned char address, float wheel_radius);

  // Initializes a position controller, possibly reversing its orientation.
  // It will stay reversed until power is lost, even across soft resets.
  void Initialize(bool reverse);

  // Updates the velocity, given in m/s.
  void UpdateVelocity(float velocity);

  // Tries to broadcast a software reset to all position controllers in the
  // hope they will stop the motors.
  static void SoftwareEmergencyStop(void (*write)(unsigned char));

 private:
  static const unsigned int kMaximumSpeed = 30;  // TODO(whess): was: 60.

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

  // Set the travel destination to a distance relative from the current
  // position.
  inline void TravelFromHere(int distance_from_here);

  unsigned char (*read_)();
  void (*write_)(unsigned char);
  unsigned char address_;
  float wheel_radius_;
  unsigned int last_position_;
  int travel_goal_;
};

#endif  // PARSECLIB_POSITION_CONTROLLER_
