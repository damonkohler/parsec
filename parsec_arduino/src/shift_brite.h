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

#ifndef PARSECLIB_SHIFT_BRITE_
#define PARSECLIB_SHIFT_BRITE_

// Class to connect to a ShiftBrite using SPI.
class ShiftBrite {
 public:
  // Constructs a ShiftBrite object and sets the pin appropriately. 
  explicit ShiftBrite(int latch_pin);

  // Initializes the ShiftBrites. SPI.begin() must have been called before
  // and either mode 0 or 3 used.
  void Initialize(int n);

  // Enable the ShiftBrites by asserting the enable pin low.
  void Enable(int enable_pin);

  // Updates the colors of daisy-chained ShiftBrites.
  // Red, green and blue are 0--1023.
  void UpdateColors(int n, int red[], int green[], int blue[]);

 private:
  inline void UpdateColor(int red, int green, int blue);
  inline void SendData(unsigned long data);
  inline void Latch(int n);

  int latch_pin_;
};

#endif  // PARSECLIB_SHIFT_BRITE_
