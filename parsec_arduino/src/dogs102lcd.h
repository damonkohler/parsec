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

#ifndef PARSECLIB_DOGS102LCD_
#define PARSECLIB_DOGS102LCD_

// Class to connect to an EA DOGS102 LCD using SPI.
class DOGS102LCD {
 public:
  // Constructs a DOGS102LCD object and sets the pins appropriately. 
  DOGS102LCD(int select_pin, int data_pin);

  // Initializes the LCD. SPI.begin() must have been called before and
  // mode 3 used, MSB first.
  void Initialize();

  // Writes a character to the display using a 6x8 pixel font.
  // Columns are 0--16, rows 0--7, ascii 0--127.
  // Non-ASCII characters will be shown as space.
  void WriteCharacter(char column, char row, char ascii);

  // Writes a string to the display. Values as for WriteCharacter.
  void WriteString(char column, char row, const char* text);

  // Writes nx8 pixels to the display beginning at a given column.
  // The least-significant bit is displayed on the top of the row,
  // the most-significant bit at the bottom.
  // Columns are 0--101, rows 0--7, ascii 0--127.
  void WritePixels(char column, char row, const unsigned char* bitmap, char n);

 private:
  static const unsigned char font_6x8[128][6];

  inline void SetPage(unsigned char page);
  inline void SetColumn(unsigned char page);

  int select_pin_;
  int data_pin_;
};

#endif  // PARSECLIB_DOGS102LCD_
