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

#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <SPI.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/Range.h>

#include <dogs102lcd.h>
#include <shift_brite.h>
#include <simple_led.h>
#include <parallax_ping.h>
#include <position_controller.h>

// SimpleLED status_led(13);
//
// // Endless loop showing LED flashing to show that we have crashed.
// void Check(bool assertion, int id) {
//   if (!assertion) {
//     PositionController::SoftwareEmergencyStop(&WriteUART1);
//     status_led.EnterCrashLoop(id);
//   }
// }

// ----------------------------------------------------------------------
// LCD stuff
// ----------------------------------------------------------------------

DOGS102LCD display(41, 40);  // Select pin 41, data pin 40.

static void SetupDisplay() {
  display.Initialize();
  for (int i = 0; i != 8; ++i) {
    display.WriteString(0, i, "                 ");
  }

  // Activate the LCD backlight.
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
}

static void printf_row(char row, const char *format, ...) {
  static char message[18];
  va_list ap;
  va_start(ap, format);
  int message_size = vsnprintf(message, 17, format, ap);
  message[18] = '\0';
  va_end(ap);
  display.WriteString(0, row, message);
}

// Endless loop flashing the LCD to show that we have crashed.
void Check(bool assertion, const char *format, ...) {
  if (!assertion) {
    PositionController::SoftwareEmergencyStop(&WriteUART1);
    display.WriteString(0, 3, " ROBO MEDITATION ");
    display.WriteString(0, 4, "                 ");
    static char error_message[18];
    va_list ap;
    va_start(ap, format);
    int message_size = vsnprintf(error_message, 17, format, ap);
    error_message[18] = '\0';
    va_end(ap);
    display.WriteString(8 - message_size / 2, 4, error_message);
    for (;;) {
      digitalWrite(8, HIGH);
      for (int i = 0; i != 500; ++i) delayMicroseconds(1000);
      digitalWrite(8, LOW);
      for (int i = 0; i != 500; ++i) delayMicroseconds(1000);
    }
  }
}

static const unsigned char large_heart[9] =
    {0x04, 0x0e, 0x1f, 0x3f, 0x7e, 0x3f, 0x1f, 0x0e, 0x04};

static const unsigned char small_heart[9] =
    {0x00, 0x04, 0x0e, 0x1e, 0x3c, 0x1e, 0x0e, 0x04, 0x00};

static void LoopDisplay() {
  static unsigned long last_heartbeat = 0;
  static bool last_heart_large = false;
  unsigned long now = micros();
  if (now - last_heartbeat > 700000l) {
    last_heartbeat = now;
    display.WritePixels(93, 0, last_heart_large ? small_heart : large_heart, 9);
    last_heart_large = !last_heart_large;
  }
}

static void ShowLogMessage(char* message) {
  size_t message_size = strlen(message);
  for (int i = 0; i != 4; ++i) {
    display.WriteString(0, 4 + i, "                 ");
    if (message_size > 17 * i) {
      display.WriteString(0, 4 + i, message + 17 * i);
    }
  }
}

// ----------------------------------------------------------------------
// Position controller stuff
// ----------------------------------------------------------------------

const float kBaseRadius = .195f;
const float kWheelRadius = .0762f;  // 6 inch wheels.

float forward_velocity = 0.0;
float angular_velocity = 0.0;
unsigned long last_update = 0;
const unsigned long timeout_us = 150000;

static bool IsUART1Available() {
  return UCSR1A & (1 << RXC1);
}

static unsigned char ReadUART1() {
  long delay = 0;
  const int delay_step = 100;
  while (!IsUART1Available() && delay < 50000l) {
    delayMicroseconds(delay_step);
    delay += delay_step;
  }
  Check(IsUART1Available(), "read failed");
  unsigned char data = UDR1;
  return data;
}

static void WriteUART1(unsigned char byte) {
  UCSR1B |= (1 << TXEN1);
  UCSR1B &= ~(1 << RXEN1);
  while(!(UCSR1A & (1 << UDRE1)));  // Wait until the data register is empty.
  UCSR1A |= (1 << TXC1);  // Clear the TX Complete flag by setting this bit.
  UDR1 = byte;
	while(!(UCSR1A & (1 << TXC1)));  // Wait until the byte was sent.
  UCSR1B &= ~(1 << TXEN1);
  UCSR1B |= (1 << RXEN1);
}

PositionController left_controller(&ReadUART1, &WriteUART1, 1, kWheelRadius);
PositionController right_controller(&ReadUART1, &WriteUART1, 2, kWheelRadius);

static void SetupPositionController() {
  // Position Controller Device serial port. Pins 19 (RX) and 18 (TX).
  // We depend on default for DDR, PORT, UCSR.
  UBRR1H = 0;
  UBRR1L = 51;  // 19200-ish baud from 16 MHz
  PORTD |= (1<<3);  // pull-up on TX1
  left_controller.Initialize(true);
  right_controller.Initialize(false);
}

static void LoopPositionController() {
  if (micros() - last_update > timeout_us) {
    forward_velocity = 0.0f;
    angular_velocity = 0.0f;
    last_update = micros();
  }
  float left_velocity = forward_velocity - kBaseRadius * angular_velocity;
  float right_velocity = forward_velocity + kBaseRadius * angular_velocity;
  printf_row(1, "Posctrl %d", 1);
  left_controller.UpdateVelocity(left_velocity);
  printf_row(1, "Posctrl %d", 2);
  right_controller.UpdateVelocity(right_velocity);
}

// ----------------------------------------------------------------------
// ROS serial communication
// ----------------------------------------------------------------------

ROS_CALLBACK(LogCallback, rosgraph_msgs::Log, log_message)
  if (log_message.level >= log_message.INFO) {  // We don't show DEBUG messages.
    ShowLogMessage(reinterpret_cast<char*>(log_message.msg));
  }
}

ros::Subscriber log_subscriber(
    "rosout", &log_message, &LogCallback);

ros::NodeHandle node_handle;

ROS_CALLBACK(VelocityCallback, geometry_msgs::Twist, velocity_message)
  forward_velocity = velocity_message.linear.x;
  angular_velocity = velocity_message.angular.z;
  last_update = micros();
}

ros::Subscriber velocity_subscriber(
    "cmd_vel", &velocity_message, &VelocityCallback);

void fx_open() {
  Serial.begin(115200);
}

int fx_putc(char c) {
  Serial.write(c);
  return 0;
}

int fx_getc() {
  return Serial.read();
}

static void SetupROSSerial() {
  node_handle.initNode();
  node_handle.subscribe(log_subscriber);
  node_handle.subscribe(velocity_subscriber);
}

static void LoopROSSerial() {
  node_handle.spinOnce();
}

// ----------------------------------------------------------------------
// Ultrasonic stuff
// ----------------------------------------------------------------------

Ultrasonic pings[10];
int next_ping = 0;

// Put a ultrasonic sensor reading in a ROS sensor_msgs::Range message.
void UltrasonicToMessage(float range, sensor_msgs::Range *range_message) {
  range_message->radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_message->field_of_view = 0.35f;
  range_message->min_range = 0.0f;
  range_message->max_range = 100.0f;
  range_message->range = range;
}

static void SetupUltrasonic() {
  // We use PORTK for addressing.
  DDRK = 0x0f;
}

static void LoopUltrasonic() {
  if (pings[next_ping].IsReady()) {
    printf_row(2, "Pinger %d", next_ping);
    printf_row(0, "%d", pings[next_ping].DebugTime());
    PORTK = next_ping;
    pings[next_ping].SendTriggerPulse();
    next_ping = (next_ping + 1) % 10;
  }
}

// ----------------------------------------------------------------------
// ShiftBrite stuff
// ----------------------------------------------------------------------

ShiftBrite shift_brite(12);  // Latch pin 12.

static void SetupShiftBrite() {
  pinMode(13, INPUT);
  pinMode(10, INPUT);
  shift_brite.Initialize(10);
  shift_brite.Enable(11);  // Enable pin 11.
}

static void ShiftBriteDemo() {
  for (;;) {
    int zeros[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int none[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int full[10] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
    shift_brite.UpdateColors(10, none, none, none);
    delay(1500);
    shift_brite.UpdateColors(10, full, full, full);
    delay(1500);
    shift_brite.UpdateColors(10, full, none, none);
    delay(1500);
    shift_brite.UpdateColors(10, none, full, none);
    delay(1500);
    shift_brite.UpdateColors(10, none, none, full);
    delay(1500);
    shift_brite.UpdateColors(10, none, none, none);
    delay(1500);
    for (int i = 0; i != 10; ++i) {
      none[i] = 1023;
      full[i] = 0;
      shift_brite.UpdateColors(10, none, full, zeros);
      none[i] = 0;
      full[i] = 1023;
      delay(500);
    }
    for (int i = 8; i >= 0; --i) {
      none[i] = 1023;
      full[i] = 0;
      shift_brite.UpdateColors(10, none, full, zeros);
      none[i] = 0;
      full[i] = 1023;
      delay(500);
    }
  }
}

static void LoopShiftBrite() {
  int datar[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int datag[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int zeros[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i != 10; ++i) {
    int cm = int(100 * pings[i].QueryDistance());
    if (cm < 0) cm = 0;
    if (cm > 100) cm = 100;
    datar[(i + 5) % 10] = 1023 - 10 * cm;
    datag[(i + 5) % 10] = 10 * cm;
  }
  shift_brite.UpdateColors(10, datar, datag, zeros);
}

// ----------------------------------------------------------------------

void setup() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SetupDisplay();
  SetupROSSerial();
  SetupUltrasonic();
  SetupPositionController();
  SetupShiftBrite();
  // ShiftBriteDemo();
}

void loop() {
  LoopDisplay();
  LoopROSSerial();
  LoopPositionController();
  LoopUltrasonic();
  LoopShiftBrite();
  delayMicroseconds(5);
}
