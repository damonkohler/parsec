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
#include <WProgram.h>

#include "dogs102lcd.h"
#include "geometry_msgs/Twist.h"
#include "odometry.h"
#include "parallax_ping.h"
#include "parsec_msgs/Odometry.h"
#include "position_controller.h"
#include "profiler.h"
#include "rosgraph_msgs/Log.h"
#include "ros.h"
#include "sensor_msgs/Range.h"
#include "shift_brite.h"
#include "simple_led.h"

inline float fminf(float x, float y) {
  return x < y ? x : y;
}

inline float fmaxf(float x, float y) {
  return x > y ? x : y;
}

ros::NodeHandle node_handle;

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
  vsnprintf(message, 18, format, ap);
  va_end(ap);
  display.WriteString(0, row, message);
}

static void DumpProfiler(char row, const Profiler &profiler) {
  printf_row(row, "%6lu %s", profiler.GetElapsedMicros(), profiler.GetName());
}

static void SetMotorPower(bool enable);

// Endless loop flashing the LCD to show that we have crashed.
void Check(bool assertion, const char *format, ...) {
  if (!assertion) {
    SetMotorPower(false);
    //PositionController::SoftwareEmergencyStop(&WriteUART1);
    display.WriteString(0, 3, "      ERROR      ");
    display.WriteString(0, 4, "                 ");
    static char error_message[18];
    va_list ap;
    va_start(ap, format);
    vsnprintf(error_message, 18, format, ap);
    va_end(ap);
    display.WriteString(8 - strlen(error_message) / 2, 4, error_message);
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
  static int last_second = -1;
  int current_second = now / 1000000lu;
  if (current_second != last_second) {    
    printf_row(0, "Uptime %d", current_second);
    last_second = current_second;
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
// Ultrasonic stuff
// ----------------------------------------------------------------------

const int kNumPingers = 10;
Ultrasonic pings[kNumPingers];
int current_ping = 0;
int next_ping = 0;
const float kStopDistance = 0.05f;  // Stop at 5cm. Parallax PING))) sensors work down to 2cm.
const float kStopTime = 3.0f;  // Adapt speed to not have to stop before (in seconds).
float ping_distance[kNumPingers] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float last_distance[kNumPingers] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const char kGreen = 0, kYellow = 1, kRed = 2;
char ping_state[kNumPingers] = {
    kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed, kRed};
const int kPingSuccessor[kNumPingers] = {5, 6, 7, 8, 9, 1, 2, 3, 4, 0};
// >>> print ', '.join(map(str, (
// ... math.cos(2 * math.pi * i / 12)
// ... for i in (4, 5, 6, 7, 8, 10, 11, 0, 1, 2))))
const float kPingerDirection[kNumPingers] = {
    0.5f, 0.8660254f, 1.0f, 0.8660254f, 0.5f,
    -0.5f, -0.8660254f, -1.0f, -0.8660254f, -0.5f};

// Put an ultrasonic sensor reading in a ROS sensor_msgs::Range message.
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
    // Interpret the last reading. We take the median of this reading, the
    // last reading and the stored value to make it more robust. Two consistent
    // readings will be stored into ping_distance and a single outlier just
    // gets ignored.
    float new_distance = pings[current_ping].QueryDistance();
    float min_distance = fminf(new_distance, last_distance[current_ping]);
    float max_distance = fmaxf(new_distance, last_distance[current_ping]);
    if (ping_distance[current_ping] < min_distance) {
      ping_distance[current_ping] = min_distance;
    } else if (ping_distance[current_ping] > max_distance) {
      ping_distance[current_ping] = max_distance;
    }
    last_distance[current_ping] = new_distance;

    // Continue with the next ultrasonic sensor.
    current_ping = next_ping;
    PORTK = current_ping;
    pings[current_ping].SendTriggerPulse(current_ping);
    next_ping = kPingSuccessor[current_ping];
  }
}

static void MakeUltrasonicSafe(float* velocity) {
  float safety_factor = 1.0;
  for (int i = 0; i != kNumPingers; ++i) {
    float ping_delta = *velocity * kPingerDirection[i] * kStopTime;
    float distance = ping_distance[i] - kStopDistance;
    if (ping_delta >= 0.0f ||
        distance + ping_delta >= 0.0f) {
      // Either its already moving away, or even after kStopTime this velocity
      // is not enough for this pinger reading to drop below kStopDistance.
      ping_state[i] = distance <= 0.0f ? kRed : kGreen;
    } else {
      if (distance <= 0.0f) {
        ping_state[i] = kRed;
        safety_factor = 0;
      } else {
        ping_state[i] = kRed;
        safety_factor = fminf(safety_factor, -distance / ping_delta);
      }
    }
  }
  *velocity *= fmaxf(safety_factor, 0.0f);
}

// ----------------------------------------------------------------------
// Position controller stuff
// ----------------------------------------------------------------------

const float kBaseRadius = .195f;
const float kWheelRadius = .0762f;  // 6 inch wheels.

float forward_velocity = 0.0;
float angular_velocity = 0.0;
unsigned long last_update = 0;
const unsigned long kTimeoutMicros = 500000ul;
unsigned long last_moving = 0;
const unsigned long kMotorShutoffDelayMicros = 5000000ul;
const int kMotorPowerPin = 9;

Odometry odometry;
float left_averaged_odometry = 0.0f;
float right_averaged_odometry = 0.0f;
unsigned long last_odometry_update = 0;
unsigned long last_odometry_message = 0;
parsec_msgs::Odometry odometry_message;
ros::Publisher odometry_publisher("odom_simple", &odometry_message);

static void SetMotorPower(bool enable) {
  digitalWrite(kMotorPowerPin, enable ? HIGH : LOW);
}

static bool IsUART1Available() {
  return UCSR1A & (1 << RXC1);
}

static int ReadUART1() {
  long delay = 0;
  const int delay_step = 100;
  while (!IsUART1Available() && delay < 50000l) {
    delayMicroseconds(delay_step);
    delay += delay_step;
  }
  if (IsUART1Available()) {
    unsigned char data = UDR1;
    return data;
  } else {
    return -1;
  }
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

static void UpdateOdometry(float left_odometry, float right_odometry) {
  unsigned long odometry_micros = micros();
  left_averaged_odometry += left_odometry;
  right_averaged_odometry += right_odometry;
  if (odometry_micros - last_odometry_update > 10000ul) {
    odometry.UpdateFromWheels(
        left_averaged_odometry, right_averaged_odometry,
        2 * kBaseRadius, 1e-6f * (odometry_micros - last_odometry_update));
    left_averaged_odometry = 0.0f;
    right_averaged_odometry = 0.0f;
    last_odometry_update = odometry_micros;
  }
  if (odometry_micros - last_odometry_message > 30000ul) {
    odometry.ToMessage(node_handle, &odometry_message);
    odometry_publisher.publish(&odometry_message);
    last_odometry_message = odometry_micros;
  }
}

static void LoopPositionController() {
  if (micros() - last_update > kTimeoutMicros) {
    forward_velocity = 0.0f;
    angular_velocity = 0.0f;
    last_update = micros();
  }
  float safe_velocity = forward_velocity;
  MakeUltrasonicSafe(&safe_velocity);
  float left_velocity = safe_velocity - kBaseRadius * angular_velocity;
  float right_velocity = safe_velocity + kBaseRadius * angular_velocity;
  float left_odometry = left_controller.UpdateVelocity(-left_velocity);
  float right_odometry = right_controller.UpdateVelocity(-right_velocity);
  UpdateOdometry(-left_odometry, -right_odometry);
  // Only power the motors if we want to maintain velocity.
  if (left_velocity || right_velocity) {
    last_moving = micros();
    SetMotorPower(true);
  } else if (micros() - last_moving > kMotorShutoffDelayMicros) {
    // TODO(whess): Deactivated due to stability issues.
    // SetMotorPower(false);
  }
}

// ----------------------------------------------------------------------
// ROS serial communication
// ----------------------------------------------------------------------

void VelocityCallback(const geometry_msgs::Twist& velocity_message) {
  forward_velocity = velocity_message.linear.x;
  angular_velocity = velocity_message.angular.z;
  last_update = micros();
}

ros::Subscriber<geometry_msgs::Twist> velocity_subscriber(
    "cmd_vel", &VelocityCallback);

static void SetupROSSerial() {
  node_handle.initNode();
  node_handle.subscribe(velocity_subscriber);
  node_handle.advertise(odometry_publisher);
}

static void LoopROSSerial() {
  node_handle.spinOnce();
  static int last_error_count = -1;
  if (node_handle.getErrorCount() != last_error_count) {
    printf_row(1, "ErrorCount %d", node_handle.getErrorCount());
    last_error_count = node_handle.getErrorCount();
  }
}

// ----------------------------------------------------------------------
// ShiftBrite stuff
// ----------------------------------------------------------------------

ShiftBrite shift_brite(42);  // Latch pin 42.

static void SetupShiftBrite() {
  pinMode(13, INPUT);
  pinMode(10, INPUT);
  shift_brite.Initialize(10);
  shift_brite.Enable(43);  // Enable pin 43.
}

static void LoopShiftBrite() {
  // Static saves about 20 us.
  static int red[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static int green[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static int blue[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i != 10; ++i) {
    if (ping_state[i] == kGreen) {
      red[i] = 0;
      green[i] = 1023;
    } else if (ping_state[i] == kYellow) {
      red[i] = 1023;
      green[i] = 1023;
    } else {
      red[i] = 1023;
      green[i] = 0;
    }
  }
  // Only initialize every so often to double performance, cutting time from
  // 1276 us to 700 us.
  static int initialize_next_at = 0;
  if (initialize_next_at == 0) {
    shift_brite.Initialize(10);
    initialize_next_at = 10;
  } else {
    --initialize_next_at;
  }
  shift_brite.UpdateColors(10, red, green, blue);
}

// ----------------------------------------------------------------------

void setup() {
  {
    // TODO(whess): Stabilize the hardware, so that this is unnecessary.
    SetMotorPower(true);
    for (int i = 0; i != 1000; ++i) { delayMicroseconds(1000); }
  }
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SetupDisplay();
  SetupROSSerial();
  SetupUltrasonic();
  SetupPositionController();
  SetupShiftBrite();
}

void loop() {
  LoopDisplay();
  LoopROSSerial();
  LoopPositionController();
  LoopUltrasonic();
  LoopShiftBrite();
}
