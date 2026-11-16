/*
  thruster_controller.ino
  Receives "FIRE:<heading_deg>:<pulse_ms>\n" from the Python tracker and opens
  the valve(s) that push the platform toward that direction.

  STATUS: written but NOT yet tested on hardware. Verify on the bench (see README).

  Conventions
    - heading_deg: desired direction of travel in TABLE coordinates,
      0 = east, 90 = north, counter-clockwise positive.
    - VALVE_PUSH_DEG: the direction each valve pushes the platform, in the
      PLATFORM's own frame (a valve whose nozzle points west pushes east = 0 deg).
    - The BNO055 heading increases clockwise, so platform yaw (CCW) = -heading.
      body_angle = world_angle - yaw = world_angle + heading.
      If the platform moves the wrong way once it has rotated, flip HEADING_SIGN.

  Parsing note
    - sscanf("%f") does NOT work on AVR boards (Uno/Nano), so the line is parsed
      by hand with strtod/strtol, which work on every board.

  Safety
    - Firing is refused until the BNO055 reports a usable calibration, because
      an uncalibrated heading would send thrust the wrong way.
    - Pulses are time-limited (MAX_PULSE_MS) and valves close by themselves.
    - If the serial link goes quiet, no valve stays open (nothing latches).
*/
#include <Wire.h>

#define USE_IMU 1          // set to 0 to test without the BNO055 (assumes heading = 0)
#if USE_IMU
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#endif

const uint8_t NUM_VALVES = 4;
const uint8_t VALVE_PIN[NUM_VALVES]        = {2, 3, 4, 5};
const float   VALVE_PUSH_DEG[NUM_VALVES]   = {0, 90, 180, 270};   // push E, N, W, S
const float   HEADING_SIGN   = 1.0;     // +1 or -1, see note above
const int     MAX_PULSE_MS   = 150;
const int     MIN_PULSE_MS   = 10;
const float   MAX_ANGLE_DIFF = 50.0;    // also fire valves within this angle of the best one
const uint8_t MIN_SYS_CAL    = 2;       // 0..3, BNO055 system calibration needed before firing
const uint8_t MIN_MAG_CAL    = 2;       // 0..3, magnetometer calibration needed (heading depends on it)
const unsigned long CAL_PRINT_MS = 2000;

#include <stdlib.h>
#include <string.h>

unsigned long valveOffAt[NUM_VALVES];
unsigned long lastCalPrint = 0;
bool          valveIsOn[NUM_VALVES];

char   lineBuf[40];
uint8_t lineLen = 0;

float angDiff(float a, float b) {                 // smallest difference, 0..180
  float d = fmod(a - b + 540.0, 360.0) - 180.0;
  return fabs(d);
}

float readHeadingDeg() {
#if USE_IMU
  sensors_event_t e;
  bno.getEvent(&e);
  return e.orientation.x;                         // 0..360, clockwise
#else
  return 0.0;
#endif
}

// True when the heading can be trusted. With USE_IMU 0 there is no IMU, so allow firing.
bool headingTrusted() {
#if USE_IMU
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  return sys >= MIN_SYS_CAL && mag >= MIN_MAG_CAL;
#else
  return true;
#endif
}

void printCalibration() {
#if USE_IMU
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("CAL sys="); Serial.print(sys);
  Serial.print(" gyro="); Serial.print(gyro);
  Serial.print(" accel="); Serial.print(accel);
  Serial.print(" mag="); Serial.println(mag);
#endif
}

void openValve(uint8_t i, int ms) {
  digitalWrite(VALVE_PIN[i], HIGH);
  valveIsOn[i] = true;
  valveOffAt[i] = millis() + ms;
}

void closeExpiredValves() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    if (valveIsOn[i] && (long)(now - valveOffAt[i]) >= 0) {
      digitalWrite(VALVE_PIN[i], LOW);
      valveIsOn[i] = false;
    }
  }
}

void fire(float worldDeg, int pulseMs) {
  if (!headingTrusted()) {
    Serial.println("NOCAL: wave the platform in a figure-8 until mag/sys calibrate; not firing.");
    return;
  }
  pulseMs = constrain(pulseMs, MIN_PULSE_MS, MAX_PULSE_MS);
  float bodyDeg = fmod(worldDeg + HEADING_SIGN * readHeadingDeg() + 720.0, 360.0);

  float d[NUM_VALVES];
  uint8_t best = 0;
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    d[i] = angDiff(bodyDeg, VALVE_PUSH_DEG[i]);
    if (d[i] < d[best]) best = i;
  }
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    if (i == best || d[i] <= MAX_ANGLE_DIFF) openValve(i, pulseMs);
  }
}

void handleLine(const char* line) {
  // Expected format: FIRE:<heading_deg>:<pulse_ms>
  if (strncmp(line, "FIRE:", 5) != 0) return;     // anything else is ignored
  char* end;
  float heading = (float)strtod(line + 5, &end);
  if (end == line + 5 || *end != ':') return;     // no number, or missing second field
  const char* pulseStart = end + 1;
  long pulse = strtol(pulseStart, &end, 10);
  if (end == pulseStart) return;
  fire(heading, (int)pulse);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                                // overflow: drop the line
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    pinMode(VALVE_PIN[i], OUTPUT);
    digitalWrite(VALVE_PIN[i], LOW);
    valveIsOn[i] = false;
  }
#if USE_IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not found - check wiring (SDA/SCL, address 0x28).");
    while (1) { delay(100); }                     // valves stay closed
  }
  bno.setExtCrystalUse(true);
#endif
  Serial.println("thruster_controller ready");
}

void loop() {
  readSerial();
  closeExpiredValves();
  if (millis() - lastCalPrint > CAL_PRINT_MS) {   // lets you watch calibration on the bench
    lastCalPrint = millis();
    printCalibration();
  }
}
