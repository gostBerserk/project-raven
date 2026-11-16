/*
  drone_rcs.ino  -  6-nozzle cold-gas assist for the drone
  Receives "FIRE:<heading_deg>:<pulse_ms>\n" and opens the valve(s) whose thrust points
  closest to that heading, in the DRONE BODY frame.

  STATUS: written but NOT yet tested on hardware. Bench-test with the drone unpowered and
  props removed (see docs/DRONE.md).

  Body-frame convention: 0 deg = forward (the side with the PROJECT RAVEN name), 90 = left,
  180 = back, 270 = right (counter-clockwise positive).
  VALVE_PUSH_DEG is the direction each valve pushes the DRONE, which is OPPOSITE to where
  its nozzle exhausts (a nozzle that blows to the left pushes the drone to the right).

    valve 1  nose, exhausts left   -> pushes right (270)
    valve 2  nose, exhausts right  -> pushes left  (90)
    valve 3  tail, exhausts left   -> pushes right (270)
    valve 4  tail, exhausts right  -> pushes left  (90)
    valve 5  front-face centreline, exhausts forward -> pushes back    (180)
    valve 6  back-face centreline, exhausts backward -> pushes forward (0)

  Valves 5 and 6 sit ON THE FUSELAGE CENTRELINE at the centre of mass. Put on a wing, a
  fore/aft nozzle twists the drone (a push that misses the centre makes torque).

  Keep this table identical to drone/rcs.py (tests/test_drone.py checks that they match).
  There is no IMU here: the Python side already works in the body frame.

  Safety
    - ARM switch: valves only open while ARM_PIN is held LOW (switch to ground).
      Leave it open on the bench and on the ground.
    - Pulses are time-limited (MAX_PULSE_MS) and valves close by themselves.
    - If the serial link goes quiet, no valve stays open (nothing latches).
*/
#include <stdlib.h>
#include <string.h>

const uint8_t NUM_VALVES = 6;
const uint8_t VALVE_PIN[NUM_VALVES]      = {2, 3, 4, 5, 6, 7};
const float   VALVE_PUSH_DEG[NUM_VALVES] = {270, 90, 270, 90, 180, 0};
const uint8_t ARM_PIN        = 8;       // INPUT_PULLUP: LOW = armed
const int     MAX_PULSE_MS   = 120;
const int     MIN_PULSE_MS   = 10;
const float   MAX_ANGLE_DIFF = 50.0;    // also fire valves within this angle of the best one

unsigned long valveOffAt[NUM_VALVES];
bool          valveIsOn[NUM_VALVES];

char   lineBuf[40];
uint8_t lineLen = 0;

float angDiff(float a, float b) {                 // smallest difference, 0..180
  float d = fmod(a - b + 540.0, 360.0) - 180.0;
  return fabs(d);
}

void openValve(uint8_t i, int ms) {
  digitalWrite(VALVE_PIN[i], HIGH);
  valveIsOn[i] = true;
  valveOffAt[i] = millis() + ms;
}

void closeAllValves() {
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    digitalWrite(VALVE_PIN[i], LOW);
    valveIsOn[i] = false;
  }
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

void fire(float bodyDeg, int pulseMs) {
  if (digitalRead(ARM_PIN) != LOW) {              // not armed: ignore
    Serial.println("DISARMED: close the ARM switch to fire.");
    return;
  }
  pulseMs = constrain(pulseMs, MIN_PULSE_MS, MAX_PULSE_MS);
  bodyDeg = fmod(fmod(bodyDeg, 360.0) + 360.0, 360.0);

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
  if (end == line + 5 || *end != ':') return;
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
  pinMode(ARM_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    pinMode(VALVE_PIN[i], OUTPUT);
  }
  closeAllValves();
  Serial.println("drone_rcs ready (valves fire only while the ARM switch is closed)");
}

void loop() {
  readSerial();
  if (digitalRead(ARM_PIN) != LOW) closeAllValves();   // disarming closes everything at once
  closeExpiredValves();
}
