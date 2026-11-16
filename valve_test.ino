/*
  valve_test.ino  -  Day 3 bench test
  Pulses each valve for 100 ms, once per second, and prints which one fired.
  Run this with the AIR DISCONNECTED first (you should hear/see each valve click),
  then with air at low pressure (2 bar or less) and check that every nozzle puffs.

  Wiring: Arduino pin -> 100-220 ohm -> logic-level MOSFET gate (10k gate-to-GND).
          Valve between +12V and MOSFET drain, flyback diode across the valve
          (cathode/band to +12V). MOSFET source and Arduino GND share ground.
*/
const uint8_t NUM_VALVES = 4;
const uint8_t VALVE_PIN[NUM_VALVES] = {2, 3, 4, 5};

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    pinMode(VALVE_PIN[i], OUTPUT);
    digitalWrite(VALVE_PIN[i], LOW);   // valves closed
  }
  Serial.println("Valve test: firing each valve for 100 ms, once per second.");
}

void loop() {
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    Serial.print("Valve "); Serial.print(i); Serial.print(" (pin ");
    Serial.print(VALVE_PIN[i]); Serial.println(")");
    digitalWrite(VALVE_PIN[i], HIGH);
    delay(100);
    digitalWrite(VALVE_PIN[i], LOW);
    delay(900);
  }
}
