/*
 * cart_motor_controller_uno.ino
 * Arduino UNO firmware for the Hudson cart (4-wheel skid-steer, BLDC).
 *
 * PIN PLAN (matches the wiring drawing):
 *   SV  (PWM, one per driver) : FL=3, RL=5, FR=6, RR=9
 *   F/R (direction, one per driver) : FL=4, RL=7, FR=8, RR=12
 *   EN  (ONE wire, split to all four drivers' EN inputs) : 10
 *   GND : Arduino GND tied to the fuse block negative bus
 *   Pins 2 and 11 are left free (2 = interrupt pin for future PG feedback;
 *   if you later want PG on 2 AND 3, move FL's SV from 3 to 11).
 *
 * Skid-steer: LEFT side = FL + RL, RIGHT side = FR + RR. Both motors on a side
 * receive the same command.
 *
 * Serial protocol (same as the Mega version, motor_bridge.py unchanged):
 *   Jetson -> Uno:  M,<left>,<right>   (-1000..1000)   |   S   (stop)
 *   Uno -> Jetson:  F,<FL>,<RL>,<FR>,<RR>,<timedOut>   every 50 ms
 *   NOTE: no PG/speed feedback is wired, so the four RPM fields are always 0.00.
 *
 * Safety: 250 ms command watchdog -> ramp to 0, then EN dropped (if EN_OFF_WHEN_IDLE).
 * No direction flip while moving. EN is disabled at boot.
 * The hardware e-stop must still cut motor POWER.
 */

#include <Arduino.h>

#if !defined(__AVR_ATmega328P__)
#error "This is the UNO version: choose Tools > Board > Arduino Uno"
#endif

// ======================= CONFIG =======================
enum { FL = 0, RL = 1, FR = 2, RR = 3 };
const uint8_t NUM_MOTORS = 4;

// Index order: FL, RL, FR, RR. All SV pins are real PWM pins on the Uno (3,5,6,9,10,11).
const uint8_t SV_PIN[NUM_MOTORS]  = {3, 5, 6, 9};
const uint8_t DIR_PIN[NUM_MOTORS] = {4, 7, 8, 12};
const uint8_t EN_PIN              = 10;   // single shared enable for all four drivers

// Right-side motors face the opposite way. If a motor spins backward, flip its entry.
const bool INVERT_DIR[NUM_MOTORS] = {false, false, true, true};

const bool SV_INVERTED = false;   // true if driver treats 0% duty as full speed

// ---- EN (enable) -- CONFIRM ON THE BENCH ----
const bool USE_EN           = true;   // false = never touch EN (e.g. EN hard-wired)
const bool EN_ACTIVE_LOW    = true;   // true: EN LOW = enabled. UNCONFIRMED: bench test!
const bool EN_OFF_WHEN_IDLE = true;   // true: drop EN once stopped after timeout/S/e-stop

const bool    USE_ESTOP_INPUT = false; // true once a NC e-stop contact is wired to ESTOP_PIN
const uint8_t ESTOP_PIN       = A0;    // INPUT_PULLUP, LOW = pressed

const uint32_t BAUD              = 115200;
const uint16_t CMD_TIMEOUT_MS    = 250;
const int16_t  MAX_CMD           = 1000;
const uint16_t CONTROL_PERIOD_MS = 10;
const int16_t  RAMP_PER_TICK     = 40;   // 0 -> full in 250 ms
const uint16_t REPORT_PERIOD_MS  = 50;
// ======================================================

enum { LEFT = 0, RIGHT = 1 };

int16_t target[2]  = {0, 0};   // per side
int16_t current[2] = {0, 0};   // per side
bool    enOn       = false;

uint32_t lastCmdMs = 0, lastControlMs = 0, lastReportMs = 0;
bool timedOut = true;

char rxBuf[32];
uint8_t rxLen = 0;

// --------------------------------------------------------------------------------
void setEnable(bool on) {
  enOn = on;
  if (!USE_EN) return;
  // active-low: on -> LOW, off -> HIGH.  active-high: on -> HIGH, off -> LOW.
  digitalWrite(EN_PIN, (on == !EN_ACTIVE_LOW) ? HIGH : LOW);
}

void writeMotor(uint8_t m, int16_t cmd) {
  bool forward = (cmd >= 0);
  if (INVERT_DIR[m]) forward = !forward;
  digitalWrite(DIR_PIN[m], forward ? HIGH : LOW);

  uint16_t mag = abs(cmd);
  uint8_t duty = (uint8_t)((uint32_t)mag * 255 / MAX_CMD);
  if (SV_INVERTED) duty = 255 - duty;
  analogWrite(SV_PIN[m], duty);
}

void rampSide(uint8_t side) {
  int16_t t = target[side];
  int16_t c = current[side];
  if ((c > 0 && t < 0) || (c < 0 && t > 0)) t = 0;  // through zero before reversing
  if (t > c) c = min((int16_t)(c + RAMP_PER_TICK), t);
  else       c = max((int16_t)(c - RAMP_PER_TICK), t);
  current[side] = c;

  if (side == LEFT) { writeMotor(FL, c); writeMotor(RL, c); }
  else              { writeMotor(FR, c); writeMotor(RR, c); }
}

// One EN wire for all drivers: on if anything is commanded or still moving,
// off only when both sides are fully stopped after a timeout/S/e-stop.
void updateEnable() {
  bool active = target[LEFT] != 0 || target[RIGHT] != 0 ||
                current[LEFT] != 0 || current[RIGHT] != 0;
  if (active) {
    if (!enOn) setEnable(true);
  } else if (EN_OFF_WHEN_IDLE && timedOut && enOn) {
    setEnable(false);
  }
}

int16_t clampCmd(long v) {
  if (v >  MAX_CMD) return  MAX_CMD;
  if (v < -MAX_CMD) return -MAX_CMD;
  return (int16_t)v;
}

void stopAll() { target[LEFT] = target[RIGHT] = 0; }

// --------------------------------------------------------------------------------
void handleLine(char *line) {
  if (line[0] == 'M') {
    long l, r;
    if (sscanf(line, "M,%ld,%ld", &l, &r) == 2) {
      target[LEFT]  = clampCmd(l);
      target[RIGHT] = clampCmd(r);
      lastCmdMs = millis();
      timedOut = false;
    }
  } else if (line[0] == 'S') {
    stopAll();
    timedOut = true;   // lets EN drop once the ramp finishes
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; handleLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}

void report() {
  // No speed feedback wired: keep the F line format so motor_bridge.py still parses it.
  Serial.print("F,0.00,0.00,0.00,0.00,");
  Serial.println(timedOut ? 1 : 0);
}

// --------------------------------------------------------------------------------
void setup() {
  // EN first, set to its DISABLED level before making it an output, so motors stay off.
  if (USE_EN) {
    digitalWrite(EN_PIN, EN_ACTIVE_LOW ? HIGH : LOW);
    pinMode(EN_PIN, OUTPUT);
  }
  enOn = false;

  Serial.begin(BAUD);

  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    pinMode(SV_PIN[m], OUTPUT);
    pinMode(DIR_PIN[m], OUTPUT);
    analogWrite(SV_PIN[m], SV_INVERTED ? 255 : 0);
  }

  if (USE_ESTOP_INPUT) pinMode(ESTOP_PIN, INPUT_PULLUP);

  lastControlMs = lastReportMs = millis();
  Serial.println("READY");
}

void loop() {
  readSerial();
  uint32_t now = millis();

  bool estop = USE_ESTOP_INPUT && digitalRead(ESTOP_PIN) == LOW;
  if (estop || (now - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopAll();
    timedOut = true;
  }

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs += CONTROL_PERIOD_MS;
    rampSide(LEFT);
    rampSide(RIGHT);
    updateEnable();
  }

  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    report();
    lastReportMs = now;
  }
}
