/*
 * cart_motor_controller_uno.ino
 * Arduino UNO firmware for the Hudson cart (4-wheel skid-steer, BLDC).
 *
 * Skid-steer: both LEFT drivers share one PWM/DIR/BRAKE/EN signal set, both RIGHT drivers
 * share another. Wire each Uno signal pin to the matching input on BOTH drivers of that side.
 * Speed feedback is read from ONE driver per side (front left, front right).
 *
 * Same serial protocol as the Mega version, so motor_bridge.py needs no changes:
 *   Jetson -> Uno:  M,<left>,<right>   (-1000..1000)   |   S   (stop)
 *   Uno -> Jetson:  F,<rpmL>,<rpmL>,<rpmR>,<rpmR>,<timedOut>   every 50 ms
 *                   (left/right RPM repeated so the FL,RL,FR,RR order still matches)
 *
 * Safety: 250 ms command watchdog -> ramp to 0 + brake, then EN dropped (if EN_OFF_WHEN_IDLE);
 * no direction flip while moving. EN is disabled at boot.
 * The hardware e-stop must still cut motor POWER.
 */

#include <Arduino.h>

#if !defined(__AVR_ATmega328P__)
#error "This is the UNO version: choose Tools > Board > Arduino Uno"
#endif

// ======================= CONFIG: CHECK AGAINST YOUR DRIVER =======================
enum { LEFT = 0, RIGHT = 1 };
const uint8_t NUM_SIDES = 2;

const uint8_t PWM_PIN[NUM_SIDES]   = {9, 10};  // Timer1 pins (safe to speed up; Timer0 runs millis)
const uint8_t DIR_PIN[NUM_SIDES]   = {4, 7};
const uint8_t BRAKE_PIN[NUM_SIDES] = {8, 12};
const uint8_t FB_PIN[NUM_SIDES]    = {2, 3};   // the Uno's only two interrupt pins
const uint8_t EN_PIN[NUM_SIDES]    = {5, 6};   // driver EN input (one per side, shared by both drivers)

// Right-side motors face the opposite way. If a side spins backward, flip its entry.
const bool INVERT_DIR[NUM_SIDES] = {false, true};

const bool BRAKE_ACTIVE_HIGH = true;   // true: BRAKE pin HIGH = brake engaged
const bool PWM_INVERTED      = false;  // true if driver treats 0% duty as full speed
const bool FAST_PWM          = true;   // ~3.9 kHz on pins 9, 10 instead of ~490 Hz. Check driver.

// ---- EN (enable) -- SET AFTER THE BENCH TEST ----
const bool USE_EN            = true;   // false = never touch EN pins (e.g. if EN is hard-wired)
const bool EN_ACTIVE_LOW     = true;   // true: EN pin LOW = driver enabled. UNCONFIRMED: bench test!
const bool EN_OFF_WHEN_IDLE  = true;   // true: drop EN once stopped after a timeout/S/e-stop.
                                       // false: leave EN on after boot so the brake can hold on a slope.

const bool    USE_ESTOP_INPUT = false; // true once a NC e-stop contact is wired to ESTOP_PIN
const uint8_t ESTOP_PIN       = A0;    // INPUT_PULLUP, LOW = pressed

const float PULSES_PER_MOTOR_REV = 12.0;  // driver speed-pulse output per motor rev. CHECK DATASHEET.
const float GEAR_RATIO           = 188.0;

const uint32_t BAUD              = 115200;
const uint16_t CMD_TIMEOUT_MS    = 250;
const int16_t  MAX_CMD           = 1000;
const uint16_t CONTROL_PERIOD_MS = 10;
const int16_t  RAMP_PER_TICK     = 40;   // 0 -> full in 250 ms
const uint16_t REPORT_PERIOD_MS  = 50;
// ================================================================================

volatile uint32_t pulseCount[NUM_SIDES] = {0, 0};
void isrLeft()  { pulseCount[LEFT]++; }
void isrRight() { pulseCount[RIGHT]++; }

int16_t target[NUM_SIDES]  = {0, 0};
int16_t current[NUM_SIDES] = {0, 0};
bool    enOn[NUM_SIDES]    = {false, false};

uint32_t lastCmdMs = 0, lastControlMs = 0, lastReportMs = 0;
bool timedOut = true;

char rxBuf[32];
uint8_t rxLen = 0;

// --------------------------------------------------------------------------------
void setBrake(uint8_t i, bool on) {
  digitalWrite(BRAKE_PIN[i], (on == BRAKE_ACTIVE_HIGH) ? HIGH : LOW);
}

void setEnable(uint8_t i, bool on) {
  enOn[i] = on;
  if (!USE_EN) return;
  digitalWrite(EN_PIN[i], (on == !EN_ACTIVE_LOW) ? HIGH : LOW);
}

void writeSide(uint8_t i, int16_t cmd) {
  bool forward = (cmd >= 0);
  if (INVERT_DIR[i]) forward = !forward;
  digitalWrite(DIR_PIN[i], forward ? HIGH : LOW);

  uint16_t mag = abs(cmd);
  uint8_t duty = (uint8_t)((uint32_t)mag * 255 / MAX_CMD);
  if (PWM_INVERTED) duty = 255 - duty;

  setBrake(i, mag == 0 && target[i] == 0);
  analogWrite(PWM_PIN[i], duty);
}

void rampSide(uint8_t i) {
  int16_t t = target[i];
  int16_t c = current[i];
  if ((c > 0 && t < 0) || (c < 0 && t > 0)) t = 0;  // through zero before reversing
  if (t > c) c = min((int16_t)(c + RAMP_PER_TICK), t);
  else       c = max((int16_t)(c - RAMP_PER_TICK), t);
  current[i] = c;

  // Enable as soon as anything is commanded; drop EN only after fully stopped.
  if (target[i] != 0 || c != 0) {
    if (!enOn[i]) setEnable(i, true);
  } else if (EN_OFF_WHEN_IDLE && timedOut && enOn[i]) {
    setEnable(i, false);
  }

  writeSide(i, c);
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
      target[LEFT] = clampCmd(l);
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

float sideRpm(uint8_t i, uint32_t counts, uint32_t dtMs) {
  float outRpm = (counts / PULSES_PER_MOTOR_REV) * (60000.0 / dtMs) / GEAR_RATIO;
  return (current[i] < 0) ? -outRpm : outRpm;
}

void report(uint32_t dtMs) {
  uint32_t c[NUM_SIDES];
  noInterrupts();
  c[LEFT] = pulseCount[LEFT];   pulseCount[LEFT] = 0;
  c[RIGHT] = pulseCount[RIGHT]; pulseCount[RIGHT] = 0;
  interrupts();

  float l = sideRpm(LEFT, c[LEFT], dtMs);
  float r = sideRpm(RIGHT, c[RIGHT], dtMs);
  Serial.print("F,");
  Serial.print(l, 2); Serial.print(',');
  Serial.print(l, 2); Serial.print(',');
  Serial.print(r, 2); Serial.print(',');
  Serial.print(r, 2); Serial.print(',');
  Serial.println(timedOut ? 1 : 0);
}

// --------------------------------------------------------------------------------
void setup() {
  // Put EN in its disabled state FIRST, before anything else, so motors stay off.
  if (USE_EN) {
    for (uint8_t i = 0; i < NUM_SIDES; i++) {
      digitalWrite(EN_PIN[i], EN_ACTIVE_LOW ? HIGH : LOW);  // set level before making it an output
      pinMode(EN_PIN[i], OUTPUT);
    }
  }

  Serial.begin(BAUD);

  for (uint8_t i = 0; i < NUM_SIDES; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(BRAKE_PIN[i], OUTPUT);
    pinMode(FB_PIN[i], INPUT_PULLUP);
    setBrake(i, true);
    analogWrite(PWM_PIN[i], PWM_INVERTED ? 255 : 0);
    enOn[i] = false;
  }

  attachInterrupt(digitalPinToInterrupt(FB_PIN[LEFT]), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(FB_PIN[RIGHT]), isrRight, RISING);

  if (FAST_PWM) {
    TCCR1B = (TCCR1B & 0b11111000) | 0x02;  // Timer1 prescaler 8 -> ~3.9 kHz on pins 9, 10
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
  }

  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    report(now - lastReportMs);
    lastReportMs = now;
  }
}
