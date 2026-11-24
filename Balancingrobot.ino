/*
  Self-balancing robot
  Arduino Uno + GY-521 (MPU-6050) + TB6612FNG + 2x JGA25-370 12 V motors with encoders

  Behaviour:
    1. Hold the robot upright. After 0.6 s the motors start and it balances.
    2. When it has been stable for 3 s, it drives forward a fixed distance.
    3. It checks that it is stable again, then drives backward the same distance.
    4. It waits until stable for 3 s again, and repeats from step 2.
  If it falls over (tilt > FALL_ANGLE_DEG) the motors switch off and it waits to be picked up.

  Pins follow the wiring sheet (balancing_robot_wiring.xlsx).

  FIRST USE: set BRINGUP_MODE to 1, then 2, then 0 (see the comments next to it).
  All gains below are STARTING GUESSES. Tuning will be made after initial startup (115200 baud):
    p            print all parameters
    kp 25        set a parameter (kp, ki, kd, kvp, kvi, ks, off, vdrive)
*/

#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ======================= PINS =======================
const uint8_t PIN_ENC_L_A = 2;   // INT0
const uint8_t PIN_ENC_R_A = 3;   // INT1
const uint8_t PIN_ENC_L_B = 4;
const uint8_t PIN_ENC_R_B = 5;
const uint8_t PIN_STBY = 6;
const uint8_t PIN_AIN2 = 7;
const uint8_t PIN_AIN1 = 8;
const uint8_t PIN_PWMA = 9;      // left motor speed
const uint8_t PIN_PWMB = 10;     // right motor speed
const uint8_t PIN_BIN2 = 11;
const uint8_t PIN_BIN1 = 12;
const uint8_t PIN_LED = 13;
const uint8_t PIN_VBAT = A0;     // battery divider 10k + 10k

// ======================= USER SETTINGS =======================
// 0 = run the robot
// 1 = print IMU angle only (motors off): check the signs below
// 2 = motor and encoder test (lift the wheels off the ground!)
#define BRINGUP_MODE 0

#define USE_BATTERY_SENSE 1   // set 0 if the A0 divider is not fitted
#define TELEMETRY 1           // print status 10 times per second
#define HIGH_FREQ_PWM 1       // 31 kHz PWM on pins 9 and 10 (silent, smoother)

// IMU orientation. Axes: 0 = X, 1 = Y, 2 = Z of the GY-521 board.
#define AXIS_UP 2             // accelerometer axis pointing up when the robot is upright
#define AXIS_FWD 0            // accelerometer axis pointing in the driving direction
#define AXIS_GYRO 1           // gyro axis that rotates about the wheel axle
#define SIGN_ACC_UP 1
#define SIGN_ACC_FWD -1
#define SIGN_GYRO 1
// Check in BRINGUP_MODE 1: leaning the robot FORWARD must give a POSITIVE angle,
// and the rate must be positive while you tilt it forward. Flip a sign (or change an axis) if not.

// Motors and encoders. Motors are mounted mirrored, so the right one is usually inverted.
// Check in BRINGUP_MODE 2: positive command -> wheel turns forward and the count goes UP.
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 1
#define ENC_L_SIGN 1
#define ENC_R_SIGN -1

// Geometry. COUNTS_PER_WHEEL_REV is a PLACEHOLDER: measure it in BRINGUP_MODE 2
// (turn a wheel exactly one revolution by hand and read the count).
const float WHEEL_DIAMETER_MM = 65.0f;
const float COUNTS_PER_WHEEL_REV = 748.0f;
const float MOVE_DISTANCE_MM = 300.0f;

// ======================= TUNABLE PARAMETERS (also changeable over serial) =======================
float KP = 22.0f;            // PWM per degree of tilt error
float KI = 0.0f;            // PWM per degree*second (usually leave at 0)
float KD = 0.9f;            // PWM per deg/s of tilt rate
float KVP = 4.0f;           // tilt command (deg) per rev/s of speed error
float KVI = 2.0f;           // tilt command (deg) per revolution of position error (holds position)
float KS = 20.0f;           // steering correction, PWM per revolution of wheel difference
float ANGLE_OFFSET = 0.0f;  // balance point in degrees (centre of mass trim)
float V_DRIVE_RPS = 0.8f;   // driving speed in wheel revolutions per second

// ======================= OTHER CONSTANTS =======================
const uint32_t LOOP_US = 5000;           // 200 Hz control loop
const float ALPHA = 0.98f;               // complementary filter
const float FALL_ANGLE_DEG = 35.0f;      // motors off beyond this tilt
const float START_ANGLE_DEG = 3.0f;      // must be this upright to start
const uint32_t START_HOLD_MS = 600;
const float MAX_TILT_CMD_DEG = 6.0f;
const float SPEED_INT_MAX = 0.6f;        // revolutions
const float ANGLE_INT_MAX = 100.0f;
const float STEER_MAX = 40.0f;
const float MOTOR_DEADBAND = 20.0f;      // PWM below which the motors do not turn
const float V_RAMP = 1.5f;               // rev/s^2 speed ramp

// "Stable" means: small tilt error, small tilt rate and almost no wheel speed
const float STABLE_ANGLE_DEG = 1.5f;
const float STABLE_RATE_DPS = 12.0f;
const float STABLE_SPEED_RPS = 0.15f;
const uint32_t STABLE_BEFORE_MOVE_MS = 3000;   // stable this long before driving forward
const uint32_t STABLE_AFTER_MOVE_MS = 2000;    // stable this long after forward, before backward
const uint32_t MOVE_TIMEOUT_MS = 6000;

const float VBAT_NOMINAL = 7.4f;
const float VBAT_MIN = 6.4f;             // stop below this (protects the cells)

const uint8_t MPU_ADDR = 0x68;

// ======================= STATE =======================
enum State { ST_IDLE, ST_HOLD, ST_FWD, ST_CHECK, ST_BACK };
State state = ST_IDLE;

float acc[3], gyr[3];
float gyroBias[3] = {0, 0, 0};
float angle = 0, gyroRate = 0;

volatile long countL = 0, countR = 0;
float posRev = 0, posL = 0, posR = 0;
float speedFilt = 0, lastPosForSpeed = 0, speedTime = 0;

float vSet = 0, vTarget = 0, speedInt = 0, angInt = 0, tiltCmd = 0, lastU = 0;
float moveStartPos = 0;
const float DIST_REV = MOVE_DISTANCE_MM / (PI * WHEEL_DIAMETER_MM);

uint32_t lastLoopUs = 0, stateSince = 0, stableSince = 0, idleOkSince = 0, lastTelemMs = 0;
float vbatF = 7.4f;
bool lowBattery = false;
uint8_t batDiv = 0;

// ======================= PARAMETER TABLE =======================
struct Param { const char *name; float *ptr; };
Param params[] = {
  {"kp", &KP}, {"ki", &KI}, {"kd", &KD}, {"kvp", &KVP},
  {"kvi", &KVI}, {"ks", &KS}, {"off", &ANGLE_OFFSET}, {"vdrive", &V_DRIVE_RPS}
};
const uint8_t NPARAMS = sizeof(params) / sizeof(params[0]);

// ======================= ENCODER INTERRUPTS =======================
void isrLeft() {
  if (digitalRead(PIN_ENC_L_A) == digitalRead(PIN_ENC_L_B)) countL++; else countL--;
}
void isrRight() {
  if (digitalRead(PIN_ENC_R_A) == digitalRead(PIN_ENC_R_B)) countR++; else countR--;
}
long readCount(volatile long &c) {
  noInterrupts(); long v = c; interrupts(); return v;
}

// ======================= MOTORS =======================
void motorsOff() {
  digitalWrite(PIN_STBY, LOW);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
  digitalWrite(PIN_LED, LOW);
}

// cmd: -255..255, positive = robot forward
void setMotor(bool left, float cmd) {
  if (left && MOTOR_L_INVERT) cmd = -cmd;
  if (!left && MOTOR_R_INVERT) cmd = -cmd;
  bool fwd = cmd >= 0;
  float mag = fabsf(cmd);
  int pwm = 0;
  if (mag >= 1.0f) {
    pwm = (int)(MOTOR_DEADBAND + mag * (255.0f - MOTOR_DEADBAND) / 255.0f);
    if (pwm > 255) pwm = 255;
  }
  uint8_t in1 = left ? PIN_AIN1 : PIN_BIN1;
  uint8_t in2 = left ? PIN_AIN2 : PIN_BIN2;
  uint8_t pw = left ? PIN_PWMA : PIN_PWMB;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  analogWrite(pw, pwm);
}

// ======================= IMU =======================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool readImu(bool applyBias) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;
  int16_t raw[7];
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  for (uint8_t i = 0; i < 3; i++) {
    acc[i] = raw[i] / 16384.0f;                       // +-2 g
    gyr[i] = raw[4 + i] / 65.5f;                      // +-500 deg/s
    if (applyBias) gyr[i] -= gyroBias[i];
  }
  return true;
}

void calibrateGyro() {
  float sum[3] = {0, 0, 0};
  const int N = 500;
  for (int n = 0; n < N; n++) {
    readImu(false);
    for (uint8_t i = 0; i < 3; i++) sum[i] += gyr[i];
    delay(3);
  }
  for (uint8_t i = 0; i < 3; i++) gyroBias[i] = sum[i] / N;
}

float accelAngle() {
  return atan2f(SIGN_ACC_FWD * acc[AXIS_FWD], SIGN_ACC_UP * acc[AXIS_UP]) * RAD_TO_DEG;
}

void updateAngle(float dt) {
  gyroRate = SIGN_GYRO * gyr[AXIS_GYRO];
  angle = ALPHA * (angle + gyroRate * dt) + (1.0f - ALPHA) * accelAngle();
}

// ======================= SERIAL TUNING =======================
void printParams() {
  for (uint8_t i = 0; i < NPARAMS; i++) {
    Serial.print(params[i].name); Serial.print('='); Serial.println(*params[i].ptr, 3);
  }
}

void parseCommand(char *s) {
  char *name = strtok(s, " ");
  char *val = strtok(NULL, " ");
  if (!name) return;
  if (strcmp(name, "p") == 0) { printParams(); return; }
  if (!val) return;
  float v = atof(val);
  for (uint8_t i = 0; i < NPARAMS; i++) {
    if (strcmp(name, params[i].name) == 0) {
      *params[i].ptr = v;
      Serial.print(name); Serial.print('='); Serial.println(v, 3);
      return;
    }
  }
  Serial.println(F("unknown parameter"));
}

void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) { buf[n] = 0; parseCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

// ======================= STATE MACHINE =======================
void changeState(State s) {
  state = s;
  stateSince = millis();
  stableSince = millis();
  if (s == ST_FWD || s == ST_BACK) moveStartPos = posRev;
  if (s == ST_CHECK || s == ST_HOLD) speedInt = 0;   // new position becomes the reference
}

void startBalancing() {
  noInterrupts(); countL = 0; countR = 0; interrupts();
  posRev = posL = posR = 0;
  lastPosForSpeed = 0; speedTime = 0; speedFilt = 0;
  vSet = vTarget = 0; speedInt = 0; angInt = 0; tiltCmd = 0;
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_LED, HIGH);
}

void updateStable() {
  float err = angle - (tiltCmd + ANGLE_OFFSET);
  bool ok = fabsf(err) < STABLE_ANGLE_DEG && fabsf(gyroRate) < STABLE_RATE_DPS &&
            fabsf(speedFilt) < STABLE_SPEED_RPS;
  if (!ok) stableSince = millis();
}

void runStateMachine() {
  uint32_t now = millis();

  if (state != ST_IDLE) {
    if (fabsf(angle) > FALL_ANGLE_DEG || lowBattery) {
      motorsOff();
      changeState(ST_IDLE);
      idleOkSince = now;
      return;
    }
    updateStable();
  }

  switch (state) {
    case ST_IDLE:
      vTarget = 0;
      if (!lowBattery && fabsf(angle) < START_ANGLE_DEG) {
        if (now - idleOkSince >= START_HOLD_MS) { startBalancing(); changeState(ST_HOLD); }
      } else {
        idleOkSince = now;
      }
      break;

    case ST_HOLD:                       // balance and wait until stable for 3 s
      vTarget = 0;
      if (now - stableSince >= STABLE_BEFORE_MOVE_MS) changeState(ST_FWD);
      break;

    case ST_FWD:                        // drive forward
      vTarget = V_DRIVE_RPS;
      if (posRev - moveStartPos >= DIST_REV || now - stateSince > MOVE_TIMEOUT_MS) changeState(ST_CHECK);
      break;

    case ST_CHECK:                      // still stable after the move?
      vTarget = 0;
      if (now - stableSince >= STABLE_AFTER_MOVE_MS) changeState(ST_BACK);
      break;

    case ST_BACK:                       // drive backward
      vTarget = -V_DRIVE_RPS;
      if (moveStartPos - posRev >= DIST_REV || now - stateSince > MOVE_TIMEOUT_MS) changeState(ST_HOLD);
      break;
  }
}

// ======================= CONTROL =======================
void updatePositionAndSpeed(float dt) {
  long cl = ENC_L_SIGN * readCount(countL);
  long cr = ENC_R_SIGN * readCount(countR);
  posL = cl / COUNTS_PER_WHEEL_REV;
  posR = cr / COUNTS_PER_WHEEL_REV;
  posRev = 0.5f * (posL + posR);
  speedTime += dt;
  if (speedTime >= 0.02f) {                         // speed every 20 ms
    float raw = (posRev - lastPosForSpeed) / speedTime;
    lastPosForSpeed = posRev;
    speedTime = 0;
    speedFilt += 0.3f * (raw - speedFilt);
  }
}

void updateBattery() {
#if USE_BATTERY_SENSE
  if (++batDiv < 20) return;                        // every 100 ms
  batDiv = 0;
  float v = analogRead(PIN_VBAT) * (5.0f / 1023.0f) * 2.0f;
  vbatF += 0.1f * (v - vbatF);
  if (vbatF < VBAT_MIN) lowBattery = true;
  else if (vbatF > VBAT_MIN + 0.3f) lowBattery = false;
#endif
}

void balanceControl(float dt) {
  // speed ramp
  float dv = V_RAMP * dt;
  if (vSet < vTarget) vSet = fminf(vSet + dv, vTarget);
  else vSet = fmaxf(vSet - dv, vTarget);

  // outer loop: speed/position -> tilt command
  float vErr = vSet - speedFilt;
  speedInt += vErr * dt;
  speedInt = constrain(speedInt, -SPEED_INT_MAX, SPEED_INT_MAX);
  tiltCmd = constrain(KVP * vErr + KVI * speedInt, -MAX_TILT_CMD_DEG, MAX_TILT_CMD_DEG);

  // inner loop: tilt -> motor command
  float err = angle - (tiltCmd + ANGLE_OFFSET);
  angInt += err * dt;
  angInt = constrain(angInt, -ANGLE_INT_MAX, ANGLE_INT_MAX);
  float u = KP * err + KI * angInt + KD * gyroRate;
#if USE_BATTERY_SENSE
  u *= constrain(VBAT_NOMINAL / vbatF, 0.8f, 1.4f);   // keep the same strength as the battery drains
#endif
  u = constrain(u, -255.0f, 255.0f);
  lastU = u;

  // keep both wheels together so the robot drives straight
  float steer = constrain(KS * (posL - posR), -STEER_MAX, STEER_MAX);
  setMotor(true, u - steer);
  setMotor(false, u + steer);
}

// ======================= SETUP / LOOP =======================
void setup() {
  pinMode(PIN_STBY, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  motorsOff();

#if HIGH_FREQ_PWM && defined(__AVR_ATmega328P__)
  TCCR1B = (TCCR1B & 0xF8) | 0x01;                  // Timer1 (pins 9, 10) -> ~31 kHz
#endif

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x02);   // digital low-pass filter ~94 Hz
  mpuWrite(0x1B, 0x08);   // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);   // accel +-2 g
  delay(100);

  if (!readImu(false)) {
    Serial.println(F("IMU not found. Check SDA (A4), SCL (A5), power."));
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  Serial.println(F("Keep the robot still: calibrating gyro..."));
  calibrateGyro();
  readImu(true);
  angle = accelAngle();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

#if USE_BATTERY_SENSE
  vbatF = analogRead(PIN_VBAT) * (5.0f / 1023.0f) * 2.0f;
#endif

#if BRINGUP_MODE == 2
  Serial.println(F("Motor test: wheels must be in the air."));
  digitalWrite(PIN_STBY, HIGH);
  setMotor(true, 80);  delay(1000); setMotor(true, 0);
  Serial.print(F("Left count after test: ")); Serial.println(ENC_L_SIGN * readCount(countL));
  setMotor(false, 80); delay(1000); setMotor(false, 0);
  Serial.print(F("Right count after test: ")); Serial.println(ENC_R_SIGN * readCount(countR));
  Serial.println(F("Both wheels should have turned FORWARD and both counts should be positive."));
  motorsOff();
#endif

  Serial.println(F("Ready. Type p for parameters."));
  lastLoopUs = micros();
  idleOkSince = millis();
}

void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt > 0.02f) dt = 0.02f;

  if (!readImu(true)) {                              // I2C failure: stop for safety
    motorsOff();
    changeState(ST_IDLE);
    return;
  }
  updateAngle(dt);

#if BRINGUP_MODE == 1
  static uint32_t t1 = 0;
  if (millis() - t1 > 50) {
    t1 = millis();
    Serial.print(F("angle ")); Serial.print(angle, 1);
    Serial.print(F("  rate ")); Serial.println(gyroRate, 1);
  }
  return;
#elif BRINGUP_MODE == 2
  static uint32_t t2 = 0;
  if (millis() - t2 > 200) {
    t2 = millis();
    Serial.print(F("countL ")); Serial.print(ENC_L_SIGN * readCount(countL));
    Serial.print(F("  countR ")); Serial.println(ENC_R_SIGN * readCount(countR));
  }
  return;
#endif

  updateBattery();
  updatePositionAndSpeed(dt);
  runStateMachine();
  if (state != ST_IDLE) balanceControl(dt);

#if TELEMETRY
  if (millis() - lastTelemMs >= 100) {
    lastTelemMs = millis();
    Serial.print(F("st ")); Serial.print((int)state);
    Serial.print(F(" ang ")); Serial.print(angle, 1);
    Serial.print(F(" cmd ")); Serial.print(tiltCmd, 1);
    Serial.print(F(" u ")); Serial.print(lastU, 0);
    Serial.print(F(" v ")); Serial.print(speedFilt, 2);
    Serial.print(F(" vb ")); Serial.println(vbatF, 2);
  }
#endif
}