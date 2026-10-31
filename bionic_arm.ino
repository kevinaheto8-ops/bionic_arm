#include <Servo.h>

/*
  Bionic Arm: flex sensors + accelerometer (MPU6050) + servos
  Board: Arduino Nano (A6 is analog-only, used for the 5th flex sensor)
         On an Uno, replace A6 with another free analog pin (A4/A5 are used by I2C).

  - 5 flex sensors (glove) -> 5 finger servos
  - MPU6050 accelerometer  -> wrist servo (roll) + elbow servo (pitch)
  - Auto-calibration on startup: open hand, then closed fist

  WIRING
  Flex sensor: one leg to 5V, other leg to analog pin AND to GND through a 10k resistor (voltage divider).
  Flex pins : Thumb A0, Index A1, Middle A2, Ring A3, Pinky A6
  Finger servos: D3, D5, D6, D9, D10
  Wrist servo: D11   Elbow servo: D12
  MPU6050: VCC 3.3V/5V, GND, SDA -> A4, SCL -> A5

  IMPORTANT: Power the servos from a separate 5-6V supply (2A+), NOT the Arduino 5V pin.
  Connect the supply GND to the Arduino GND.
*/

#include <Servo.h>
#include <Wire.h>

// ---------- Pins ----------
const uint8_t NUM_FINGERS = 5;
const uint8_t flexPins[NUM_FINGERS]   = {A0, A1, A2, A3, A6};
const uint8_t fingerPins[NUM_FINGERS] = {3, 5, 6, 9, 10};
const uint8_t WRIST_PIN = 11;
const uint8_t ELBOW_PIN = 12;

// ---------- Servo setup ----------
Servo fingers[NUM_FINGERS];
Servo wrist, elbow;

// Servo angle when finger is open / closed. Swap values per finger if it moves backwards.
int openAngle[NUM_FINGERS]   = {0,   0,   0,   0,   0};
int closedAngle[NUM_FINGERS] = {170, 170, 170, 170, 170};

// ---------- Flex calibration ----------
int flexOpen[NUM_FINGERS];    // raw reading with hand open
int flexClosed[NUM_FINGERS];  // raw reading with fist closed

// ---------- Smoothing ----------
float smoothFinger[NUM_FINGERS];
float smoothWrist = 90, smoothElbow = 90;
const float FINGER_ALPHA = 0.30;  // 0-1, higher = faster/jerkier
const float ARM_ALPHA    = 0.10;  // arm movement is smoother

// ---------- MPU6050 ----------
const uint8_t MPU_ADDR = 0x68;
float accX, accY, accZ;

void mpuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // power management register
  Wire.write(0);      // wake up
  Wire.endTransmission(true);
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
  if (Wire.available() >= 6) {
    int16_t rx = (Wire.read() << 8) | Wire.read();
    int16_t ry = (Wire.read() << 8) | Wire.read();
    int16_t rz = (Wire.read() << 8) | Wire.read();
    accX = rx / 16384.0;  // in g (default +/-2g range)
    accY = ry / 16384.0;
    accZ = rz / 16384.0;
  }
}

// ---------- Helpers ----------
int readFlexAvg(uint8_t pin, uint8_t samples = 16) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / samples;
}

void calibrate() {
  Serial.println(F("Calibration: hold hand FLAT/OPEN for 3 seconds..."));
  delay(3000);
  for (uint8_t i = 0; i < NUM_FINGERS; i++) flexOpen[i] = readFlexAvg(flexPins[i], 32);

  Serial.println(F("Now make a tight FIST for 3 seconds..."));
  delay(3000);
  for (uint8_t i = 0; i < NUM_FINGERS; i++) flexClosed[i] = readFlexAvg(flexPins[i], 32);

  Serial.println(F("Calibration done."));
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    Serial.print(F("Finger "));  Serial.print(i);
    Serial.print(F(": open="));  Serial.print(flexOpen[i]);
    Serial.print(F(" closed=")); Serial.println(flexClosed[i]);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    fingers[i].attach(fingerPins[i]);
    fingers[i].write(openAngle[i]);
    smoothFinger[i] = openAngle[i];
  }
  wrist.attach(WRIST_PIN);
  elbow.attach(ELBOW_PIN);
  wrist.write(90);
  elbow.write(90);

  mpuInit();
  calibrate();
}

// ---------- Loop ----------
void loop() {
  // 1) Fingers from flex sensors
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    int raw = analogRead(flexPins[i]);

    // Guard against identical calibration values (divide by zero / no-op map)
    if (flexOpen[i] != flexClosed[i]) {
      int target = map(raw, flexOpen[i], flexClosed[i], openAngle[i], closedAngle[i]);
      int lo = min(openAngle[i], closedAngle[i]);
      int hi = max(openAngle[i], closedAngle[i]);
      target = constrain(target, lo, hi);

      smoothFinger[i] += FINGER_ALPHA * (target - smoothFinger[i]);
      fingers[i].write((int)smoothFinger[i]);
    }
  }

  // 2) Wrist + elbow from accelerometer tilt
  mpuRead();

  // Roll (rotation around X axis) -> wrist, Pitch -> elbow
  float roll  = atan2(accY, accZ) * 180.0 / PI;                          // -180..180
  float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI; // -90..90

  int wristTarget = constrain(map((int)roll,  -90, 90, 0, 180), 0, 180);
  int elbowTarget = constrain(map((int)pitch, -90, 90, 0, 180), 0, 180);

  smoothWrist += ARM_ALPHA * (wristTarget - smoothWrist);
  smoothElbow += ARM_ALPHA * (elbowTarget - smoothElbow);
  wrist.write((int)smoothWrist);
  elbow.write((int)smoothElbow);

  // 3) Debug output (comment out once everything works)
  Serial.print(F("Roll: "));  Serial.print(roll, 1);
  Serial.print(F("  Pitch: ")); Serial.print(pitch, 1);
  Serial.print(F("  Flex: "));
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    Serial.print(analogRead(flexPins[i]));
    Serial.print(' ');
  }
  Serial.println();

  delay(15);  // ~60 Hz update
}
