// Credentials live in secrets.h (git-ignored) so they are never pushed to GitHub
#include "secrets.h"

#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

TinyGPSPlus gps;
MPU6050 mpu6050(Wire);
HardwareSerial SerialGPS(1);
const int RXPin = 16, TXPin = 17;
const uint32_t GPSBaud = 9600;

double currentSpeed = 0.0, previousSpeed = 0.0;
double estimatedSpeed = 0.0;  // Accelerometer-based estimate, kept separate from GPS speed
unsigned long lastAccidentTime = 0;
const unsigned long accidentDebounce = 5000;

bool calibrated = false;
float initialAngle = 0.0;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Wire.begin();

  // Connects to Wi-Fi and Blynk using values from secrets.h
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connected to Blynk");

  mpu6050.begin();
  calibrateGyro();
}

void loop() {
  Blynk.run();
  readGPS();
  mpu6050.update();

  if (!calibrated) {
    calibrateGyro();
  }

  checkForAccidents();
  calculateSpeedFromAccelerometer();
  calculateLeanAngle();
  delay(100);
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isUpdated()) {
        Blynk.virtualWrite(V0, gps.location.lat());
        Blynk.virtualWrite(V1, gps.location.lng());
      }
      if (gps.speed.isUpdated()) {
        previousSpeed = currentSpeed;
        currentSpeed = gps.speed.kmph();
        Blynk.virtualWrite(V2, currentSpeed);
      }
    }
  }
}

void checkForAccidents() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastAccidentTime > accidentDebounce) {
    float speedDifference = previousSpeed - currentSpeed;
    float gyroAngle = max(abs(mpu6050.getAngleX()), max(abs(mpu6050.getAngleY()), abs(mpu6050.getAngleZ())));

    bool isAccidentDetected = false;
    String message = "Status: ";

    if (speedDifference >= 30 && currentSpeed > 0) {
      message += "Sudden braking detected; ";
      isAccidentDetected = true;
    }
    if (previousSpeed >= 30 && currentSpeed == 0) {
      message += "Vehicle stopped suddenly; ";
      isAccidentDetected = true;
    }
    if (gyroAngle > 75) {
      message += "High gyro movement detected; ";
      isAccidentDetected = true;
    }
    if (isAccidentDetected) {
      lastAccidentTime = currentMillis;
      Blynk.virtualWrite(V3, message);
      Serial.println(message);
    }
  }
}

void calculateSpeedFromAccelerometer() {
  const float g = 9.81;

  // MPU6050_tockn already returns acceleration in g, so convert straight to m/s^2
  float ax = mpu6050.getAccX() * g;
  float ay = mpu6050.getAccY() * g;
  float az = mpu6050.getAccZ() * g;

  float totalAcc = sqrt(ax * ax + ay * ay + az * az) - g;

  // Integrates over the 0.1 s loop and converts m/s to km/h; drifts over time, so GPS speed stays the primary value
  estimatedSpeed += totalAcc * 0.1 * 3.6;
  if (estimatedSpeed < 0) estimatedSpeed = 0;

  Blynk.virtualWrite(V4, estimatedSpeed);
}

void calculateLeanAngle() {
  // Sends tilt relative to the angle recorded at startup
  float gyroX = mpu6050.getAngleX() - initialAngle;
  Blynk.virtualWrite(V5, gyroX);
}

void calibrateGyro() {
  // Records the resting angle so lean is measured from the mounted position
  initialAngle = mpu6050.getAngleX();
  calibrated = true;
  Serial.print("Gyro calibration complete. Initial angle: ");
  Serial.println(initialAngle);
}
