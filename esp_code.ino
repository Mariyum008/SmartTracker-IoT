#define BLYNK_TEMPLATE_ID "TMPL3MNSFDQTb"
#define BLYNK_TEMPLATE_NAME "ESP32"
#define BLYNK_AUTH_TOKEN "0AWqG0X1wWC-NKoolKOgSDWDF0_ip8Y3"

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
unsigned long lastAccidentTime = 0;
const unsigned long accidentDebounce = 5000;

const char* ssid = "Aim";
const char* password = "MeowMeow";

bool calibrated = false;
float initialAngle = 0.0;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Wire.begin();
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize MPU6050 sensor
  mpu6050.begin();
  
  // Calibrate gyro
  calibrateGyro();
  
  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  Serial.println("Connected to Blynk");
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
  // Acceleration due to gravity (m/s^2)
  const float g = 9.81;

  // Calculate acceleration components
  float ax = mpu6050.getAccX() / 16384.0;
  float ay = mpu6050.getAccY() / 16384.0;
  float az = mpu6050.getAccZ() / 16384.0;

  // Print raw accelerometer data for debugging
  Serial.print("Raw Acceleration: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);

  // Calculate total acceleration (excluding gravity)
  float totalAcc = sqrt(ax * ax + ay * ay + az * az) - g;

  // Print calculated acceleration for debugging
  Serial.print("Total Acceleration: ");
  Serial.println(totalAcc);

  // Integrate acceleration to obtain speed
  currentSpeed += totalAcc * 0.1; // Assuming a fixed time step of 0.1 seconds (100 milliseconds)

  // Print calculated speed for debugging
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);

  // Send calculated speed to Blynk
  Blynk.virtualWrite(V4, currentSpeed);
}

// void calculateLeanAngle() {
//   // Read raw gyroscope data from the MPU6050
//   float gyroX = mpu6050.getAngleX() - initialAngle; // Adjusted angle
  
//   // Print the raw gyro data and adjusted angle
//   Serial.print("Raw Gyro Angle: ");
//   Serial.print(mpu6050.getAngleX());
//   Serial.print(", Adjusted Angle: ");
//   Serial.println(gyroX);
  
//   // Send calculated lean angle to Blynk
//   Blynk.virtualWrite(V5, gyroX);
// }

// void calibrateGyro() {
//   // Calibrate the gyro by taking the initial orientation as reference
//   initialAngle = mpu6050.getAngleX();
//   calibrated = true;
  
//   Serial.print("Gyro Calibration Complete. Initial Angle: ");
//   Serial.println(initialAngle);
// }
