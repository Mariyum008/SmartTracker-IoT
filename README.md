# :red_car: SmartTracker IoT - Vehicle Location and Speed Monitoring

![IoT Monitoring](IOT.jpeg)

An ESP32-based vehicle tracker that streams **live GPS location and speed** to the **Blynk** mobile app and raises **alerts for sudden braking, sudden stops, and abnormal tilt** - built so parents can keep an eye on young drivers in real time.

:page_facing_up: **Published research:** *IoT-Based Vehicle Location and Speed Monitoring for Parental Peace of Mind* - JETIR, Vol. 11, Issue 4, April 2024 - [Read paper](https://www.jetir.org/view?paper=JETIR2404639) | [PDF in repo](Published%20Research%20Paper(JETIR).pdf)

---

## :sparkles: Features

- :round_pushpin: **Live location** - latitude and longitude from the NEO-6M GPS, pushed to a Blynk map
- :zap: **Live speed** - GPS speed in km/h, plus an accelerometer-based estimate from the MPU6050
- :rotating_light: **Driving alerts** sent to the app:

| Alert | Trigger |
| --- | --- |
| Sudden braking | Speed drops by 30 km/h or more between GPS readings |
| Sudden stop | Vehicle goes from 30+ km/h to 0 |
| High gyro movement | Tilt angle on any axis exceeds 75° (possible rollover or crash) |

Alerts are debounced to one every 5 seconds to avoid spamming the app.

---

## :tools: Hardware

| Component | Role |
| --- | --- |
| ESP32 WROOM DevKit | Main controller and Wi-Fi connectivity |
| NEO-6M GPS module | Location and speed |
| MPU6050 (accelerometer + gyroscope) | Motion, tilt, and impact detection |
| Blynk IoT app | Mobile dashboard and alerts |

### Wiring

| Module | Module pin | ESP32 pin |
| --- | --- | --- |
| NEO-6M GPS | TX | GPIO 16 (RX2) |
| NEO-6M GPS | RX | GPIO 17 (TX2) |
| MPU6050 | SDA | GPIO 21 |
| MPU6050 | SCL | GPIO 22 |
| Both | VCC / GND | 3.3V / GND |

---

## :iphone: Blynk Datastreams

| Virtual pin | Data |
| --- | --- |
| V0 | Latitude |
| V1 | Longitude |
| V2 | GPS speed (km/h) |
| V3 | Alert message |
| V4 | Accelerometer speed estimate |
| V5 | Lean angle |

---

## :rocket: Getting Started

1. Install the **Arduino IDE** and add the **ESP32 board package**.
2. Install these libraries from the Library Manager:
   - `TinyGPSPlus`
   - `MPU6050_tockn`
   - `Blynk`
3. Create a Blynk template with the datastreams above and copy your Template ID and Auth Token.
4. Copy `secrets.example.h` to `secrets.h` and fill in your values (`secrets.h` is git-ignored, so your keys stay private):

```cpp
#define BLYNK_TEMPLATE_ID   "your-template-id"
#define BLYNK_TEMPLATE_NAME "SmartTracker"
#define BLYNK_AUTH_TOKEN    "your-auth-token"
#define WIFI_SSID           "your-wifi-name"
#define WIFI_PASSWORD       "your-wifi-password"
```

5. Select **ESP32 Dev Module**, upload `esp_code.ino`, and open the Serial Monitor at **115200 baud**.
6. Take the GPS module outdoors for its first fix, then open the Blynk app to see live data.

---

## :file_folder: Repository Contents

| File | Description |
| --- | --- |
| `esp_code.ino` | ESP32 firmware |
| `secrets.example.h` | Template for your Blynk and Wi-Fi credentials |
| `IOT.jpeg` | System diagram |
| `Project presentation.pptx` | Project presentation |
| `Published Research Paper(JETIR).pdf` | Published paper |

---

## :crystal_ball: Future Enhancements

- :world_map: **Geofencing** - alerts when the vehicle leaves a predefined safe zone
- :robot: **ML-based driver scoring** - flag risky driving patterns before they become incidents
- :wrench: **Vehicle diagnostics** - fuel level and engine health via OBD-II
- :globe_with_meridians: **Smart city integration** - traffic and accident data sharing

---

## :busts_in_silhouette: Team

- **Manila Gupta**
- **Mariyum Siddique** - [@Mariyum008](https://github.com/Mariyum008)
- **Aimaan Khan** - [@AiMk937](https://github.com/AiMk937)
- **Pranali Thorat**

B.E. Computer Engineering, University of Mumbai
