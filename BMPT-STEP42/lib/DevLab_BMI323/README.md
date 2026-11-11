# DevLab_BMI323

Arduino library for the Bosch BMI323 6-axis IMU using direct I2C register communication.

This library provides a lightweight and reliable object-oriented interface for:

- Accelerometer acquisition
- Gyroscope acquisition
- Temperature reading

Compatible with ESP32, RP2040, and Arduino-compatible platforms.


---

# Features

- Direct 16-bit register communication
- Object-oriented API
- Lightweight implementation
- High-speed I2C communication
- Supports custom I2C pins
- Handles BMI323 dummy I2C bytes internally
- Compatible with Bosch BMI323 devices
- Optimized for embedded systems and rapid prototyping

---

# Supported Interfaces

| Interface | Support Status |
|---|---|
| I2C | Supported |
| SPI | Supported |

---

# Installation

## Manual Installation


1. Open Arduino IDE
2. Go to:

```text
Sketch -> Library Manager -> Search DevLab_BMI323...
```

3. Click on Install

4. Compile and upload the examples for the sensor

---

# Quick Start Example

```cpp
#include <DevLab_BMI323.h>

#define SDA_PIN 1
#define SCL_PIN 6

DevLab_BMI323 imu(Wire, 0x69);
BMI323_SensorData data;

void setup() {

  Serial.begin(115200);

  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {

    Serial.println("BMI323 initialization failed.");

    while (1);
  }
}

void loop() {

  if (imu.readData(data)) {

    Serial.print("ACC X: ");
    Serial.print(data.accX);

    Serial.print(" Y: ");
    Serial.print(data.accY);

    Serial.print(" Z: ");
    Serial.println(data.accZ);

    Serial.print("GYR X: ");
    Serial.print(data.gyrX);

    Serial.print(" Y: ");
    Serial.print(data.gyrY);

    Serial.print(" Z: ");
    Serial.println(data.gyrZ);

    Serial.print("Temperature: ");
    Serial.print(data.temperatureC);
    Serial.println(" °C");
  }

  delay(200);
}
```

---

# Wiring Example

| BMI323 | MCU |
|---|---|
| SDA | SDA |
| SCL | SCL |
| VDD | 3.3V |
| VDDIO | 3.3V |
| GND | GND |
| CSB | 3.3V |
| SDO | 3.3V (0x69) or GND (0x68) |

---

# Compatibility

| MCU Platform | Status |
|---|---|
| ESP32 | Tested |
| ESP32-C6 | Tested |
| ESP32-C5 | Tested |
| ESP32-H2 | Tested |
| Arduino-compatible boards | Compatible |

---

# Notes

BMI323 I2C burst reads include 2 dummy bytes at the beginning of the transfer.

The DevLab_BMI323 library automatically handles those internally.

Current library version supports only I2C communication.

SPI support may be added in future releases.

---

# Folder Structure

```text
DevLab_BMI323/
├── examples/
│   └── BasicRead/
├── src/
│   ├── DevLab_BMI323.h
│   └── DevLab_BMI323.cpp
├── library.properties
├── README.md
└── LICENSE
```

---

# Version

| Parameter | Value |
|---|---|
| Library Name | DevLab_BMI323 |
| Version | 1.0.0 |
| Communication | I2C/SPI|
| Architecture | Cross-platform |

---

# Author

Adrián Rabadán Ortiz | Jonathan Mejorado Lopez 

UNIT Electronics - DevLab Ecosystem

---

# License

MIT License
