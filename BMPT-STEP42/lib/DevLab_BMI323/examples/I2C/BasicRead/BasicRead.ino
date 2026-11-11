#include <DevLab_BMI323.h>

/*
  DevLab_BMI323 - Basic Read Example

  Description:
  This example demonstrates how to initialize and read
  accelerometer, gyroscope, and temperature data from
  the Bosch BMI323 IMU using the DevLab_BMI323 library.

  Supported Platforms:
  - ESP32
  - RP2040
  - Arduino-compatible boards with Wire support

  Connections:
  BMI323 -> MCU

  SDA   -> GPIO 6
  SCL   -> GPIO 7
  VDD   -> 3.3V
  VDDIO -> 3.3V
  GND   -> GND
  CSB   -> 3.3V
  SDO   -> 3.3V (I2C address 0x69)

  Author:
  Adrian Rabadan Ortiz

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#include <DevLab_BMI323.h>

// --------------------------------------------------
// I2C pin configuration
// --------------------------------------------------
#define SDA_PIN 6
#define SCL_PIN 7

// --------------------------------------------------
// Create BMI323 object
// --------------------------------------------------
DevLab_BMI323 imu(Wire, 0x69);

// Structure to store sensor data
BMI323_SensorData data;

// --------------------------------------------------
// Arduino setup
// --------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" DevLab BMI323 - Basic Read Example");
  Serial.println("==================================================");

  Serial.println("Initializing BMI323...");

  // Initialize BMI323
  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {

    Serial.println("ERROR: BMI323 initialization failed.");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("BMI323 initialized successfully.");
  Serial.println();
}

// --------------------------------------------------
// Arduino main loop
// --------------------------------------------------
void loop() {

  // Read sensor data
  if (imu.readData(data)) {

    Serial.println("--------------------------------------------------");

    // Accelerometer data
    Serial.print("Accelerometer [raw]");
    Serial.print("  X: ");
    Serial.print(data.accX);

    Serial.print("   Y: ");
    Serial.print(data.accY);

    Serial.print("   Z: ");
    Serial.println(data.accZ);

    // Gyroscope data
    Serial.print("Gyroscope     [raw]");
    Serial.print("  X: ");
    Serial.print(data.gyrX);

    Serial.print("   Y: ");
    Serial.print(data.gyrY);

    Serial.print("   Z: ");
    Serial.println(data.gyrZ);

    // Temperature data
    Serial.print("Temperature   [C]");
    Serial.print("    ");
    Serial.println(data.temperatureC, 2);

  } else {

    Serial.println("ERROR: Failed to read BMI323 data.");
  }

  delay(200);
}
