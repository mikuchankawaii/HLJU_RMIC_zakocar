/*
DevLab_BMI323 - SPI Basic Read Example

  Description:
  This example demonstrates how to initialize and read
  accelerometer, gyroscope, and temperature data from
  the Bosch BMI323 IMU using SPI 4 Wire conecction
  on the DevLab_BMI323 library.

  Supported Platforms:
  - ESP32
  - RP2040
  - Arduino-compatible boards with Wire support

  Connections:
  BMI323 -> MCU

  SDA   -> D11
  SCK   -> D13
  VDD   -> 3.3V
  VDDIO -> 3.3V
  GND   -> GND
  CSB   -> D1O
  SA0   -> D12

  Author:
  Jonathan Mejorado Lopez

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#include <SPI.h>
#include "DevLab_BMI323.h"



/// *************Setup SPI Config
#define MOSI_PIN D11
#define MISO_PIN D12
#define SCK_PIN  D13
#define CS_PIN   D10 
#define SPI_FAST_SPEED 4000000

SPIClass spi_bus(SPI);                    //Inicializacion de bus SPI

DevLab_BMI323 imuSpi(spi_bus,CS_PIN,MISO_PIN,MOSI_PIN,SCK_PIN,SPI_FAST_SPEED);

BMI323_SensorData data;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Iniciando puerto");

  if(!imuSpi.begin()){
    Serial.println("Error : BMI323 initialization failed");

    while(1){
      delay(1000);
    }
  }
  Serial.println("BMI323 Initialized succesfully");
  delay(10);

  imuSpi.test_chip_id(BMI323_CHIP_ID, REG_CHIP_ID);

}

void loop() {

    // Read sensor data
  if (imuSpi.readData(data)) {

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
