/** 
    DevLab_BMI323 - Advanced Read Example

  Description:
  This example sweep measurements for accelerometer,
  gyroscope, and temperature data,between all the
  available configs from the Bosch BMI323 IMU using 
  the DevLab_BMI323 library.

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
  Jonathan Mejorado Lopez

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#include <Wire.h>
#include "DevLab_BMI323.h"

// ------------- I2C Cfg
#define PIN_INT1 D7
#define PIN_INT2 D6
#define SDA_PIN 6
#define SCL_PIN 7
#define FAST_SPEED  400000
// ----------- Registros principales ------------------------

#define BMI323_ADDR 0x69

#define REG_CHIP_ID       0x00

#define REG_ACC_DATA_X    0x03
#define REG_ACC_DATA_Y    0x04
#define REG_ACC_DATA_Z    0x05
#define REG_GYR_DATA_X    0x06
#define REG_GYR_DATA_Y    0x07
#define REG_GYR_DATA_Z    0x08
#define REG_TEMP_DATA     0x09
#define REG_ACC_CONF      0x20
#define REG_GYR_CONF      0x21
#define REG_CMD           0x7E

uint16_t regBuilded = 0;
// Tiempos de asentamiento seguros (Periodo Base de cada ODR multiplicado por 2.5)
// Incluye el nuevo valor inicial para 0.78 Hz en el índice 0.
const uint16_t tiemposAsentamientoODR[15] = { 3200, 1600, 800, 400, 200, 100, 50, 25, 13, 10, 10, 10, 10, 10, 10 };
//Valores de configuracion acelerometro 
const uint8_t acc_odr[14] = { 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE };
const uint8_t acc_range[4] = { 0x0, 0x1, 0x2, 0x3 };
const uint8_t acc_bw[2] = { 0x0, 0x1 };
const uint8_t acc_avgnum[7] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
const uint8_t acc_mode[4] = { 0x0, 0x3, 0x4, 0x7 };

const char* etiquetasAccOdr[] = { "0.78", "1.56", "3.12", "6.25", "12.5", "25", "50", "100", "200", "400", "800", "1K6", "3K2", "6K4" };
const char* etiquetasAccRange[] = { "2G", "4G", "8G", "16G" };
const char* etiquetasAccBw[] = { "ODR/2", "ODR/4" };
const char* etiquetasAccAVGNUM[] = { "NOAVG", "AVG2", "AVG4", "AVG8", "AVG16", "AVG32", "AVG64" };
const char* etiquetasMode[] = { "Disabled", "Cycling", "Continous", "HPM"};

// ----------------- Giroscopio Config --------------------------

//Valores de configuracion acelerometro 
const uint8_t gyr_odr[14] = { 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE };
const uint8_t gyr_range[5] = { 0x0, 0x1, 0x2, 0x3, 0x4 };
const uint8_t gyr_bw[2] = { 0x0, 0x1};
const uint8_t gyr_avgnum[7] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6 };
const uint8_t gyr_mode[5] = { 0x0, 0x1, 0x3, 0x4, 0x7 };

const char* etiquetasGyrOdr[] = { "0.78", "1.56", "3.12", "6.25", "12.5", "25", "50", "100", "200", "400", "800", "1K6", "3K2", "6K4" };
const char* etiquetasGyrRange[] = { "+-125", "+-250", "+-500", "+-1000","+-2000" };
const char* etiquetasGyrBw[] = { "ODR/2", "ODR/4" };
const char* etiquetasGyrAVGNUM[] = { "NOAVG", "AVG2", "AVG4", "AVG8", "AVG16", "AVG32", "AVG64"};
const char* etiquetasGyrMode[] = { "Disabled", "DDriveEnabled", "Cycling", "Continous", "HPM"};

// ── Valores esperados / constantes ──────3──────────────────
#define BMI323_CHIP_ID    0x43
#define BMI323_SOFT_RST   0xDEAF  // Comando soft reset

//Configuracion Constructor IMU
DevLab_BMI323 imu(Wire,BMI323_ADDR);

BMI323_SensorData data;
BMI323_AccCfg   acc_Config;
BMI323_GyrCfg   gyr_Config;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(500);

  Serial.println(F("\n============================================"));
  Serial.println(F("  BMI323 — UNIT ELECTRONICS"));
  Serial.println(F("  Datasheet: BST-BMI323-DS000-13"));
  Serial.println(F("============================================\n"));

  // Inicializar I2C a 400kHz (modo Fm)
   // Initialize BMI323
  if (!imu.begin(SDA_PIN, SCL_PIN, FAST_SPEED)) {

    Serial.println("ERROR: BMI323 initialization failed.");

    while (1) {
      delay(1000);
    }
  }
  delay(5);
  //Verificacion ID del dispositivo
  imu.test_chip_id(BMI323_CHIP_ID,REG_CHIP_ID);


  
  Serial.println("BMI323 initialized successfully.");
  Serial.println();
}

void sweepMeasurementsGyr(){
    for (int m = 2; m < 5; m++) {
      gyr_Config.gyr_mode = gyr_mode[m];
      for(int i = 0; i < 14; i++){
        gyr_Config.gyr_odr = gyr_odr[i];

        imu.softReset();
        delay(15);
          for(int j = 0; j < 4; j++){
            gyr_Config.gyr_range = acc_range[j];
            for(int k = 0; k < 2; k++){
              gyr_Config.gyr_bw = acc_bw[k];
              for(int l = 0; l < 7; l++){
                gyr_Config.gyr_avgnum = gyr_avgnum[l];
          

                //imu.softReset();


                regBuilded = imu.configGyrBMI323(gyr_Config);

                
                delay(tiemposAsentamientoODR[i] + 20);
                if (imu.readData(data)) {
                  
                  Serial.print(regBuilded, HEX);
                  Serial.print(";");
                  Serial.print(etiquetasGyrMode[m]);
                  Serial.print(";");
                  Serial.print(etiquetasGyrOdr[i]);
                  Serial.print(";");
                  Serial.print(etiquetasGyrRange[j]);
                  Serial.print(";");
                  Serial.print(etiquetasGyrBw[k]);
                  Serial.print(";");
                  Serial.print(etiquetasGyrAVGNUM[l]);
                  Serial.print(";");
                  // Accelerometer data
                  Serial.print(data.gyrX);
                  Serial.print(";");
                  Serial.print(data.gyrY);
                  Serial.print(";");
                  Serial.println(data.gyrZ);

                } else {

                  Serial.println("ERROR: Failed to read BMI323 data.");
                }

              }
            }
          }
        }
      }
    
}


void sweepMeasurements(){
  for (int m = 1; m < 4; m++) {
   acc_Config.acc_mode = acc_mode[m];
    for(int i = 0; i < 14; i++){
      acc_Config.acc_odr = acc_odr[i];

      imu.softReset();
      delay(15);
      for(int j = 0; j < 4; j++){
        acc_Config.acc_range = acc_range[j];
        for(int k = 0; k < 2; k++){
          acc_Config.acc_bw = acc_bw[k];
          for(int l = 0; l < 7; l++){
            acc_Config.acc_avgnum = acc_avgnum[l];
          

            //imu.softReset();


            regBuilded = imu.configAccBMI323(acc_Config);

            
            delay(tiemposAsentamientoODR[i] + 20);
            if (imu.readData(data)) {
              
              Serial.print(regBuilded, HEX);
              Serial.print(";");
              Serial.print(etiquetasMode[m]);
              Serial.print(";");
              Serial.print(etiquetasAccOdr[i]);
              Serial.print(";");
              Serial.print(etiquetasAccRange[j]);
              Serial.print(";");
              Serial.print(etiquetasAccBw[k]);
              Serial.print(";");
              Serial.print(etiquetasAccAVGNUM[l]);
              Serial.print(";");
              // Accelerometer data
              Serial.print(data.accX);
              Serial.print(";");
              Serial.print(data.accY);
              Serial.print(";");
              Serial.println(data.accZ);


              /*

              // Temperature data
              Serial.print("Temperature   [C]");
              Serial.print("    ");
              Serial.println(data.temperatureC, 2);
              */

            } else {

              Serial.println("ERROR: Failed to read BMI323 data.");
            }

          }
        }
      }
    }
  }
}

void sweepTemp(){
          
            if (imu.readData(data)) {
              
              
              // Temperature data
              Serial.print("Temperature   [C]");
              Serial.print(";");
              Serial.print("    ");
              Serial.println(data.temperatureC, 2);

            } else {

              Serial.println("ERROR: Failed to read BMI323 data.");
            }

          
       
}
void loop() {
  // put your main code here, to run repeatedly:
  //sweepMeasurements();
  sweepMeasurementsGyr();
  //sweepTemp();
  //imu.test_chip_id(BMI323_CHIP_ID, REG_CHIP_ID);

}
