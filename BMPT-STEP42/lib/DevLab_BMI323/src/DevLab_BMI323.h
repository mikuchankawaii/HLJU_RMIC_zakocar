/*
  DevLab_BMI323.h

  Arduino library for Bosch BMI323 IMU
  using direct register communication

  Features:
  - Accelerometer reading
  - Gyroscope reading
  - Temperature reading
  - Direct 16-bit register communication
  - Compatible with ESP32, RP2040, and Arduino platforms

  Author:
  Adrian Rabadan Ortiz | Jonathan Mejorado Lopez

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#ifndef DEVLAB_BMI323_H
#define DEVLAB_BMI323_H


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define BMI323_CHIP_ID 0x43
#define REG_CHIP_ID 0x00
    
  // ── Fuentes de interrupción ─────────────────────────────────────
// Cada valor es una máscara de 1 bit en los registros INT1_MAP/INT2_MAP.
// El bit position corresponde al layout del datasheet 6.1.17.
// El usuario combina con OR si necesita múltiples fuentes en una llamada:
//   imu.mapINT(BMI323_SRC_DRDY_ACC | BMI323_SRC_DRDY_GYR, BMI323_INT1);

enum BMI323_INT1_SRC : uint8_t {
  BMI323_SRC1_NO_MOTION_OUT     = 0,   
  BMI323_SRC1_ANY_MOTION_OUT    = 2,   
  BMI323_SRC1_FLAT_OUT          = 4,   
  BMI323_SRC1_ORIENTATION_OUT   = 6,   
  BMI323_SRC1_STEP_DETECTOR_OUT = 8,   
  BMI323_SRC1_STEP_COUNTER_OUT  = 10,   
  BMI323_SRC1_SIG_MOTION_OUT    = 12,   
  BMI323_SRC1_TILT_OUT          = 14
};

enum BMI323_INT2_SRC : uint8_t {
  BMI323_SRC2_TAP_OUT             = 0,   
  BMI323_SRC2_I3C_OUT             = 2,   
  BMI323_SRC2_ERR_STATUS          = 4,   
  BMI323_SRC2_TEMP_DRDY_INT       = 6,   
  BMI323_SRC2_GYR_DRDY_INT        = 8,   
  BMI323_SRC2_ACC_DRDY_INT        = 10,  
  BMI323_SRC2_FIFO_WATERMARK_INT  = 12,   
  BMI323_SRC2_FIFO_FULL_INT       = 14 
};
// ── Destino del pin físico ──────────────────────────────────────
enum BMI323_INT_DEST : uint8_t {
  BMI323_NONE = 0x0,   // Interrupt Disabled
  BMI323_INT1 = 0x1,   // solo pin INT1 (pin 4 del IC)
  BMI323_INT2 = 0x2,   // solo pin INT2 (pin 9 del IC)
  BMI323_IBI  = 0x3,   // I3C IBI
};


enum BMI323_ACC_ODR : uint8_t {
  BMI323_ACC_ODR_0_78HZ  = 0x1,
  BMI323_ACC_ODR_1_56HZ  = 0x2,
  BMI323_ACC_ODR_3_12HZ  = 0x3,
  BMI323_ACC_ODR_6_25HZ  = 0x4,
  BMI323_ACC_ODR_12_5HZ  = 0x5,
  BMI323_ACC_ODR_25HZ    = 0x6,
  BMI323_ACC_ODR_50HZ    = 0x7,
  BMI323_ACC_ODR_100HZ   = 0x8,
  BMI323_ACC_ODR_200HZ   = 0x9,
  BMI323_ACC_ODR_400HZ   = 0xA,
  BMI323_ACC_ODR_800HZ   = 0xB,
  BMI323_ACC_ODR_1600HZ  = 0xC,
  BMI323_ACC_ODR_3200HZ  = 0xD,
  BMI323_ACC_ODR_6400HZ  = 0xE,
};

enum BMI323_ACC_RANGE : uint8_t {
  BMI323_ACC_RANGE_2G  = 0x0,
  BMI323_ACC_RANGE_4G  = 0x1,
  BMI323_ACC_RANGE_8G  = 0x2,
  BMI323_ACC_RANGE_16G = 0x3,
};

enum BMI323_ACC_BW : uint8_t {
  BMI323_ACC_BW_ODR_2 = 0x0,
  BMI323_ACC_BW_ODR_4 = 0x1,
};

enum BMI323_ACC_AVGNUM : uint8_t {
  BMI323_ACC_AVG_NONE = 0x0,
  BMI323_ACC_AVG_2    = 0x1,
  BMI323_ACC_AVG_4    = 0x2,
  BMI323_ACC_AVG_8    = 0x3,
  BMI323_ACC_AVG_16   = 0x4,
  BMI323_ACC_AVG_32   = 0x5,
  BMI323_ACC_AVG_64   = 0x6,
};

enum BMI323_ACC_MODE : uint8_t {
  BMI323_ACC_MODE_DISABLED = 0x0,
  BMI323_ACC_MODE_CYCLING  = 0x3,
  BMI323_ACC_MODE_CONTINUOUS   = 0x4,
  BMI323_ACC_MODE_HP       = 0x7,
};

enum BMI323_GYR_ODR : uint8_t {
  BMI323_GYR_ODR_0_78HZ = 0x1,
  BMI323_GYR_ODR_1_56HZ = 0x2,
  BMI323_GYR_ODR_3_12HZ = 0x3,
  BMI323_GYR_ODR_6_25HZ = 0x4,
  BMI323_GYR_ODR_12_5HZ = 0x5,
  BMI323_GYR_ODR_25HZ   = 0x6,
  BMI323_GYR_ODR_50HZ   = 0x7,
  BMI323_GYR_ODR_100HZ  = 0x8,
  BMI323_GYR_ODR_200HZ  = 0x9,
  BMI323_GYR_ODR_400HZ  = 0xA,
  BMI323_GYR_ODR_800HZ  = 0xB,
  BMI323_GYR_ODR_1600HZ = 0xC,
  BMI323_GYR_ODR_3200HZ = 0xD,
  BMI323_GYR_ODR_6400HZ = 0xE,
};

enum BMI323_GYR_RANGE : uint8_t {
  BMI323_GYR_RANGE_125DPS  = 0x0,
  BMI323_GYR_RANGE_250DPS  = 0x1,
  BMI323_GYR_RANGE_500DPS  = 0x2,
  BMI323_GYR_RANGE_1000DPS = 0x3,
  BMI323_GYR_RANGE_2000DPS = 0x4,
};

enum BMI323_GYR_BW : uint8_t {
  BMI323_GYR_BW_ODR_2 = 0x0,
  BMI323_GYR_BW_ODR_4 = 0x1,
};

enum BMI323_GYR_AVGNUM : uint8_t {
  BMI323_GYR_AVG_NONE = 0x0,
  BMI323_GYR_AVG_2    = 0x1,
  BMI323_GYR_AVG_4    = 0x2,
  BMI323_GYR_AVG_8    = 0x3,
  BMI323_GYR_AVG_16   = 0x4,
  BMI323_GYR_AVG_32   = 0x5,
  BMI323_GYR_AVG_64   = 0x6,
};

enum BMI323_GYR_MODE : uint8_t {
  BMI323_GYR_MODE_DISABLED      = 0x0,
  BMI323_GYR_MODE_DRIVE_ENABLED = 0x1,
  BMI323_GYR_MODE_CYCLING       = 0x3,
  BMI323_GYR_MODE_CONTINUOUS    = 0x4,
  BMI323_GYR_MODE_HP            = 0x7,
};




class DevLab_BMI323 {

public:

  /*
    Structure containing all BMI323 sensor data
  */
  struct SensorData {
    int16_t accX;
    int16_t accY;
    int16_t accZ;

    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    int16_t tempRaw;
    float temperatureC;
  };

  struct acc_cfg {
    uint8_t acc_odr;
    uint8_t acc_range;
    uint8_t acc_bw;
    uint8_t acc_avgnum;
    uint8_t acc_mode;
  };

  struct gyr_cfg {
    uint8_t gyr_odr;
    uint8_t gyr_range;
    uint8_t gyr_bw;
    uint8_t gyr_avgnum;
    uint8_t gyr_mode;
  };

  struct int_ctrl{
    uint8_t int1_level;
    uint8_t int1_od;
    uint8_t int1_en;
    uint8_t int2_level;
    uint8_t int2_od;
    uint8_t int2_en;
  };


  /*
    Constructor I2C

    @param wire    I2C interface instance
    @param address I2C device address
  */
  DevLab_BMI323(TwoWire &wire = Wire, uint8_t address = 0x69);

  /*
    Constructor SPI

    @param spi    SPI interface instance
    @param csPin  Chip select pin
    @param speed  SPI clock frequency
  */
  DevLab_BMI323(SPIClass &spi, uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, uint32_t speed = 4000000);

  /*
    Initialize BMI323 sensor

    @param sdaPin  SDA pin
    @param sclPin  SCL pin
    @param clock   I2C clock frequency

    @return true if initialization succeeds
  */
  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock = 400000);


  /*
    Initialize BMI323 sensor with SPI

    @return true if initialization succeeds
  */
  bool begin();

  /*
    Configure accelerometer and gyroscope
  */
  void configure();

  void configureAccGyr(acc_cfg acc_Cfg, gyr_cfg gyr_Cfg);

  uint16_t configAccBMI323(acc_cfg acc_Cfg);

  uint16_t configGyrBMI323(gyr_cfg gyr_Cfg);

  /*
    【本工程新增】直接读回 GYR_CONF 寄存器原始值，用于开机核对量程是否真的设上了。
    为什么需要：库的 configure() 是"写进去就认为成功"，没有回读校验。
      本工程曾多次遇到"改了量程但实际没生效"，导致读数与 SCALE 差 4~16 倍，
      稳定器带着错的标度去猛摆，却完全看不出来。
    期望值：
      0x708B = ±125 dps（库的原始默认）
      0x70AB = ±500
      0x70BB = ±1000
      0x70CB = ±2000（本工程使用）
  */
  uint16_t readGyrConf();

  /*
    Read accelerometer, gyroscope and temperature data

    @param data Structure containing sensor values

    @return true if read succeeds
  */
  bool readData(SensorData &data);


  bool enableFeatEngine();

  uint8_t readErrorStatus(); 

  bool waitDrdyAcc(uint32_t timeoutMs);


  /*
    Perform BMI323 soft reset
  */
  void softReset();

  void test_chip_id(int expectedID, int regID);
  

  //INTERRUPTS FUNCTIONS
  void configINT(int_ctrl intCfg);

  void configIntLatch(uint8_t latch );          //0: pulso 1:latched

  void setINTMap1(BMI323_INT1_SRC src,BMI323_INT_DEST trgt);

  void setINTMap2(BMI323_INT2_SRC src,BMI323_INT_DEST trgt);

  void clearAllINTMap();

  uint16_t readINTStatus1();

  uint16_t readINTStatus2();

  uint16_t getINT1Map() const { return _int1MapShadow; }

  uint16_t getINT2Map() const { return _int2MapShadow; }



    /*
    Only for test delete after testing
    **/

  uint16_t testAddresses(uint8_t);

  void configAnyMotion();

protected:

    // ── Variables globales del checklist ──────────────────────
  struct ValidationResult {
    bool chip_detected   = false;
    bool no_fatal_error  = false;
    bool por_ok          = false;
    bool acc_configured  = false;
    bool gyr_configured  = false;
    bool acc_x_ok        = false;
    bool acc_y_ok        = false;
    bool acc_z_ok        = false;
    bool gyr_x_ok        = false;
    bool gyr_y_ok        = false;
    bool gyr_z_ok        = false;
    bool temp_ok         = false;
    bool stability_ok    = false;
  };

  //Declaraciones de validacion 
  ValidationResult vr;


  void print_separator();

  void print_pass(const char* test);

  void print_fail(const char* test);

  void print_warn(const char* test);



  
  
  
private:

  TwoWire *_wire = nullptr;
  uint8_t _address = 0x69; // Dirección I2C por defecto del BMI323

  SPIClass *_spi = nullptr;
  uint8_t _csPin = 0;
  uint8_t _misoPin = 0;
  uint8_t _mosiPin = 0;
  uint8_t _sckPin = 0;
  uint32_t _spiSpeed = 4000000U;

  bool _useI2C = true;
  uint16_t _int1MapShadow = 0x0000;
  uint16_t _int2MapShadow = 0x0000;
  /*
    Write 16-bit value to register
  */
  void writeRegister16(uint8_t reg, uint16_t value);

  /*
    Read 16-bit value from register
  */
  uint16_t readRegister16(uint8_t reg);

  void spiActivateMode();
  bool _spiAlive();



};

using BMI323_AccCfg          = DevLab_BMI323::acc_cfg;
using BMI323_GyrCfg          = DevLab_BMI323::gyr_cfg;
using BMI323_IntCtrl         = DevLab_BMI323::int_ctrl;
using BMI323_SensorData      = DevLab_BMI323::SensorData;


#endif
