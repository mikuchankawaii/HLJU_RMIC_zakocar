/*
  DevLab_BMI323.cpp

  Implementation file for the DevLab_BMI323 Arduino library.

  This library communicates with the Bosch BMI323 IMU using direct
  16-bit register access.

  Author:
  Adrian Rabadan Ortiz | Jonathan Mejorado 

  Organization:
  UNIT Electronics - DevLab Ecosystem

  License:
  MIT License
*/

#include "DevLab_BMI323.h"

// --------------------------------------------------
// BMI323 register definitions
// --------------------------------------------------
#define REG_ERR_REG       0x01
#define REG_STATUS        0x02
#define REG_ACC_X         0x03
#define REG_ACC_DATA_Y    0x04
#define REG_ACC_DATA_Z    0x05
#define REG_GYR_DATA_X    0x06
#define REG_GYR_DATA_Y    0x07
#define REG_GYR_DATA_Z    0x08
#define REG_TEMP_DATA     0x09
#define REG_FEATURE_IO0        0x10
#define REG_FEATURE_IO1        0x11
#define REG_FEATURE_IO2        0x12
#define REG_FEATURE_IO_STATUS  0x14
#define REG_GYR_CONF      0x21
#define REG_CMD           0x7E
#define REG_ACC_CONF      0x20




//Interrupts defines 
#define REG_INT_CTRL      0x38
#define REG_INT_CONF      0x39
#define REG_FEATURE_CTRL       0x40
#define REG_INT_STATUS1   0x0D
#define REG_INT_STATUS2   0x0E
#define REG_INT1_MAP      0x3A
#define REG_INT2_MAP      0x3B


#define SOFT_RESET_CMD 0xDEAF

#define SPI_READ_BIT  0x80
#define SPI_FAST_SPEED 4000000




// --------------------------------------------------
// Constructor
// --------------------------------------------------
DevLab_BMI323::DevLab_BMI323(TwoWire &wire, uint8_t address) {
  _wire = &wire;
  _address = address;
  _useI2C = true;
}

DevLab_BMI323::DevLab_BMI323(SPIClass &spi, uint8_t csPin,uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, uint32_t speed) {
  _spi = &spi;
  _csPin = csPin;
  _misoPin = misoPin;
  _mosiPin = mosiPin;
  _sckPin = sckPin;
  _spiSpeed = speed;
  _useI2C = false;
}


// --------------------------------------------------
// Initialize interfaces and configure BMI323
// --------------------------------------------------
bool DevLab_BMI323::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock)
{
  _wire->begin(sdaPin, sclPin);
  _wire->setClock(clock);
  delay(100);

  softReset();
  delay(100);
  configure();

  // return configure();

  uint16_t idRaw = readRegister16(REG_CHIP_ID);

  // Serial.print(">> DEBUG I2C - CHIP ID RAW DEVUELTO: 0x");
  // Serial.println(idRaw, HEX);

  return ((idRaw & 0xFF) == BMI323_CHIP_ID);
}

bool DevLab_BMI323::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH); // CS inactivo
  _spi->begin(_sckPin, _misoPin, _mosiPin, _csPin);

  delay(10);
  spiActivateMode();

  softReset();
  configure();
  return _spiAlive();
}
// --------------------------------------------------
// Configure accelerometer and gyroscope
// --------------------------------------------------
void DevLab_BMI323::configure() {
  /*
    ACC_CONF = 0x708B

    Mode      : High performance
    Average   : No averaging
    Filtering : ODR / 4
    Range     : ±2g
    ODR       : 800 Hz
  */
  writeRegister16(REG_ACC_CONF, 0x708B);
  delay(10);

  /*
    GYR_CONF = 0x70CB

    【本工程改动】原值为 0x708B（±125 dps），这里改为 ±2000 dps。
    【为什么】炮塔要抵消底盘转向：低速档约 342 °/s、高速档约 1080 °/s。
      陀螺测的是"炮塔在惯性系里的角速度" —— 稳定器正常时它≈0，
      但闭环一旦跟不上，炮塔被底盘拖着走时它【就等于底盘转速】。
      量程若小于这个值，陀螺立刻【削顶】成固定值，闭环彻底瞎眼、再也拉不回来。
      所以必须覆盖最坏情况 1080 °/s => 只有 ±2000 档够（可测到 ±2040）。

    【编码推导】GYR_CONF 位定义：
        bit0~3  = ODR（0xB = 800Hz）
        bit4~6  = 量程（0=±125, 1=±250, 2=±500, 3=±1000, 4=±2000）
        bit7    = 滤波带宽（1 = ODR/4）
        bit8~10 = 平均次数（0 = 不平均）
        bit12~14= 模式（7 = High performance）
      原值 0x708B 的量程字段 = 0（±125），改成 4 就是 4<<4 = 0x40：
        0x708B + 0x40 = 0x70CB

    【灵敏度（见数据表）】±2000 量程 = 16.384 LSB/(°/s)
      => SCALE = 1/16.384 = 0.0610352 dps/LSB
      （主机里 GYRO_SCALE 已同步为该值）

    【为什么改在这里、不能运行中改】
      configure() 由 begin() 在 softReset() 之后调用，此时传感器还在 suspend，
      改 ODR/量程是安全的；而"运行中直接写 GYR_CONF"实测会让陀螺输出变成
      死值（≈0 LSB）且不自愈 —— 这是本工程踩过的坑，不要再试。

    Mode      : High performance
    Average   : No averaging
    Filtering : ODR / 4
    Range     : ±2000 dps
    ODR       : 800 Hz
  */
  writeRegister16(REG_GYR_CONF, 0x70CB);
  delay(10);
}

void DevLab_BMI323::configureAccGyr(acc_cfg acc_Cfg, gyr_cfg gyr_Cfg){
  configAccBMI323(acc_Cfg);
  configGyrBMI323(gyr_Cfg);

  
}
uint16_t DevLab_BMI323::configAccBMI323(acc_cfg acc_Cfg){
  uint16_t configReg = ((uint16_t)acc_Cfg.acc_odr    << 0)  | 
                       ((uint16_t)acc_Cfg.acc_range  << 4)  | 
                       ((uint16_t)acc_Cfg.acc_bw     << 7)  | 
                       ((uint16_t)acc_Cfg.acc_avgnum << 8)  | 
                       ((uint16_t)acc_Cfg.acc_mode   << 12);
  writeRegister16(REG_ACC_CONF, configReg);
  //Verificar que el registro se haya escrito correctamente
  //Serial.print("Registro ACC_CONF armado: 0x");
  //Serial.println(configReg, HEX);
  delay(10);
  return configReg;
}


uint16_t DevLab_BMI323::configGyrBMI323(gyr_cfg gyr_Cfg){
  uint16_t configReg = ((uint16_t)gyr_Cfg.gyr_odr    << 0)  |
                       ((uint16_t)gyr_Cfg.gyr_range  << 4)  | 
                       ((uint16_t)gyr_Cfg.gyr_bw     << 7)  |   
                       ((uint16_t)gyr_Cfg.gyr_avgnum << 8)  | 
                       ((uint16_t)gyr_Cfg.gyr_mode   << 12);
  writeRegister16(REG_GYR_CONF, configReg);

  return configReg;
}

// 【本工程新增】回读 GYR_CONF，用于核对量程是否真的写进去了
uint16_t DevLab_BMI323::readGyrConf(){
  return readRegister16(REG_GYR_CONF);
}


// --------------------------------------------------
// Perform BMI323 soft reset
// --------------------------------------------------
void DevLab_BMI323::softReset() {
  writeRegister16(REG_CMD, SOFT_RESET_CMD);
  delay(50);
  _int1MapShadow = 0x0000;
  _int2MapShadow = 0x0000;
  if(!_useI2C){
    spiActivateMode();
  }

}

// --------------------------------------------------
// Read accelerometer, gyroscope and temperature data
// --------------------------------------------------
bool DevLab_BMI323::readData(SensorData &dataOut) {
  uint8_t buffer[16];
  uint8_t i = 0;
  uint8_t offset = 0;  // Skip dummy bytes
    
  if(_useI2C){
  _wire->beginTransmission(_address);
  _wire->write(REG_ACC_X);
  _wire->endTransmission(false);

  /*
    BMI323 I2C burst reads include 2 dummy bytes at the beginning.
    Requested bytes:
    - 2 dummy bytes
    - 6 bytes accelerometer XYZ
    - 6 bytes gyroscope XYZ
    - 2 bytes temperature
  */
  _wire->requestFrom(_address, (uint8_t)16);

 

  while (_wire->available() && i < 16) {
    buffer[i++] = _wire->read();
  }

  if (i < 16) {
    return false;
  }
  
    offset = 2;  // Skip dummy bytes
  }
  else{



    _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    _spi->transfer(REG_ACC_X | 0x80); // MSB=1 para lectura
    _spi->transfer(0x00); // Enviar byte dummy para iniciar lectura
    while(i < 14){

      buffer[i++] = _spi->transfer(0x00); // Enviar byte dummy para recibir datos
    }
    if (i < 14) {
    return false;
    }
    digitalWrite(_csPin, HIGH); // Terminar transacción
    _spi->endTransaction();
    delayMicroseconds(5);

   
  }


  dataOut.accX = (int16_t)(buffer[offset + 0]  | ((uint16_t)buffer[offset + 1]  << 8));
  dataOut.accY = (int16_t)(buffer[offset + 2]  | ((uint16_t)buffer[offset + 3]  << 8));
  dataOut.accZ = (int16_t)(buffer[offset + 4]  | ((uint16_t)buffer[offset + 5]  << 8));

  dataOut.gyrX = (int16_t)(buffer[offset + 6]  | ((uint16_t)buffer[offset + 7]  << 8));
  dataOut.gyrY = (int16_t)(buffer[offset + 8]  | ((uint16_t)buffer[offset + 9]  << 8));
  dataOut.gyrZ = (int16_t)(buffer[offset + 10] | ((uint16_t)buffer[offset + 11] << 8));

  dataOut.tempRaw = (int16_t)(buffer[offset + 12] | ((uint16_t)buffer[offset + 13] << 8));

  // BMI323 temperature conversion formula
  dataOut.temperatureC = (dataOut.tempRaw / 512.0f) + 23.0f;

  return true;
}

// --------------------------------------------------
// Write a 16-bit value to a BMI323 register
// --------------------------------------------------
void DevLab_BMI323::writeRegister16(uint8_t reg, uint16_t value) {
  if(_useI2C){
      _wire->beginTransmission(_address);
      _wire->write(reg);
      _wire->write(value & 0xFF);         // LSB
      _wire->write((value >> 8) & 0xFF);  // MSB
      _wire->endTransmission();
  }else{
    _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg & 0x7F);                   // MSB=0 para escritura
    _spi->transfer(value & 0xFF);                 // LSB
    _spi->transfer((value >> 8) & 0xFF);          // MSB
    digitalWrite(_csPin, HIGH);                   // Terminar transacción
    _spi->endTransaction();
    delayMicroseconds(5);
  }
}

// --------------------------------------------------
// Read a 16-bit value from a BMI323 register
// --------------------------------------------------
uint16_t DevLab_BMI323::readRegister16(uint8_t reg) {
  if(_useI2C){
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);// ── Variables globales del checklist ──────────────────────

    // 2 dummy bytes + 2 data bytes
    _wire->requestFrom(_address, (uint8_t)4);

    if (_wire->available() < 4) {
      return 0xFFFF;
    }

    _wire->read();  // Dummy byte 1
    _wire->read();  // Dummy byte 2

    uint8_t lsb = _wire->read();
    uint8_t msb = _wire->read();

    return ((uint16_t)msb << 8) | lsb;
  }
  else{
    _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80); // MSB=1 para lectura
    _spi->transfer(0x00); // Enviar byte dummy para iniciar lectura
    uint8_t lsb = _spi->transfer(0x00); // Enviar byte dummy para recibir LSB
    uint8_t msb = _spi->transfer(0x00); // Enviar byte dummy para recibir MSB
    digitalWrite(_csPin, HIGH); // Terminar transacción
    _spi->endTransaction();
    delayMicroseconds(5);

    return ((uint16_t)msb << 8) | lsb;
  }
}

// ── TEST 1: Detección del dispositivo y Chip ID ────────────
void DevLab_BMI323::test_chip_id(int expectedID, int regID) {
  if(_useI2C){
  Serial.println(F("\n[TEST 1] Deteccion del dispositivo (CHIP_ID)"));
  print_separator();

  uint16_t chip_id_raw = readRegister16(regID);  // REG_CHIP_ID
  uint8_t  chip_id     = (uint8_t)(chip_id_raw & 0xFF);  // solo bits [7:0] son validos

  Serial.print(F("  Registro 0x00 (raw 16b): 0x"));
  Serial.println(chip_id_raw, HEX);
  Serial.print(F("  CHIP_ID [7:0]: 0x"));
  Serial.print(chip_id, HEX);
  Serial.print(F("  (esperado: 0x"));
  Serial.print(expectedID, HEX);
  Serial.println(F(")"));

  if (chip_id == expectedID) {
    vr.chip_detected = true;
    print_pass("CHIP_ID correcto");
  } else {
    print_fail("CHIP_ID incorrecto — verificar conexiones y nivel logico");
  }

  }
  else{
      Serial.println(F("\n[TEST SPI] Verificar comunicación SPI con BMI323"));
  print_separator();

  // Escribir un valor de prueba en un registro (ejemplo: REG_ACC_CONF)
  uint16_t raw    = readRegister16(REG_CHIP_ID);
  uint8_t  chipId = (uint8_t)(raw & 0xFF);

  Serial.print(F("  raw 16b : 0x")); Serial.println(raw, HEX);
  Serial.print(F("  CHIP_ID : 0x")); Serial.print(chipId, HEX);
  Serial.print(F("  (esperado: 0x")); Serial.print(BMI323_CHIP_ID, HEX);

  Serial.println(F(")"));
  if      (chipId == BMI323_CHIP_ID) Serial.println(F("  [PASS] SPI OK"));
  else if (chipId == 0xFF)           Serial.println(F("  [FAIL] MISO sin conexion"));
  else if (chipId == 0x00)           Serial.println(F("  [FAIL] CS o alimentacion"));
  else {
    Serial.println(F("  [FAIL] CHIP_ID incorrecto"));
    Serial.println(F("  [WARN] Probar SPI_MODE3 si MODE0 falla"));
  }
  }
}

bool DevLab_BMI323::enableFeatEngine(){

  softReset();

  writeRegister16(REG_FEATURE_IO2, 0x12C);

  writeRegister16(REG_FEATURE_IO_STATUS,0x0001);

  writeRegister16(REG_FEATURE_CTRL, 0x0001);

    uint32_t t0 = millis();
  while (millis() - t0 < 100) {
    uint16_t io1 = readRegister16(REG_FEATURE_IO1);
    if (io1 & 0x0001) return true;   // error_status = 1 = engine listo
    delay(2);
  }
  return false;  // timeout — engine no respondió
}

uint8_t DevLab_BMI323::readErrorStatus()
{
    return (uint8_t)(readRegister16(REG_ERR_REG) & 0xFF);
}

bool DevLab_BMI323::waitDrdyAcc(uint32_t timeoutMs) {
  const uint32_t startTime = millis();
  while (millis() - startTime < timeoutMs) {
    if (readRegister16(REG_STATUS) & (1 << 7)) return true;
    delay(1);
  }
  return false;
  
}

/*************** INTERRUPT FUNCTIONS*******************/

void DevLab_BMI323::configINT(int_ctrl int_ctrlInt) {

  uint16_t intConf =   ((uint16_t)int_ctrlInt.int1_level    << 0)  | 
                       ((uint16_t)int_ctrlInt.int1_od       << 1)  | 
                       ((uint16_t)int_ctrlInt.int1_en       << 2)  | 
                       ((uint16_t)int_ctrlInt.int2_level    << 8)  | 
                       ((uint16_t)int_ctrlInt.int2_od       << 9)  |
                       ((uint16_t)int_ctrlInt.int2_en       << 10)  ;
  //Serial.print("IO_INT_CTRL escribiendo: 0x");
  //Serial.println(intConf, HEX);   // debe imprimir 0x505
  writeRegister16(REG_INT_CTRL, intConf);
}

void DevLab_BMI323::configIntLatch(uint8_t latch){
  writeRegister16(REG_INT_CONF, (uint16_t)(latch & 0x01));
}

void DevLab_BMI323::setINTMap1(BMI323_INT1_SRC src,BMI323_INT_DEST trgt) {
  _int1MapShadow &= ~((uint16_t)0x3 << src);    // limpiar campo de 2 bits
  _int1MapShadow |=  ((uint16_t)trgt << src);   // escribir destino
  writeRegister16(REG_INT1_MAP, _int1MapShadow);
  delayMicroseconds(5);
}
void DevLab_BMI323::setINTMap2(BMI323_INT2_SRC src,BMI323_INT_DEST trgt) {
  _int2MapShadow &= ~((uint16_t)0x3 << src);
  _int2MapShadow |=  ((uint16_t)trgt << src);
  writeRegister16(REG_INT2_MAP, _int2MapShadow);
  delayMicroseconds(5);
}

void DevLab_BMI323::clearAllINTMap() {
  _int1MapShadow = 0x0000;
  _int2MapShadow = 0x0000;
  writeRegister16(REG_INT1_MAP, 0x0000);
  delayMicroseconds(5);
  writeRegister16(REG_INT2_MAP, 0x0000);
  delayMicroseconds(5);
}

uint16_t DevLab_BMI323::readINTStatus1() {
  return readRegister16(REG_INT_STATUS1);
}

uint16_t DevLab_BMI323::readINTStatus2() {
  return readRegister16(REG_INT_STATUS2);
}

// ───────────────────────────────────────────────────────────
//  FUNCIONES DE VALIDACIÓN
// ───────────────────────────────────────────────────────────

void DevLab_BMI323::print_separator() {
  Serial.println(F("--------------------------------------------------"));
}

void DevLab_BMI323::print_pass(const char* test) {
  Serial.print(F("  [PASS] ")); Serial.println(test);
}

void DevLab_BMI323::print_fail(const char* test) {
  Serial.print(F("  [FAIL] ")); Serial.println(test);
}

void DevLab_BMI323::print_warn(const char* test) {
  Serial.print(F("  [WARN] ")); Serial.println(test);
}

uint16_t DevLab_BMI323::testAddresses(uint8_t reg){
  uint16_t featStatus = readRegister16(reg); // FEAT_ENG_STATUS
  Serial.print(F("FEAT_ENG_STATUS (0x14): 0x"));
  Serial.println(featStatus, HEX);

  return featStatus;

}


void DevLab_BMI323::configAnyMotion(){

   uint16_t io0 = readRegister16(REG_FEATURE_IO0);
  io0 |= (1 << 3) | (1 << 4) | (1 << 5);   // habilitar x, y, z
  writeRegister16(REG_FEATURE_IO0, io0);
  delayMicroseconds(5);

  // Activar la configuración en el feature engine
  writeRegister16(REG_FEATURE_IO_STATUS, 0x0001);
  delay(5);
}

/********************* SPI FUNCTIONS ********************************/

void DevLab_BMI323::spiActivateMode(){
  _spi->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_csPin,HIGH);
  _spi->endTransaction();
  delay(1);
  readRegister16(REG_CHIP_ID);
  delay(1);
}

bool DevLab_BMI323::_spiAlive() {
  uint16_t raw = readRegister16(REG_CHIP_ID);
  uint8_t chipId = (uint8_t)(raw & 0xFF);
  return (chipId == BMI323_CHIP_ID);
}
