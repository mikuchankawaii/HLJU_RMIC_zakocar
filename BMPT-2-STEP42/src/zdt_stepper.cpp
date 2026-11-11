// =====================================================================
//  张大头 42 闭环步进（ZDT Emm42 V5.0）驱动实现
//  每一帧的字节布局都按《Emm42_V5.0 通讯控制命令列表》6.3 节写，
//  并在注释里标注手册里的命令示例，方便逐条对照。
// =====================================================================
#include "zdt_stepper.h"

ZdtStepper zdt;

void ZdtStepper::begin() {
  Serial1.begin(ZDT_BAUD, SERIAL_8N1, ZDT_RX_PIN, ZDT_TX_PIN);
}

// 发送：body（含地址与功能码）+ 固定校验 0x6B
//   flush() 必须留：115200 下 10 字节约 0.9ms，不等它发完就读回包会错位。
void ZdtStepper::sendFrame(const uint8_t *body, uint8_t bodyLen) {
  // 【发送前清掉残留】单线半双工下，上一条指令（尤其 jog）的回显可能还堆在 RX 里。
  //   不清掉的话，接下来的 readFrame 会先读到那条旧回显，
  //   功能码对不上 => 本次回读直接失败，真正的回包也随之被抛弃。
  //   这里丢掉的是"上一条已过期"的数据，不会影响本次问答。
  while (Serial1.available()) (void)Serial1.read();

  Serial1.write(body, bodyLen);
  Serial1.write(fixedChecksum());
  Serial1.flush();
  _txCnt++;

  // 【记录刚发出的完整帧】供 readFrame 剥掉回显（单线半双工会把自己的帧听回来）。
  //   最多存 10 字节，够本工程所有帧（最长 9+1）。
  const uint8_t total = (uint8_t)(bodyLen + 1);
  _lastTxLen = (total <= (uint8_t)sizeof(_lastTx)) ? total : (uint8_t)sizeof(_lastTx);
  for (uint8_t i = 0; i < _lastTxLen; i++) {
    _lastTx[i] = (i < bodyLen) ? body[i] : fixedChecksum();
  }
}

int ZdtStepper::readFrame(uint8_t *buf, uint8_t maxLen, uint16_t timeoutMs) {
  const uint32_t t0 = millis();
  int echoRounds = 0;                      // 已经"只收到回显"的轮数

  while (millis() - t0 < timeoutMs) {
    uint8_t n = 0;

    // 1) 等首字节（必须是本站地址）
    //    【注意】这一条【无法】区分回显与真回包 —— 因为我们自己的帧也以地址开头。
    //    真正的回显消除放在下面第 3 步做（比对 _lastTx 前缀）。
    uint32_t tw = millis();
    while (millis() - tw < 6 && n == 0) {
      if (!Serial1.available()) continue;
      const uint8_t b = (uint8_t)Serial1.read();
      if (b != ZDT_ADDR) continue;
      buf[n++] = b;
    }
    if (n == 0) continue;                  // 这一轮没等到首字节，继续等（受总超时约束）

    // 2) 收齐后续字节：用"短静默"判定帧尾（帧长不固定）
    uint32_t t1 = millis();
    while (millis() - t1 < 4 && n < maxLen) {
      while (Serial1.available() && n < maxLen) {
        buf[n++] = (uint8_t)Serial1.read();
        t1 = millis();
      }
    }

    // 3) 【关键】剥掉自己刚发出去的回显
    //    单线半双工时我们发出的帧会回到 RX，且首字节同样是地址，
    //    原来它会被当成回包开头 => 长度不够 => 判失败 => 真回包被抛弃。
    //    没有回显时 memcmp 不匹配 => 与修改前完全一致。
    if (_lastTxLen > 0 && n >= _lastTxLen &&
        memcmp(buf, _lastTx, (size_t)_lastTxLen) == 0) {
      const uint8_t rest = (uint8_t)(n - _lastTxLen);
      if (rest > 0) memmove(buf, buf + _lastTxLen, (size_t)rest);
      n = rest;
      if (n == 0) {                        // 只收到回显，还没等到真回包
        if (++echoRounds >= 3) return -1;   // 连回显都没有下文 => 放弃
        continue;
      }
    }
    return (int)n;
  }
  return -1;
}


bool ZdtStepper::enable(bool on) {
  // 手册示例：使能 01 F3 AB 01 00 6B ；不使能 01 F3 AB 00 00 6B
  const uint8_t body[5] = { ZDT_ADDR, ZDT_CMD_ENABLE, ZDT_ENABLE_SUB,
                            (uint8_t)(on ? 0x01 : 0x00), ZDT_SYNC_OFF };
  sendFrame(body, 5);
  _lastOkMs = millis();
  return true;
}

bool ZdtStepper::beginAndEnable() {
  begin();
  delay(20);                     // 等驱动器上电就绪
  return enable(true);
}

bool ZdtStepper::stop() {
  // 手册示例：01 FE 98 00 6B（立即停止，速度/位置模式都可用）
  const uint8_t body[4] = { ZDT_ADDR, ZDT_CMD_STOP, ZDT_STOP_SUB, ZDT_SYNC_OFF };
  sendFrame(body, 4);
  _lastOkMs = millis();
  return true;
}

bool ZdtStepper::zeroPosition() {
  // 手册示例：01 0A 6D 6B —— 把当前位置角度、位置误差、脉冲数全部清零
  const uint8_t body[3] = { ZDT_ADDR, ZDT_CMD_ZERO, ZDT_ZERO_SUB };
  sendFrame(body, 3);
  _lastOkMs = millis();
  return true;
}

bool ZdtStepper::clearProtection() {
  // 手册示例：01 0E 52 6B —— 堵转后用它解除保护
  const uint8_t body[3] = { ZDT_ADDR, ZDT_CMD_CLEAR_PROT, ZDT_CLEAR_SUB };
  sendFrame(body, 3);
  _lastOkMs = millis();
  return true;
}

bool ZdtStepper::jog(uint16_t rpm, uint8_t acc, bool dirCW) {
  // 手册示例：01 F6 01 05 DC 0A 00 6B
  //   地址, 0xF6, 方向, 速度高, 速度低, 加速度, 同步, 校验
  // 【与 movePulses 用同一个符号修正】否则点动测试的方向会和稳定器的不一致，
  //   排查时会被误导。dirCW 表示"期望转向"，符号只落在输出字节上。
  const bool outCW = (dirCW == (ZDT_DIR_SIGN > 0));
  const uint8_t body[7] = {
    ZDT_ADDR, ZDT_CMD_VEL,
    (uint8_t)(outCW ? ZDT_DIR_CW : ZDT_DIR_CCW),
    (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
    acc, ZDT_SYNC_OFF
  };
  sendFrame(body, 7);
  _lastOkMs = millis();
  return true;
}

bool ZdtStepper::movePulses(int32_t pulses, uint16_t rpm, uint8_t acc, bool absMode) {
  // 手册示例：01 FD 01 05 DC 00 00 00 7D 00 00 00 6B
  //   地址, 0xFD, 方向, 速度高, 速度低, 加速度, 脉冲3, 脉冲2, 脉冲1, 脉冲0,
  //   相对/绝对, 同步, 校验
  // 【手册原文】01 表示 CCW（00 表示 CW）；00 表示相对位置模式（01 表示绝对）
  // 【符号修正只施加一次】ZDT_DIR_SIGN 见头文件说明（实测取 -1）。
  //   之前这里错了：变换既写进了 dirCW 又写进了输出字节，两次抵消 => 改了没用。
  //   现在改成：dirCW 表示"期望的转向"，符号只落在输出字节上。
  const bool dirCW = (pulses >= 0);                          // 期望转向
  const uint32_t mag = (uint32_t)(dirCW ? pulses : -pulses); // 幅值取正
  const bool outCW = (dirCW == (ZDT_DIR_SIGN > 0));          // 实际发给驱动器的方向
  const uint8_t body[12] = {
    ZDT_ADDR, ZDT_CMD_POS,
    (uint8_t)(outCW ? ZDT_DIR_CW : ZDT_DIR_CCW),
    (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
    acc,
    (uint8_t)((mag >> 24) & 0xFF), (uint8_t)((mag >> 16) & 0xFF),
    (uint8_t)((mag >> 8) & 0xFF),  (uint8_t)(mag & 0xFF),
    (uint8_t)(absMode ? ZDT_POS_ABS : ZDT_POS_REL),
    ZDT_SYNC_OFF
  };
  sendFrame(body, 12);
  _lastOkMs = millis();
  return true;
}

// 读实时位置：手册示例 01 36 6B -> 01 36 01 00 01 00 00 6B
//   回包：地址, 0x36, 符号(01=负/00=正), 位置4字节(大端, 0~65535 = 一圈), 校验
bool ZdtStepper::readRawPosition(uint16_t &raw) {
  const uint8_t body[2] = { ZDT_ADDR, ZDT_CMD_READ_POS };
  sendFrame(body, 2);

  uint8_t buf[16];
  const int n = readFrame(buf, sizeof(buf), 8);

  // ==================== 【关键】必须验证回包，否则乱码会被当成有效位置 ====================
  // 【为什么必须做】实测出现过：自检"读版本"打印了个 0x00/0xC8 的乱码，
  //   说明 readFrame 会把噪声当成回包。位置一旦读成乱码，累加角度就被污染，
  //   每次算出的增量都是巨大的假值 => 电机自己一直转。
  // 三条一起验：① 长度够 ② 功能码对 ③ 校验字节 = 0x6B
  if (n < 8 || buf[1] != ZDT_CMD_READ_POS || buf[7] != 0x6B) {
    _rxErrCnt++;
    _lastBad[0] = (uint8_t)n;
    for (int i = 0; i < 8 && i < n; i++) _lastBad[1 + i] = buf[i];
    return false;
  }

  const bool neg = (buf[2] == 0x01);
  uint32_t v = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
               ((uint32_t)buf[5] << 8)  |  (uint32_t)buf[6];
  raw = (uint16_t)(v & 0xFFFF);      // 手册：0-65535 表示一圈（单圈量）
  if (neg) raw = (uint16_t)(0u - raw);
  _rxOkCnt++; _lastOkMs = millis();
  return true;
}

bool ZdtStepper::readSingleTurnDeg(float &deg) {
  uint16_t raw;
  if (!readRawPosition(raw)) return false;
  deg = (float)raw * 360.0f / ZDT_COUNTS_PER_REV;   // 0~360
  if (deg > 180.0f) deg -= 360.0f;                 // 归到 -180~180
  return true;
}

// 读实时转速：手册示例 01 35 6B -> 01 35 01 05 DC 6B
//   回包：地址, 0x35, 符号(01=负), 转速2字节(大端), 校验
bool ZdtStepper::readSpeedRpm(int16_t &rpm) {
  const uint8_t body[2] = { ZDT_ADDR, ZDT_CMD_READ_SPEED };
  sendFrame(body, 2);

  uint8_t buf[12];
  const int n = readFrame(buf, sizeof(buf), 6);
  if (n < 6 || buf[1] != ZDT_CMD_READ_SPEED || buf[n - 1] != 0x6B) { _rxErrCnt++; return false; }

  const bool neg = (buf[2] == 0x01);
  uint16_t v = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];
  rpm = neg ? (int16_t)(-(int16_t)v) : (int16_t)v;
  _rxOkCnt++; _lastOkMs = millis();
  return true;
}

// 读电机状态标志：手册示例 01 3A 6B -> 01 3A 03 6B
//   位0 使能 | 位1 到位 | 位2 堵转 | 位3 堵转保护
bool ZdtStepper::readStatusFlags(uint8_t &flags) {
  const uint8_t body[2] = { ZDT_ADDR, ZDT_CMD_READ_STATUS };
  sendFrame(body, 2);

  uint8_t buf[12];
  const int n = readFrame(buf, sizeof(buf), 6);
  if (n < 4 || buf[1] != ZDT_CMD_READ_STATUS || buf[n - 1] != 0x6B) { _rxErrCnt++; return false; }

  flags = buf[2];
  _rxOkCnt++; _lastOkMs = millis();
  // 【语义修正】返回值只表示"这次读取是否成功"，不再掺入状态判断。
  // 【为什么改】原来返回 (flags & 0x08)==0，也就是"堵转保护置位时返回 false"。
  //   调用方无法区分"读失败"和"读到了但状态异常"，而我们要做的恰恰是
  //   "读到堵转保护 -> 去解除它"，用原来的语义会把这个分支整个跳过去。
  // 状态位含义（手册 0x3A）：
  //   bit0 使能   bit1 到位   bit2 堵转   bit3 堵转保护
  return true;
}

// 读固件版本：手册示例 01 1F 6B -> 01 1F 7D 6F 6B
bool ZdtStepper::readVersion(uint8_t &fw, uint8_t &hw) {
  const uint8_t body[2] = { ZDT_ADDR, ZDT_CMD_VERSION };
  sendFrame(body, 2);

  uint8_t buf[16];
  const int n = readFrame(buf, sizeof(buf), 8);
  // 校验：长度 + 功能码 + 校验字节
  // 【为什么必须验】实测出现过"读到 0x00/0xC8"这种明显不合理的版本号，
  //   说明没有校验时噪声会被当成有效回包 —— 那位置读数也会被同样污染。
  if (n < 5 || buf[1] != ZDT_CMD_VERSION || buf[n - 1] != 0x6B) {
    _rxErrCnt++;
    _lastBad[0] = (uint8_t)n;
    for (int i = 0; i < 8 && i < n; i++) _lastBad[1 + i] = buf[i];
    return false;
  }

  fw = buf[2]; hw = buf[3];
  _rxOkCnt++; _lastOkMs = millis();
  return true;
}

// ==================== 绝对位置模式 ====================
// 手册位置模式帧：地址 + 0xFD + 方向 + 速度(2B) + 加速度 + 脉冲数(4B)
//                  + 相对/绝对 + 同步 + 校验
//   区别只在【相对/绝对字节】和【脉冲数的含义】：
//     相对：脉冲数 = 要走多少（会叠加）
//     绝对：脉冲数 = 要走到哪个坐标（幂等，不走就不动）
// 【为什么必须支持它】相对模式在"上一段没走完又发下一段"时会叠加，
//   实测出现单次 49000 脉冲（=5500°）、目标涨到 2825° 的飞车。
bool ZdtStepper::moveAbsCounts(int32_t absCounts, uint16_t rpm, uint8_t acc) {
  const bool dirCW = (absCounts >= 0);
  const uint32_t mag = (uint32_t)(dirCW ? absCounts : -absCounts);
  const bool outCW = (dirCW == (ZDT_DIR_SIGN > 0));
  const uint8_t body[12] = {
    ZDT_ADDR, ZDT_CMD_POS,
    (uint8_t)(outCW ? ZDT_DIR_CW : ZDT_DIR_CCW),
    (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
    acc,
    (uint8_t)((mag >> 24) & 0xFF), (uint8_t)((mag >> 16) & 0xFF),
    (uint8_t)((mag >> 8) & 0xFF),  (uint8_t)(mag & 0xFF),
    ZDT_POS_ABS,                       // 【绝对模式】
    ZDT_SYNC_OFF
  };
  sendFrame(body, 12);
  _lastOkMs = millis();
  return true;
}