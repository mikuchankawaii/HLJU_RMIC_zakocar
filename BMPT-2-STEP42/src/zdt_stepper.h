// =====================================================================
//  张大头 42 闭环步进电机（ZDT Emm42 V5.0）驱动层
//  协议依据：《Emm42_V5.0 通讯控制命令列表》6.3 节（官方案例）
//  —— 只负责“编帧、发指令、解回包”，不含炮塔/稳定器逻辑
// =====================================================================
//
//  接线（从机 BMPT-2）：
//      驱动器 RX  <- 从机 GPIO26 (TX)
//      驱动器 TX  -> 从机 GPIO25 (RX)
//      GND 共地；驱动器按手册单独供电
//      115200、8N1、校验固定 0x6B（手册：通讯校验方式固定为 0x6B）
//
//  【重要】地址默认 1（手册配置参数里 ID=0x01，可按需改）
//
#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <Arduino.h>

// ---------------- 接线与总线 ----------------
#define ZDT_RX_PIN        25      // 从机 RX <- 驱动器 TX
#define ZDT_TX_PIN        26      // 从机 TX -> 驱动器 RX
#define ZDT_BAUD          115200  // 手册：波特率选项 0x05 = 115200
#define ZDT_ADDR          1       // 电机 ID（手册默认 1）

// ---------------- 机械参数（手册确认值）----------------
// 手册：16 细分下 3200 脉冲 = 一圈；细分默认 0x10=16（256 细分时填 0x00）
#define ZDT_MICROSTEP      16
#define ZDT_MOTOR_STEP_DEG 1.8f   // 手册：电机类型 0x19=1.8°
#define ZDT_PULSES_PER_REV  ((int32_t)(360.0f / ZDT_MOTOR_STEP_DEG * ZDT_MICROSTEP))
// = 3200 脉冲/圈

// 位置到达窗口：手册默认 0x0001 = 0.1°（驱动器内部判定，我们另做自己的判定）
#define ZDT_ARRIVE_DEG     0.5f

// 加速度档位 0~255。手册：0 = 不使用曲线加减速。
//   曲线加减速时间 t2-t1 = (256-acc)*50us，Vt2 = Vt1 + 1RPM
//   【注意单位】这是【每变化 1 RPM】所需的时间，不是总时间！
//   实际延迟 = 每RPM时间 × 转速变化量：
//       acc=200 -> 2.8ms/RPM -> 0→17RPM(100dps) 要 48ms，反向 34RPM 要 95ms
//       acc=240 -> 0.8ms/RPM -> 0→17RPM 要 14ms，反向 27ms
//   【为什么从 200 改到 240 —— 实测结论】
//     炮塔稳定器是速度闭环：主机每 5ms 算一次角速度指令，从机照着跑。
//     95ms 的加减速延迟在 5Hz 处就是 171° 相位滞后，而速度环增益约 1.0
//     => 必然自激。表现就是日志里"瞬时速率 ±100 交替、但平均只有 23 dps"
//     的高频抖动（250ms 采样把它混叠成慢摆的假象）。
//     改到 240 后延迟 27ms，5Hz 相位滞后仅 49°，速度环增益 0.5 => 有裕度。
//   【代价必须知情】ramp 变硬更容易激起步进固有共振（10~20 RPM 附近）。
//     若出现机械嗡鸣，可在 220~240 之间折中；不建议再降回 200。
#define ZDT_DEFAULT_ACC    240

// =====================================================================
//  命令码与帧格式（全部来自手册 6.3 节，逐条对应）
// ---------------------------------------------------------------------
//  使能：      地址 + 0xF3 + 0xAB + 使能状态 + 多机同步标志 + 校验
//  速度模式：  地址 + 0xF6 + 方向 + 速度(2B) + 加速度 + 同步 + 校验
//  位置模式：  地址 + 0xFD + 方向 + 速度(2B) + 加速度 + 脉冲数(4B)
//                    + 相对/绝对 + 同步 + 校验
//  立即停止：  地址 + 0xFE + 0x98 + 同步 + 校验
//  位置清零：  地址 + 0x0A + 0x6D + 校验
//  解除堵转：  地址 + 0x0E + 0x52 + 校验
//  读固件版本：地址 + 0x1F + 校验
//  读实时位置：地址 + 0x36 + 校验  ->  地址 + 0x36 + 符号 + 位置(4B) + 校验
//  读实时转速：地址 + 0x35 + 校验  ->  地址 + 0x35 + 符号 + 转速(2B) + 校验
//  读电机状态：地址 + 0x3A + 校验  ->  地址 + 0x3A + 状态标志 + 校验
// =====================================================================
#define ZDT_CMD_ENABLE      0xF3
#define ZDT_ENABLE_SUB      0xAB
#define ZDT_CMD_VEL         0xF6
#define ZDT_CMD_POS         0xFD
#define ZDT_CMD_STOP        0xFE
#define ZDT_STOP_SUB        0x98
#define ZDT_CMD_ZERO        0x0A
#define ZDT_ZERO_SUB        0x6D
#define ZDT_CMD_CLEAR_PROT  0x0E
#define ZDT_CLEAR_SUB       0x52
#define ZDT_CMD_VERSION     0x1F
#define ZDT_CMD_READ_POS    0x36
#define ZDT_CMD_READ_SPEED  0x35
#define ZDT_CMD_READ_STATUS 0x3A

// 方向：手册「01 表示 CCW（00 表示 CW）」
// 【为什么需要这个符号宏】手册的 CW/CCW 是【电机出轴】的方向，
//   而它相对"炮塔左转/右转"还取决于电机怎么装、皮带怎么绕。
// 【只改这一处即可】翻方向就改这个值（1 / -1），
//   不要动主机的积分方向 —— 那会连带把摇杆方向也一起弄反。
//   本值同时作用于 movePulses 与 jog，所以点动测试方向会一起跟随，
//   这样"点动方向"和"稳定器方向"始终一致，排查时不会被误导。
#define ZDT_DIR_SIGN      (+1)
#define ZDT_DIR_CW   0x00
#define ZDT_DIR_CCW  0x01

// 位置模式 相对/绝对：手册「00 表示相对位置模式，01 表示绝对位置模式」
#define ZDT_POS_REL  0x00
#define ZDT_POS_ABS  0x01

// 多机同步标志：00 不启用 / 01 启用
#define ZDT_SYNC_OFF 0x00
#define ZDT_SYNC_ON  0x01

// 命令返回状态（手册：正确 0x02，条件不满足 0xE2）
#define ZDT_ST_OK    0x02
#define ZDT_ST_COND  0xE2

// =====================================================================
//  为什么要有“单圈”这个概念：
//  手册明确「0x36 回包的 4 字节位置，0-65535 表示一圈」，
//  即它是【单圈绝对角度】，转满一圈就绕回。所以：
//    - 驱动层提供 readSingleTurnDeg() 给出 -180~180 的单圈角度
//    - 多圈绝对位置由上层（从机 main.cpp）自己累加维护
//  这样上层才能知道“炮塔到底转了几圈”。
// =====================================================================
#define ZDT_COUNTS_PER_REV  65536.0f

class ZdtStepper {
public:
  void begin();
  bool beginAndEnable();

  bool enable(bool on);
  bool stop();
  bool clearProtection();
  bool zeroPosition();

  // 速度模式：rpm 转速、acc 加速度档位、dirCW 方向
  bool jog(uint16_t rpm, uint8_t acc, bool dirCW);
  // 位置模式：带符号脉冲数（正=CW）、rpm 限速、acc 加速度档位
  //   absMode=true  绝对位置模式（驱动器内部单圈坐标）
  //   absMode=false 相对位置模式（推荐：上层算好增量）
  bool movePulses(int32_t pulses, uint16_t rpm, uint8_t acc, bool absMode);
  // 【绝对位置模式】参数是【驱动器坐标系的绝对计数】(int32，可正可负)。
  //   与 movePulses 的区别：幂等、不会叠加、不需要上层猜跨了几圈。
  //   内部按手册 4 字节大端发送，相对/绝对标志位置 01。
  bool moveAbsCounts(int32_t absCounts, uint16_t rpm, uint8_t acc);

  // 读回包
  bool readRawPosition(uint16_t &raw);        // 0~65535 单圈原始值
  bool readSingleTurnDeg(float &deg);         // -180~180
  bool readSpeedRpm(int16_t &rpm);
  // 读状态位。返回 true = 【通讯成功】（不是"状态健康"）。
  //   位含义：bit0 使能 / bit1 到位 / bit2 堵转 / bit3 堵转保护
  //   调用方需自己检查位，例如 (flags & 0x0C) 表示堵转或堵转保护。
  bool readStatusFlags(uint8_t &flags);
  bool readVersion(uint8_t &fw, uint8_t &hw);

  // 诊断计数
  // 最近一次"不合法回包"的原始字节，用于诊断（[0]=长度，[1..]=内容）
  const uint8_t* lastBadFrame() const { return _lastBad; }
  uint32_t txCount()   const { return _txCnt; }
  uint32_t rxOkCount() const { return _rxOkCnt; }
  uint32_t rxErrCount() const { return _rxErrCnt; }
  uint32_t lastOkMs()  const { return _lastOkMs; }

private:
  void    sendFrame(const uint8_t *body, uint8_t bodyLen);
  // 读一条回包；会过滤掉首字节不是本站地址的杂帧。返回字节数，-1 = 超时
  int     readFrame(uint8_t *buf, uint8_t maxLen, uint16_t timeoutMs);
  static uint8_t fixedChecksum() { return 0x6B; }   // 手册：校验固定 0x6B

  uint32_t _txCnt = 0, _rxOkCnt = 0, _rxErrCnt = 0, _lastOkMs = 0;
  uint8_t  _lastBad[10] = {0};
  // ==================== 回显消除 ====================
  // 【为什么需要】单线半双工接法下，我们发出去的帧会回显到自己的 RX。
  //   而 readFrame 的过滤条件是"首字节不是 0x01 就丢"——
  //   可我们自己的帧也以 0x01(地址) 开头，于是回显【不会被丢掉】，
  //   反而被当成回包的开头：读位置发出 01 36 6B，收回来的就是这 3 个字节，
  //   长度不够 8 => 判定失败，真正的 8 字节回包随之被抛弃。
  //   实测表现：位置/状态回读绝大部分失败、偶尔成功、成功时还可能错位。
  // 【做法】sendFrame 记下刚发出的完整帧；readFrame 收齐后先剥掉这段前缀。
  //   没有回显时 memcmp 不匹配 => 行为与原来完全一致（零风险）。
  uint8_t  _lastTx[10] = {0};
  uint8_t  _lastTxLen  = 0;
};

extern ZdtStepper zdt;

#endif // ZDT_STEPPER_H
