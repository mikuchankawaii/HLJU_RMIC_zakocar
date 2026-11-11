/*
                     !
              n     /N
         /@]+-]_Ω==Ω_[-+[@\
        [@]/   -=HH=-   \[@]             哒哒哒哒哒哒哒哒..... 咻咻~....哒哒哒哒哒哒哒哒哒哒哒哒哒哒哒哒.......... 咻咻~
             /=====【】\
    {MN==|==[==]===+=[==]==|==NM}
   {MN=/[__][__][___][___][__]\=NM}
   {MN/+[__][__][___][___][__]+\NM}
   {MN|WWWW|++__________++|WWWW|NM}
      |WWWW|              |WWWW|
*/

/* 从机喵 */
#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// 从机干的活 收主机的包 驱动炮塔步进电机和底盘四个轮子 主机一断自己停车
#define USE_STEPPER   1

// 多久没收到包就当主机没了 1.5 秒大约 75 个包的时间
// 太短会被主机卡顿误判 太长主机真死了车还在跑
#define LINK_LOST_MS  1500UL
#define LINK_ZERO_MS   200UL

static volatile unsigned long g_lastPktMs = 0;
static volatile uint32_t      g_rxPkts    = 0;
static int16_t g_pktTarget01 = 0;

static uint8_t g_pktMaxSpeedDps   = 20;
#if USE_STEPPER
#include "zdt_stepper.h"
#endif
#include <ESP32Servo.h>

#define YAW_SERVO_PIN   27
#define MOTOR_RX_PIN    16
#define MOTOR_TX_PIN    17
#define MOTOR_BAUD      115200

#define REVERSE_FWD_BWD 0

#define REVERSE_TURN    1

// 四个轮子的正反 哪个装反就翻哪个
#define INV_FL 1
#define INV_FR 0
#define INV_RL 1
#define INV_RR 1

// 左右微调 只在原地转向时用
// 直行和平移用下面那两个 1.0 保持左右对称 不然车会走偏
#define TRIM_LEFT  1.10f
#define TRIM_RIGHT 1.00f

#define TRIM_LEFT_STRAIGHT   1.0f
#define TRIM_RIGHT_STRAIGHT  1.0f

static volatile bool g_pktIsTurn = false;

// 主机用 'M' 标出「低速档 自转+平移」的混合包
// 这种包的四个轮值是按「平移+自转」的配比算好的，左右微调主机已经套过
// 从机这边不能再套微调，也不能打软启动斜坡（斜坡会把最慢那只轮子压回死区）
static volatile bool g_pktIsMix = false;

#define YAW_CENTER_US    1500
#define YAW_SPAN_POS_US   500
#define YAW_SPAN_NEG_US   500
#define YAW_MIN           0
#define YAW_MAX         180
#define YAW_CENTER       90

#define YAW_SOFTSTOP_HOLD_MS  200

// 底盘 PWM 多久重发一次 驱动板丢包会一直保持上一次的值 所以要反复发
#define MOTOR_RESEND_MS 250

// 舵机模式下多久没新指令就回中 步进模式不用这条
#define YAW_TIMEOUT_MS  500

#define YAW_RAMP_STEP_MS 5
#define YAW_RAMP_DIV     8

#define MOTOR_MODE_310   2

#define RAMP_RISE_PCT_S    200.0f

#define RAMP_START_PWM     0.50f

#define LOOP_DELAY_MS      2

const char*    WIFI_SSID   = "BMPT-90Terminator3";
const char*    WIFI_PASS   = "targetdestory";
const char*    MASTER_IP   = "192.168.4.1";

// 固定 IP 省掉 DHCP：拿不到地址 / 续租失败是断链最常见的来源之一
#define SLAVE_STATIC_IP 1
const uint16_t MASTER_PORT = 3333;

typedef struct __attribute__((packed)) {
  char    chassis;
  uint8_t yaw;

  int16_t fl;
  int16_t fr;
  int16_t rl;
  int16_t rr;

  int16_t targetAngle01deg;
  uint8_t maxSpeedDps;

} CtrlPacket;

// 往主机回的遥测包 关掉主机就看不到炮塔实际转速了
#define STEP_TELEMETRY_EN    1
#define STEP_TELEMETRY_MS  50
#define STEP_TELE_MAGIC0  0xA5
#define STEP_TELE_MAGIC1  0x5A
#define STEP_TELE_TYPE    0x01
typedef struct __attribute__((packed)) {
  uint8_t  m0, m1;
  uint8_t  type;
  int16_t  dpsCmd10;
  int16_t  dpsOut10;
  int16_t  dpsLimit;
  uint16_t rpm;
  uint8_t  dir;
  uint16_t stallClears;
  uint16_t rxErr;
  int16_t  rpmReal;
  uint16_t rawPos;
  uint32_t ms;
  uint8_t  sum;
} StepTelemetry;

static float    g_tmDpsCmd = 0.0f;
static float    g_tmDpsOut = 0.0f;
static float    g_tmLimit  = 0.0f;
static uint16_t g_tmRpm    = 0;
static bool     g_tmDirCW  = true;
static uint16_t g_tmStall  = 0;
static uint16_t g_tmRaw    = 0;
static uint32_t g_tmSent   = 0;
static uint32_t g_tmSkip   = 0;
static uint32_t g_tmFail   = 0;

static float         s_rampFrac    = 0.0f;
static unsigned long g_rampLastMs  = 0;

void softRampReset() {
  s_rampFrac   = 0.0f;
  g_rampLastMs = 0;
}

// 起步软一点 四个轮子同时满占空会顿一下
// 混合包（'M'）不打斜坡：四轮是配比出来的，坡一起步就会把最慢那只轮子压回死区，
// 自转直接没了，看起来就是"按下去先直着蹭一下"
void softRampApply(float &fFL, float &fFR, float &fRL, float &fRR) {
  if (g_pktIsMix) return;

  float tgt[4] = {fFL, fFR, fRL, fRR};
  float mx = fmaxf(fmaxf(fabs(tgt[0]), fabs(tgt[1])), fmaxf(fabs(tgt[2]), fabs(tgt[3])));
  if (mx < 1e-4f) {
    // 完全停车才复位 下一次起步重新走斜坡
    // 以前是最凶那只轮子的符号一变就复位 混合时来回按方向键等于一路重新起步 车一顿一顿的
    s_rampFrac   = 0.0f;
    g_rampLastMs = millis();
    return;
  }

  unsigned long now = millis();
  float dt = (g_rampLastMs == 0) ? 0.0f : (float)(now - g_rampLastMs);
  g_rampLastMs = now;
  if (dt > 200.0f) dt = 200.0f;

  if (s_rampFrac < 1.0f) {
    s_rampFrac += RAMP_RISE_PCT_S * dt / 100000.0f;
    if (s_rampFrac > 1.0f) s_rampFrac = 1.0f;
  }

  float out[4];
  for (int i = 0; i < 4; i++) {
    float a = fabs(tgt[i]);
    if (a < 1e-4f) { out[i] = 0.0f; continue; }
    float start = RAMP_START_PWM;
    if (start > a) start = a;
    float s = start + (a - start) * s_rampFrac;
    out[i] = (tgt[i] > 0) ? s : -s;
  }

  fFL = out[0]; fFR = out[1]; fRL = out[2]; fRR = out[3];
}

#if !USE_STEPPER

Servo yawServo;
#endif
WiFiClient tcpClient;

#if !USE_STEPPER
volatile int  g_yawAngle     = YAW_CENTER;
volatile bool g_yawDirty     = false;

int           g_yawCurrentUs   = YAW_CENTER_US;
unsigned long g_yawLastStepMs  = 0;
bool          g_yawSoftStop    = false;

unsigned long g_yawCenterSinceMs = 0;
#endif

unsigned long g_lastYawRecvMs = 0;
unsigned long lastMotorSend  = 0;

uint8_t g_mac[6] = {0};
int16_t g_mFL = 0, g_mFR = 0, g_mRL = 0, g_mRR = 0;

uint8_t rxBuf[sizeof(CtrlPacket)];
int     rxLen = 0;

#if !USE_STEPPER

#define YAW_US_MIN  1000
#define YAW_US_MAX  2000
int yawToMicroseconds(int yaw) {
  int delta = yaw - YAW_CENTER;
  int us;
  if (delta >= 0) us = YAW_CENTER_US + delta * YAW_SPAN_POS_US / 90;
  else            us = YAW_CENTER_US + delta * YAW_SPAN_NEG_US / 90;
  if (us < YAW_US_MIN) us = YAW_US_MIN;
  if (us > YAW_US_MAX) us = YAW_US_MAX;
  return us;
}

void writeYawImmediate(int yaw) {
  int us = yawToMicroseconds(yaw);
  g_yawCurrentUs = us;
  g_yawSoftStop  = false;
  yawServo.writeMicroseconds(us);
}

void beginSoftStop() {
  g_yawSoftStop   = true;
  g_yawLastStepMs = millis();
}
#endif

static bool          s_mbReady      = false;
static bool          s_mbMotorDirty = false;
static unsigned long s_mbLastMotorMs = 0;
static float         s_mbPend[4]    = {0, 0, 0, 0};

void motBusWriteRaw(const char* s) { Serial2.print(s); }

void motBusSendPwm(float a, float b, float c, float d) {
  s_mbPend[0] = a; s_mbPend[1] = b; s_mbPend[2] = c; s_mbPend[3] = d;
  s_mbMotorDirty = true;
}

static void motBusFlush() {
  if (!s_mbReady || !s_mbMotorDirty) return;
  char buf[80];
  snprintf(buf, sizeof(buf), "$Car_Pwm:%.1f,%.1f,%.1f,%.1f!",
           s_mbPend[0], s_mbPend[1], s_mbPend[2], s_mbPend[3]);
  motBusWriteRaw(buf);
  s_mbMotorDirty  = false;
  s_mbLastMotorMs = millis();
}

// 底盘串口发送 只在值变了或者超时才发 别一直刷
void motBusPump() {
  if (!s_mbReady) return;
  unsigned long now = millis();
  if (s_mbMotorDirty && now - s_mbLastMotorMs >= 5) motBusFlush();
}

// 四轮 PWM 最后一道加工 单轮方向 左右微调 总上限 软启动 然后交给总线
void sendMotorRaw(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {

  if (INV_FL) fl = -fl;
  if (INV_FR) fr = -fr;
  if (INV_RL) rl = -rl;
  if (INV_RR) rr = -rr;

  const float kL = g_pktIsTurn ? TRIM_LEFT  : TRIM_LEFT_STRAIGHT;
  const float kR = g_pktIsTurn ? TRIM_RIGHT : TRIM_RIGHT_STRAIGHT;
  float fFL = (float)fl * kL;
  float fRL = (float)rl * kL;
  float fFR = (float)fr * kR;
  float fRR = (float)rr * kR;

#define MAX_OUT_PERCENT  40.0f
  fFL = constrain(fFL, -255, 255) * MAX_OUT_PERCENT / 255.0f;
  fFR = constrain(fFR, -255, 255) * MAX_OUT_PERCENT / 255.0f;
  fRL = constrain(fRL, -255, 255) * MAX_OUT_PERCENT / 255.0f;
  fRR = constrain(fRR, -255, 255) * MAX_OUT_PERCENT / 255.0f;

  softRampApply(fFL, fFR, fRL, fRR);

  motBusSendPwm(fFL, fFR, fRL, fRR);
}

void setMotorType(uint8_t type) {
  char buf[40];
  snprintf(buf, sizeof(buf), "$MOTOR_4CH_SET:%u!", type);
  motBusWriteRaw(buf);
}

void setMotors(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  g_mFL = fl; g_mFR = fr; g_mRL = rl; g_mRR = rr;
  sendMotorRaw(fl, fr, rl, rr);
  lastMotorSend = millis();
}

#if USE_STEPPER

static bool     s_inited        = false;
static bool     s_haveRef       = false;
static uint16_t s_rawRef        = 0;
static float    s_turnDeg       = 0.0f;
static uint16_t s_raw           = 0;
static uint16_t s_rpm           = 27;

static int32_t  s_lastPulses    = 0;
static uint32_t s_lastSendMs    = 0;
static uint32_t s_lastReadMs    = 0;
static uint32_t s_lastTraceMs   = 0;
static uint16_t s_stallClears   = 0;
static uint16_t s_pollFail      = 0;
static bool     s_lastDirCW     = true;

// 度每秒换成 RPM 步进的速度字段是整数转每分 一转每分就是六度每秒
// 所以最慢只能到六度每秒 再慢就没有档位了
static inline uint16_t dpsToRpm(float dps) {
  if (dps < 0) dps = -dps;
  uint16_t r = (uint16_t)(dps / 6.0f + 0.5f);
  if (r < 1) r = 1;
  return r;
}

static float g_motorDpsActual = 0.0f;
static int32_t  s_prevRawPos  = -1;
static unsigned long s_prevRawMs = 0;

static bool stepReadPos() {
  uint16_t rawNow = 0;
  if (!zdt.readRawPosition(rawNow)) { s_pollFail++; return false; }

  const unsigned long nowMs = millis();
  if (s_prevRawPos >= 0) {
    int32_t d = (int32_t)rawNow - s_prevRawPos;
    if (d >  32768) d -= 65536;
    if (d < -32768) d += 65536;
    const unsigned long dtMs = nowMs - s_prevRawMs;
    if (dtMs >= 50 && dtMs <= 2000) {
      g_motorDpsActual = (float)d * 360000.0f / (65536.0f * (float)dtMs);
    }
  }
  s_prevRawPos = (int32_t)rawNow;
  s_prevRawMs  = nowMs;

  s_raw     = rawNow;
  s_turnDeg = (float)rawNow * 360.0f / ZDT_COUNTS_PER_REV;
  if (s_turnDeg > 180.0f) s_turnDeg -= 360.0f;
  s_pollFail = 0;
  return true;
}

static void stepJogPoll() {
  while (Serial.available()) {
    const int ch = Serial.read();
    switch (ch) {
      case 'a': case 'd': {
        const bool cw = (ch == 'a');
        const uint16_t rpm = dpsToRpm(30);
        zdt.jog(rpm, ZDT_DEFAULT_ACC, cw);
        Serial.printf("[JOG] %s %u RPM\n", cw ? "正转" : "反转", rpm);
        break;
      }
      case 's': case 'A':
        zdt.stop();
        Serial.println("[JOG] 停");
        break;
      case 'p': {
        const bool ok = stepReadPos();
        Serial.printf("[JOG] 位置=%s 单圈=%.2f° raw=%u\n",
                      ok ? "OK" : "读失败", s_turnDeg, s_raw);
        break;
      }
      case '?': {
        float d = 0; uint8_t fl = 0; int16_t sp = 0;
        zdt.readSingleTurnDeg(d);
        zdt.readStatusFlags(fl);
        zdt.readSpeedRpm(sp);
        Serial.println(F("---------- [STEP] 诊断 ----------"));
        Serial.printf("  通讯  : 发送%lu次 成功%lu次 失败%lu次\n",
                      (unsigned long)zdt.txCount(), (unsigned long)zdt.rxOkCount(),
                      (unsigned long)zdt.rxErrCount());
        Serial.printf("  位置  : raw=%u 单圈=%.2f°  本次会话采样失败=%u\n",
                      s_raw, s_turnDeg, s_pollFail);
        Serial.printf("  状态  : 0x%02X 使能=%d 到位=%d 堵转=%d 堵转保护=%d 转速=%d RPM\n",
                      fl, fl & 1, (fl >> 1) & 1, (fl >> 2) & 1, (fl >> 3) & 1, sp);
        Serial.printf("  指令  : 主机给的角速度=%+.2f °/s   实发RPM=%u\n",
                      g_pktTarget01 / 10.0f, s_rpm);
        Serial.printf("  输出  : 限速RPM=%u 上次脉冲=%ld DIR_SIGN=%d 基准raw=%u\n",
                      s_rpm, (long)s_lastPulses, (int)ZDT_DIR_SIGN, s_rawRef);
        Serial.println(F("---------------------------------"));
        break;
      }
      default: break;
    }
  }
}

// 把主机的角速度指令变成步进转速
// 关键 指令只要超过一点点噪声就给最小档 1 转每分 不能抹成 0
// 抹成 0 的话小误差永远推不动 表现就是一直差着几度回不来
void applyStepperTarget() {
  const uint32_t nowMs = millis();

  if (!s_inited) {
    zdt.begin();
    delay(20);
    zdt.enable(true);
    delay(80);
    Serial.println("[STEP] 开始自检 ...");
    for (int i = 0; i < 10 && !s_haveRef; i++) { if (stepReadPos()) s_haveRef = true; else delay(20); }
    if (s_haveRef) {
      s_rawRef = s_raw;
      Serial.printf("[STEP] ✓ 位置读取正常  单圈=%.2f° raw=%u（仅诊断用，不参与控制）\n",
                    s_turnDeg, s_raw);
    } else {
      Serial.println("[STEP] ✗ 位置读取失败 —— 请检查 TX/RX 交叉、共地、ID、波特率、校验0x6B");
    }
    s_inited = true;
    return;
  }

  if (nowMs - s_lastReadMs >= 40) { s_lastReadMs = nowMs; stepReadPos(); }

  const float dpsCmd = (float)g_pktTarget01 / 10.0f;
  const float dpsLimit = (float)g_pktMaxSpeedDps * 8.0f;

  float dps = dpsCmd;
  if (dps >  dpsLimit) dps =  dpsLimit;
  if (dps < -dpsLimit) dps = -dpsLimit;

  const float EPS_DPS = 0.5f;
  int  newRpmMag;
  if (fabsf(dps) < EPS_DPS) {
    newRpmMag = 0;
  } else {
    newRpmMag = (int)(fabsf(dps) / 6.0f + 0.5f);
    if (newRpmMag < 1) newRpmMag = 1;
  }

  const uint8_t accForCmd = (newRpmMag >= 20) ? 250 : ZDT_DEFAULT_ACC;
  const bool newDirCW  = (dps >= 0);

  if (newRpmMag < 1) {
    const bool stateChanged = (s_rpm != 0) || (s_lastDirCW != newDirCW);
    if (!stateChanged && (nowMs - s_lastSendMs < 500)) return;
    s_rpm        = 0;
    s_lastDirCW  = newDirCW;
    s_lastPulses = 0;
    zdt.stop();
    s_lastSendMs = nowMs;
  } else {

    const bool stateChanged = (newRpmMag != (int)s_rpm) || (newDirCW != s_lastDirCW);
    const bool needRefresh  = (nowMs - s_lastSendMs >= 300);

    if (stateChanged || needRefresh) {

      static unsigned long s_lastStatusMs = 0;
      const bool statusDue = (nowMs - s_lastStatusMs >= 300);
      uint8_t fl = 0;
      if (statusDue) {
        s_lastStatusMs = nowMs;
        if (zdt.readStatusFlags(fl) && (fl & 0x0C)) {

          s_stallClears++;
          zdt.clearProtection();
          Serial.printf("[STEP] ⚠ 驱动器报堵转/堵转保护(0x%02X)，已解除并重发 #%u\n",
                        fl, (unsigned)s_stallClears);
        }
      }
      s_lastDirCW  = newDirCW;
      s_rpm        = (uint16_t)newRpmMag;
      s_lastPulses = (int32_t)dps;
      zdt.jog(s_rpm, accForCmd, newDirCW);
      s_lastSendMs = nowMs;
    }
  }

  g_tmDpsCmd = dpsCmd;
  g_tmDpsOut = dps;
  g_tmLimit  = dpsLimit;
  g_tmRpm    = s_rpm;
  g_tmDirCW  = s_lastDirCW;
  g_tmStall  = s_stallClears;

  if (nowMs - s_lastTraceMs >= 500) {
    s_lastTraceMs = nowMs;

    g_tmRaw = s_raw;
    Serial.printf("[STEP] 主机速度=%7.1f°/s 实发=%7.1f°/s 限速=%6.1f RPM令=%3u 电机实=%7.1f acc=%u 位读失=%u raw=%u %s\n",
                  dpsCmd, dps, dpsLimit, (unsigned)s_rpm,
                  g_motorDpsActual,
                  (unsigned)accForCmd, (unsigned)s_pollFail, s_raw,
                  newDirCW ? "正" : "反");
  }
}
#endif

// 收到一个控制包 就更新一遍目标
void handleCtrlPacket(const CtrlPacket& pkt) {

  g_pktIsTurn = (pkt.chassis == 'T');
  g_pktIsMix  = (pkt.chassis == 'M');

  if (pkt.fl == 0 && pkt.fr == 0 && pkt.rl == 0 && pkt.rr == 0) {
    softRampReset();
    setMotors(0, 0, 0, 0);
  } else {
    setMotors(pkt.fl, pkt.fr, pkt.rl, pkt.rr);
  }

  g_pktTarget01     = pkt.targetAngle01deg;
  g_pktMaxSpeedDps  = pkt.maxSpeedDps;

#if !USE_STEPPER

  int yaw = constrain(pkt.yaw, YAW_MIN, YAW_MAX);
  if (yaw != g_yawAngle) {
    g_yawAngle = yaw;
    g_yawDirty = true;
  }
#endif
  g_lastYawRecvMs = millis();
  g_lastPktMs     = millis();
  g_rxPkts++;
}

// ==================== 链路实时判定 + 自愈 ====================
// 从机在车上 拉不到手 所以断了必须自己爬起来 不能等人拔电
// 节奏：TCP 每 500ms 重连一次（单次最多阻塞 250ms，绝不能把主循环卡住）
//       断了 8 秒 => 把 WiFi 栈整个重来
//       断了 30 秒且之前通过 => 直接 ESP.restart()
#define LINK_TCP_RETRY_MS      500UL
#define LINK_TCP_TIMEOUT_MS    250
#define LINK_WIFI_RESET_MS    8000UL
#define LINK_CHIP_RESET_MS   30000UL
#define LINK_WR_ERR_MAX          20

static uint32_t      g_linkConnects  = 0;
static uint32_t      g_linkDrops     = 0;
static uint32_t      g_linkSelfResets = 0;
static unsigned long g_linkDownSince = 0;
static bool          g_linkEverUp    = false;
static bool          g_linkWifiReset = false;

// 掉线重连 主机硬复位不发 FIN socket 会变成半开 所以不能只看 connected
// 还要看多久没收到包 超过 LINK_LOST_MS 就主动 stop 逼它重连
void maintainConnection() {
  const unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {

    if (tcpClient.connected()) tcpClient.stop();
    if (g_linkDownSince == 0) {
      g_linkDownSince = now;
      g_linkDrops++;
      Serial.printf("[LINK] ✗ WiFi 掉线 #%lu\n", (unsigned long)g_linkDrops);
    }

    static unsigned long lastWifiTry = 0;
    if (now - lastWifiTry >= 5000UL) {
      lastWifiTry = now;
      WiFi.disconnect(true);
      delay(10);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

  } else {

    if (!tcpClient.connected()) {

      if (g_linkDownSince == 0) {
        g_linkDownSince = now;
        g_linkDrops++;
        Serial.printf("[LINK] ✗ TCP 断开 #%lu\n", (unsigned long)g_linkDrops);
      }

      static unsigned long lastTcpTry = 0;
      if (now - lastTcpTry >= LINK_TCP_RETRY_MS) {
        lastTcpTry = now;

        if (tcpClient.connect(MASTER_IP, MASTER_PORT, LINK_TCP_TIMEOUT_MS)) {
          tcpClient.setNoDelay(true);
          g_linkConnects++;
          g_linkEverUp    = true;
          g_linkWifiReset = false;
          if (g_linkDownSince != 0) {
            Serial.printf("[LINK] ✓ 重连成功 #%lu（断了 %lu ms）\n",
                          (unsigned long)g_linkConnects, now - g_linkDownSince);
          } else {
            Serial.printf("[LINK] ✓ 连接成功 #%lu\n", (unsigned long)g_linkConnects);
          }
          g_linkDownSince = 0;
          g_lastPktMs     = now;
        }
      }

    } else if (g_linkDownSince != 0) {

      g_linkDownSince = 0;
      Serial.println("[LINK] ✓ 链路恢复（socket 仍在）");
    }
  }

  // 断到一定程度就升级处理：先重开 WiFi，再不行自己重启
  if (g_linkDownSince != 0) {
    const unsigned long down = now - g_linkDownSince;

    if (down >= LINK_CHIP_RESET_MS && g_linkEverUp) {
      Serial.printf("[LINK] ✗ 断了 %lu ms 还没回来 => 自愈重启（第 %lu 次）\n",
                    down, (unsigned long)(g_linkSelfResets + 1));
      Serial.flush();
      delay(50);
      ESP.restart();
    }

    if (down >= LINK_WIFI_RESET_MS && !g_linkWifiReset) {
      g_linkWifiReset = true;
      Serial.printf("[LINK] ⚠ 断了 %lu ms => 重开 WiFi 栈\n", down);
      WiFi.disconnect(true);
      delay(20);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
}

// 开机 串口 电机总线 WiFi 连 AP TCP 连主机
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== BOOT (SLAVE) ===");

  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  delay(200);

  setMotorType(MOTOR_MODE_310);
  delay(200);
  s_mbReady = true;
  sendMotorRaw(0, 0, 0, 0);
  delay(100);

#if !USE_STEPPER
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  yawServo.setPeriodHertz(50);

  yawServo.attach(YAW_SERVO_PIN, YAW_US_MIN, YAW_US_MAX);
  delay(100);
  g_yawCurrentUs = YAW_CENTER_US;
  yawServo.writeMicroseconds(YAW_CENTER_US);
#else

  Serial.println("[STEP] 步进模式：已跳过舵机初始化（GPIO27 不使用、无 PWM 输出）");
#endif

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);        // 省电模式会让收包卡几百 ms，断链一大半是它
  WiFi.setAutoReconnect(true);
#if SLAVE_STATIC_IP
  WiFi.config(IPAddress(192, 168, 4, 200),
              IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  Serial.println("[LINK] 固定 IP 192.168.4.200（不走 DHCP，避开 AP 的租约段）");
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  memcpy(g_mac, mac, 6);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
  }
  #if STEP_TELEMETRY_EN
  Serial.println("[STEP] 反向遥测发送已启用（200ms 一帧，经 TCP 回传给主机）");
  #endif
  Serial.println("=== SLAVE READY ===");
  g_linkDownSince = millis();   // 从开机起就开始计时，别把首次连接算成掉线
  lastMotorSend   = millis();
  g_lastYawRecvMs = millis();
  rxLen           = 0;
}

#if STEP_TELEMETRY_EN
// 把炮塔实际转速和底盘编码器发回主机
static uint8_t s_wrErr = 0;

static void sendStepTelemetry() {
  static unsigned long lastMs = 0;
  const unsigned long now = millis();
  if (now - lastMs < STEP_TELEMETRY_MS) return;
  lastMs = now;
  if (!tcpClient.connected()) return;

  const int freeB = tcpClient.availableForWrite();
  if (freeB > 0 && freeB < (int)sizeof(StepTelemetry)) { g_tmSkip++; return; }

  g_tmRaw = s_raw;   // 位置读的是 150ms 一次，发之前再取一遍，别让主机拿到 500ms 前的旧角

  StepTelemetry t;
  t.m0          = STEP_TELE_MAGIC0;
  t.m1          = STEP_TELE_MAGIC1;
  t.type        = STEP_TELE_TYPE;
  t.dpsCmd10    = (int16_t)constrain(g_tmDpsCmd * 10.0f, -32000.0f, 32000.0f);
  t.dpsOut10    = (int16_t)constrain(g_tmDpsOut * 10.0f, -32000.0f, 32000.0f);
  t.dpsLimit    = (int16_t)constrain(g_tmLimit,          -32000.0f, 32000.0f);
  t.rpm         = g_tmRpm;
  t.dir         = g_tmDirCW ? 1 : 0;
  t.stallClears = g_tmStall;
  t.rxErr       = (uint16_t)(zdt.rxErrCount() & 0xFFFFu);

  t.rpmReal     = (int16_t)constrain(g_motorDpsActual * 10.0f, -32000.0f, 32000.0f);
  t.rawPos      = g_tmRaw;
  t.ms          = (uint32_t)now;
  {
    const uint8_t* p = (const uint8_t*)&t;
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < sizeof(StepTelemetry); i++) sum = (uint8_t)(sum + p[i]);
    t.sum = sum;
  }
  const size_t wrote = tcpClient.write((const uint8_t*)&t, sizeof(t));
  if (wrote == sizeof(t)) {
    g_tmSent++;
    s_wrErr = 0;
  } else {
    g_tmFail++;
    // 连续写不进去多半是半开连接：buffer 满了对端也不 ACK
    // 与其在这儿死等，不如自己断开让重连逻辑接手
    if (++s_wrErr >= LINK_WR_ERR_MAX) {
      s_wrErr = 0;
      Serial.println("[LINK] ✗ 遥测连续写失败 => 判定半开，主动断开重连");
      tcpClient.stop();
    }
  }
}
#endif

// 主循环 收包 驱动炮塔 驱动底盘 发遥测
void loop() {

  maintainConnection();

  while (tcpClient.connected() && tcpClient.available() > 0) {
    int c = tcpClient.read();
    if (c < 0) break;
    rxBuf[rxLen++] = (uint8_t)c;
    if (rxLen == (int)sizeof(CtrlPacket)) {
      CtrlPacket pkt;
      memcpy(&pkt, rxBuf, sizeof(pkt));
      handleCtrlPacket(pkt);
      rxLen = 0;
    }
  }

  unsigned long now = millis();

  // 链路心跳：5 秒一行，收包速率掉下来就是链路在变差
  {
    static unsigned long s_hbMs   = 0;
    static uint32_t      s_hbPkts = 0;
    if (now - s_hbMs >= 5000UL) {
      const uint32_t dp = g_rxPkts - s_hbPkts;
      s_hbPkts = g_rxPkts;
      s_hbMs   = now;
      const unsigned long silent = (g_lastPktMs == 0) ? 99999UL : (now - g_lastPktMs);
      Serial.printf("[LINK] hb 连接#%lu 掉线#%lu 收包 %lu/5s 静默 %lu ms",
                    (unsigned long)g_linkConnects, (unsigned long)g_linkDrops,
                    (unsigned long)dp, silent);
      Serial.println((silent > LINK_LOST_MS) ? " => 判失联" : "");
    }
  }

  motBusPump();

  motBusPump();

  {
    static bool          s_linkLost  = false;
    static unsigned long s_zeroSentMs = 0;
    const bool lost = (g_lastPktMs != 0) &&
                      ((unsigned long)(now - g_lastPktMs) > (unsigned long)LINK_LOST_MS);
    if (lost) {
      if (!s_linkLost) {
        s_linkLost = true;
        Serial.printf("[SAFE] ⚠ TCP 失联 %lu ms => 炮塔停、底盘 PWM 归零、主动断开重连\n",
                      (unsigned long)(now - g_lastPktMs));
      }

      g_pktTarget01 = 0;

      if (now - s_zeroSentMs >= (unsigned long)LINK_ZERO_MS) {
        s_zeroSentMs = now;
        sendMotorRaw(0, 0, 0, 0);
      }

      if (tcpClient.connected()) tcpClient.stop();
    } else if (s_linkLost) {
      s_linkLost = false;
      Serial.println("[SAFE] 控制包已恢复，链路正常");
    }
  }

#if !USE_STEPPER
  if (now - g_lastYawRecvMs > YAW_TIMEOUT_MS && g_yawAngle != YAW_CENTER) {
    g_yawAngle = YAW_CENTER;
    g_yawDirty = true;
  }
#endif

#if USE_STEPPER

  stepJogPoll();
  applyStepperTarget();
  #if STEP_TELEMETRY_EN
  sendStepTelemetry();
  #endif
#endif

#if !USE_STEPPER
  if (g_yawDirty) {
    g_yawDirty = false;
    if (g_yawAngle == YAW_CENTER) {

      if (g_yawCenterSinceMs == 0) g_yawCenterSinceMs = now;
      if (now - g_yawCenterSinceMs >= YAW_SOFTSTOP_HOLD_MS) beginSoftStop();
    } else {
      g_yawCenterSinceMs = 0;
      writeYawImmediate(g_yawAngle);
    }
  }

  if (g_yawSoftStop) {
    if (now - g_yawLastStepMs >= YAW_RAMP_STEP_MS) {
      g_yawLastStepMs = now;
      int diff = YAW_CENTER_US - g_yawCurrentUs;
      if (abs(diff) <= 3) {
        g_yawCurrentUs = YAW_CENTER_US;
        yawServo.writeMicroseconds(YAW_CENTER_US);
        g_yawSoftStop = false;
      } else {
        g_yawCurrentUs += diff / YAW_RAMP_DIV;
        yawServo.writeMicroseconds(g_yawCurrentUs);
      }
    }
  }
#endif

  if (now - lastMotorSend >= MOTOR_RESEND_MS) {
    lastMotorSend = now;
    sendMotorRaw(g_mFL, g_mFR, g_mRL, g_mRR);
  }

  delay(LOOP_DELAY_MS);
}