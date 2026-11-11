
/*测试喵*/

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

/* 主机喵 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <DevLab_BMI323.h>
#include <MadgwickAHRS.h>
#include <Preferences.h>

// 引脚 陀螺走 I2C 俯仰舵机一根 发射电机一根
#define PITCH_SERVO_PIN 39
#define SDA_PIN 15
#define SCL_PIN 16

// 俯仰舵机上电停中间 行程大概 100 到 150 超出会顶结构
#define PITCH_INIT_DEG   127
#define PITCH_MIN_DEG    100
#define PITCH_MAX_DEG    147

// 发射电机单独一路 PWM
#define SHOOT_PWM_PIN   3
#define SHOOT_CH        5

#define SHOOT_PWM_FREQ  5000


#define SHOOT_DUTY      255

// 每 100ms 打一发 大概每秒十发
// 下面那个是每次通电多久 窄了推不动 宽了一次可能出两发
// 串口按 < 和 > 能现场改 不用重烧
#define SHOOT_PERIOD_MS_DEF     100UL
#define SHOOT_PULSE_MS_DEF       40UL
volatile unsigned long g_shootPeriodMs = SHOOT_PERIOD_MS_DEF;
volatile unsigned long g_shootPulseMs  = SHOOT_PULSE_MS_DEF;

// 主机开 TCP 服务 从机连过来 控制包走这条线
#define CTRL_TCP_PORT 3333

// 方向符号 下面几个翻错会直接失控 别一次改两个
#define REVERSE_YAW   1

// 上电时稳定器默认关着 要在网页上开
#define YAW_STAB_ON        0

// 炮塔 PID 三个数 调之前先看文件最后那段
// 真正起作用的是 KP 乘 8.333 也就是每差一度 给多少度每秒的修正速度
// KP 管现在差多少 KI 管一直差多少 KD 管正在多快地在动
#define YAW_KP                   0.4900f
#define YAW_KI                   0.7470f
#define YAW_KD                   0.0500f


// D 项死区 陀螺停着不动也在抖几度 不压掉的话 D 会把抖动当真运动去阻尼

#define YAW_D_RATE_DEADBAND      8.0f
// D 不能比 P 大 超了会自己激励出振荡 1.0 就是最多跟 P 一样
#define YAW_KD_MAX_RATIO         1.0f
// 误差很小的时候 P 也小 按比例算 D 会被压没 所以留个地板
// 0.09 乘输出上限大概 13 度命令 保证静止时 D 还剩一点力气
#define YAW_D_LIM_FLOOR_RATIO    0.090f



// 高速要顶住 637 度每秒 换算成命令大概 76 这里 95 留了点余量
// 加大好顶 但松手后要多推一段才会收住 炮塔就冲过头
#define YAW_I_LIMIT              88.0f
// 误差超过这个就不让积分再涨 180 基本等于不限制 真正的限制看下面那条
#define YAW_I_ERR_GATE           180.0f
// 误差收敛太快的时候停积分 免得带着一堆积分冲过头
// 80 的意思 每秒缩小 80 度以上就算在快速回追 这时候不该再攒
#define YAW_I_SLEW_GATE_DPS      80.0f
#define YAW_I_SLEW_GATE_OFF      0
#define YAW_I_PRELOAD_EN         0
// 松手后积分按 150ms 时间常数慢慢撤 窗口 600ms 是四个时间常数 撤干净
// 撤太快炮塔已经被惯性带走没人管 撤太慢积分还在推 就甩出去了
#define YAW_I_DECAY_MS           600


// 输出上限 144 度命令 乘下面那个换算正好 1200 度每秒 步进电机最快转速
#define YAW_MAX_TRIM             144.0f
#define YAW_MAX_TRIM_RATE_DPS    1200.0f
#define YAW_CMD_TO_DPS           (YAW_MAX_TRIM_RATE_DPS / YAW_MAX_TRIM)

// 推杆时目标最多领先炮塔 50 度
// 设小了目标会跟着炮塔跑 环路失去固定基准 推杆就往一个方向摆
// 推杆瞬间真实滞后大概 17 度 50 差不多三倍 正常推杆根本碰不到这个钳位
#define YAW_MAX_LEAD             50.0f
#define YAW_DEADBAND_DEG         0.0f


// 测量滤波 两级低通 滤得重稳但反应慢
// 换方向时误差会涨到摇杆速度乘延迟 大概 17 度 这是物理的 调增益消不掉
// 第二级专门给 D 用 因为 D 对噪声最敏感
#define YAW_MEAS_ALPHA           0.14f
#define YAW_MEAS_ALPHA2          0.30f


// 摇杆推到底目标每秒转多少度
// 下面 FF_PER_DPS 是配套的 160 乘 0.12 正好 19.2 度命令 等于 160 度每秒
// 这样推杆时速度是前馈直接给的 不用等 P 追
#define YAW_JOY_RATE             120.0f
#define YAW_FF_PER_DPS           0.12f
// 推杆期间 P 要不要降权 现在 1.0 不降
// 试过降 0.3 结果 P 变小了 D 还是全权 环路变成继电器 反向打舵开始摆 退回来了
#define YAW_JOY_KP_SCALE         1.00f


// 底盘起转前馈 底盘刚起步那一下会把炮塔拖着走 先按标定值反向推一把
// 1 开 0 关 关掉就退回转向期间稳态偏几十度 不过不会摆
#define YAW_FF_ENABLE            1
// 两个档位分开给 低速拖得慢起转快 给满会冲过头所以踩到 0.7
// 高速必须给满 要顶住 637 度每秒 少给多少就留多少缺口
#define YAW_FF_SCALE_LOW         0.70f
#define YAW_FF_SCALE_HIGH        0.98f
#define YAW_FF_SCALE             YAW_FF_SCALE_HIGH
// 前馈自己也有斜坡 时间常数要跟底盘真实起转对上
// 跟太快会冲在底盘前面 太慢就等于没补
#define YAW_FF_RISE_MS           400.0f
#define YAW_FF_RISE_ALPHA        (CONTROL_DT * 1000.0f / YAW_FF_RISE_MS)

// ==================== 底盘自转 实测（陀螺在炮塔上）====================
// 陀螺装在炮塔上 它量到的是【炮塔的绝对角速度】
// 稳定器把炮塔顶住的时候 陀螺读数≈0 底盘在转这件事在陀螺里是看不见的
// 所以：
//     底盘自转（陀螺系）= 陀螺 − 极性 × 炮塔相对转速
// 炮塔相对转速就是本任务自己下发的 g_yawRateDps 极性 yawSign 标定时已经学到
// 为什么要它：低速档「自转 + 四项平移」时 底盘实际转速会偏离「命令 × k」那个估值
//             （开环占空比 死区非线性 加上平移分掉负载）
//             前馈照着估值给 就会多补或者少补 炮口在混合那几秒里漂几度
// 只用来改前馈的【幅度】 符号还是命令的 比值超出下面这个窗口就不用（退回估值）
// ==================== 静止锁存 ====================
// 稳定器开着，炮塔会「看起来在慢慢跑」，根子不是炮塔要动，是陀螺零偏在积分：
//   测得角 = ∫陀螺 → 零偏让这个积分一直往一个方向走
//   环路看见「测得的角跑了」，就下指令把炮塔转回去追这个假漂移
//   步进最低档是 1 rpm = 6 dps，追 0.03 dps 的漂移只能隔几秒打一个 6 dps 的短脉冲
//   => 编码器上就是 0.1° 一小步、一小步地爬
// 所以：底盘没转、摇杆没动、误差又很小的时候，直接把目标对齐到当前朝向，
// 让漂移自己流走，不让炮塔去追。真要瞄（误差大 / 摇杆动 / 底盘在转）立刻交回环路
#define YAW_QUIET_HOLD_EN     1
#define YAW_QUIET_ERR_DEG     1.0f
#define YAW_QUIET_MS       1500UL

#define YAW_CHS_MEAS_EN       1
#define YAW_CHS_MEAS_R_MIN    0.40f
#define YAW_CHS_MEAS_R_MAX    1.40f


// 稳定器反馈方向 取反了就是正反馈 一开就自己往一个方向加速 遥控也没用
// 这个数是现场试出来的 别凭推理改
#define YAW_STAB_SIGN            -1.0f
// 摇杆左右方向 只影响摇杆 不影响稳定器 两个符号互相独立
#define YAW_STEER_SIGN           (+1.0f)

// 极性标定 0 上电自动测 1 强制用上面那个 STAB_SIGN
#define YAW_SIGN_FORCE     0

#define YAW_SIGN_AUTODETECT 1

// 陀螺读出来的角速度 度每秒 imuTask 每 5ms 更新一次
// 后面两个是滤过的 一级给状态用 二级专门给 D 用
volatile float g_gyroYawRate = 0;

volatile float g_gyroYawRateLP = 0;
#define GYRO_RATE_LP_ALPHA  0.376f

volatile float g_gyroYawRateLP2 = 0;
#define GYRO_RATE_LP2_ALPHA  0.12f

volatile float g_yawOutMag = 0;

volatile float g_yawRateDps = 0.0f;

// 把角度硬压到正负 180 之内 用在需要绝对范围的地方
// 这个不能用在误差上 误差跨过 180 会反号 环路就顺着拖拽方向推了
static inline float clamp180f(float a) {
  if (a >  180.0f) return  180.0f;
  if (a < -180.0f) return -180.0f;
  return a;
}

// 把角度折回正负 180 连续目标角要用这个折
static inline float wrap180f(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// 运行状态 稳定器开关 校准请求 底盘在不在转 前馈当前给了多少
// 这些是几个任务共用的 所以都加了 volatile
volatile int g_stabOn = (YAW_STAB_ON != 0);

volatile int g_reqBiasCal  = 0;
volatile int g_biasCalBusy = 0;

volatile bool  g_biasCalOff = false;
volatile float g_calErr     = 0.0f;
volatile int g_reqPolarCal = 0;

volatile int g_stabAlignReq = 0;

volatile bool g_chassisTurning = false;

volatile float g_chassisTurnDps = 0.0f;

// 陀螺在炮塔上 底盘自转要靠「陀螺 − 极性×炮塔相对转速」解出来 见 YAW_CHS_MEAS_EN
volatile float g_chsMeasDps  = 0.0f;
volatile bool  g_yawChsMeasOn = (YAW_CHS_MEAS_EN != 0);

volatile float g_ffDps = 0.0f;

// 极性标定 朝一边转一点二秒大概四十八度 量出实际转向再转回来
// 回位按角度判断 四秒只是兜底超时
#define YAW_CAL_MOVE_TIME_MS 1200
#define YAW_CAL_BACK_TIME_MS 4000
#define YAW_CAL_MIN_DELTA    8.0f

#define YAW_CAL_RATE_DPS     40.0f
#define YAW_CAL_BACK_TOL     3.0f

// 调试开关 稳定器日志和陀螺自检 平时开着方便看
#define STAB_DIAG          1
#define IMU_SELFTEST       1

#define IMU_SAMPLE_FREQ_HZ 200.0f

// 陀螺超过 200ms 没新数据就不信它 退回纯摇杆 总比拿旧值硬转强
#define YAW_IMU_TIMEOUT_MS 200

// 下发节流 变化太小或者间隔太短就不发 省点带宽
#define YAW_SEND_MIN_MS    15
#define YAW_SEND_MIN_STEP  1

// 底盘速度 这里是高档 下面低档慢一点 方便精细操作
#define SPEED_FWD     7.8f
#define SPEED_STRAFE  7.8f

// 原地转向看门狗 两秒还没测到底盘在动就强制切断
// 但前提是编码器真的报过数 没报过就当它坏了 不给切
#define CHS_TURN_TIMEOUT_MS   2000UL
#define SPEED_TURN    4.9f

// 档位 0 低 1 高
#define SPD_MODE_LOW        0
#define SPD_MODE_HIGH       1

#define SPD_LOW_FWD     3.0f
#define SPD_LOW_STRAFE  3.0f
#define SPD_LOW_TURN    2.1f

// ==================== 低速档「原地自转 + 前后左右」====================
// 麦轮四轮速度本来就是「平移」和「自转」两个分量直接相加，相加这一步老代码已经有
// 真正卡住的是减出来的那只轮子：
//   低速档 平移 3.0 减 自转 2.1 只剩 0.9，还不到单独自转 2.1 的一半
//   0.9 连静摩擦都过不去，那只轮子不转就变成刹车，整个底盘丢掉自转
//   表现就是：按住原地左再按前，车不转，只往前蹭
// 做法：混合时把平移整体放大，抬到最慢那只轮子也还有 MIX_MIN_WHEEL_CMD
// 自转尽量不动 —— 它是炮塔前馈标定过的量，一削前馈就不准了
// 抬到 MIX_LIFT_MAX 还不够，才退一步削自转，最少留 MIX_SPIN_MIN_RATIO
// 高速档明确不支持：那个速度下麦轮本来就打滑，转速也没法标定，混起来全是未知
#define MIX_SPIN_LOW_EN      1

#define MIX_SPIN_HIGH_EN     0

#define MIX_MIN_WHEEL_CMD    2.0f
#define MIX_LIFT_MAX         1.60f
#define MIX_SPIN_MIN_RATIO   0.75f

// 混合时套在「自转分量」上的左右微调，必须和从机 TRIM_LEFT / TRIM_RIGHT 一致
// 从机只在纯转向 'T' 时套微调；混合走 'M' 不套，否则两边都套就变成 1.21
// 也不能把微调套在整条左轮上，那样平移也跟着歪，车会斜着走
#define MIX_TRIM_LEFT        1.10f
#define MIX_TRIM_RIGHT       1.00f

// ==================== 四项平移的基准 = 炮塔朝向 ====================
// 原地转向 + 平移时，前/后/左/右 不再跟着车头转，而是以【炮口当前朝向】为基准
// 车体在转，「前」永远是炮口方向。稳定器开着的时候炮口是钉死的，所以车走直线
// 换算只需要「炮口相对车体的角度 θ」，车体系 = 炮口系绕 z 轴转 -θ：
//     vx_c = vx*cosθ − va*sinθ
//     va_c = vx*sinθ + va*cosθ
// θ 来自步进编码器 rawPos（65536/圈）减去「炮口正前」那个零点
// 零点存 flash，开机读；现场用网页「炮口正前」或串口 z 重量一次
// 只在低速档混控里生效，纯平移还是车体系（不然炮塔歪着的时候按前会横着走）
#define MIX_FRAME_TURRET     1

// 炮口往右打、按前车却往左走 => 把这个符号翻过来
#define MIX_FRAME_SIGN       (+1.0f)

// 底盘在转 炮塔相对车体的角就一直在变，所以这个角必须【每个控制周期重算】
// 遥测 50ms 一帧，帧之间用最近两帧的角度差做一阶外推，把这几度滞后补掉
// 不然车体 48 dps 自转时方向永远慢半拍（50~80ms ≈ 4°）
#define TZ_RATE_LP_A         0.35f
#define TZ_RATE_MAX_DPS    400.0f
#define TZ_EXTRAP_MAX_DEG   15.0f

// 底盘每单位速度命令大概对应多少度每秒 这是初值 实际会被 k 标定覆盖
#define YAW_CHASSIS_TURN_DPS_PER_UNIT  23.0f

// 底盘 k 标定 让底盘匀速自转 用陀螺量真实转速 算出每单位命令对应多少度每秒
// 前馈要靠这个数 标定期间底盘会无视遥控自己转 所以要按两次 k 才启动
#define CALK_EN              1
#define CALK_SETTLE_MS       500
#define CALK_TURN_MS        1500
#define CALK_IDLE_MS         300
#define CALK_MIN_DPS         8.0f
#define CALK_MAX_DPS      1200.0f
#define CALK_K_MIN           5.0f
#define CALK_K_MAX         200.0f

#define CHS_FF_MIN_RAW        4
#define CHS_FF_FRESH_MS     500UL

#define CALK_TOTAL_TIMEOUT_MS 12000

static float g_kDpsPerUnitLow  = YAW_CHASSIS_TURN_DPS_PER_UNIT;
static float g_kDpsPerUnitHigh = YAW_CHASSIS_TURN_DPS_PER_UNIT;

volatile int g_calKForceDir  = 0;
volatile int g_calKForceGear = -1;

volatile unsigned long g_calKLastTickMs = 0;
#define CALK_OVERRIDE_TTL_MS  200
volatile int g_calKReq       = 0;

#define CALK_NVS_NS     "calk"
#define CALK_NVS_KEY    "rec"
#define CALK_STORE_MAGIC 0x4B4C4131u
#define CALK_STORE_VER   1

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t ver;
  uint16_t runs;

  float    kLow;
  float    meanLow;
  float    sdLow;
  float    minLow, maxLow;
  uint32_t nLow;

  float    kHigh;
  float    meanHigh;
  float    sdHigh;
  float    minHigh, maxHigh;
  uint32_t nHigh;
} CalKStore;
static CalKStore g_calKStore;
static bool      g_calKStoreValid = false;

static unsigned long g_calkBootEchoAt = 0;
static uint8_t       g_calkBootEchoN  = 0;

volatile int g_speedMode = SPD_MODE_LOW;

// 低速档「自转+平移」开关，串口按 m 可以现场开关，方便 A/B 对比
volatile bool g_mixSpinLow = (MIX_SPIN_LOW_EN != 0);

// 混控平移基准：炮口相对车体的角度（正 = 炮口朝右），来自步进编码器
static volatile uint16_t g_turretRawPos   = 0;
static volatile bool     g_turretRawValid = false;
static uint16_t          g_turretZeroRaw  = 0;
static bool              g_turretZeroSet  = false;
static volatile float    g_turretRelDeg   = 0.0f;

typedef struct __attribute__((packed)) {
// 发给从机的控制包 十三字节 两边结构必须一模一样 改一边就得改另一边
  char    chassis;
  uint8_t yaw;

  int16_t fl;
  int16_t fr;
  int16_t rl;
  int16_t rr;

  int16_t targetAngle01deg;
  uint8_t maxSpeedDps;

} CtrlPacket;

CtrlPacket ctrlPacket = {'S', 90, 0, 0, 0, 0, 0, 0};

#define STEP_TELEMETRY_EN    1
#define STEP_TELE_MAGIC0  0xA5
#define STEP_TELE_MAGIC1  0x5A
#define STEP_TELE_TYPE    0x01
typedef struct __attribute__((packed)) {
  uint8_t  m0, m1;
  uint8_t  type;
// 从机回传的遥测 炮塔实际转速和底盘编码器 两边也必须对齐
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
static uint8_t  s_tmBuf[sizeof(StepTelemetry)];
static int      s_tmLen = 0;

#define CHS_TELE_TYPE   0x02
typedef struct __attribute__((packed)) {
  uint8_t  m0, m1;
  uint8_t  type;
  int16_t  encA, encB, encC, encD;
  int32_t  yawRaw;
  uint32_t ms;
  uint8_t  sum;
} ChsTelemetry;
static ChsTelemetry s_chsTm;
static uint32_t     s_chsFrames = 0;
static unsigned long s_chsLastRxMs = 0;
static int           dbgFfBlkCnt   = 0;
static int      s_tmExpect = 0;
static uint32_t s_tmFrames = 0;
static unsigned long s_tmLastWarnMs = 0;
static uint32_t s_tmSeenAtWarn = 0;
static bool     s_tmEverGot = false;
static uint32_t s_tmBadSum = 0;

static int32_t  s_prevRaw     = -1;
static uint32_t s_prevRawMs   = 0;
static float    s_motorDps    = 0.0f;
static bool     s_motorMoved  = false;

Servo pitchServo;
DevLab_BMI323 imu(Wire, 0x69);
BMI323_SensorData imuData;
Madgwick filter;

WebServer        server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiServer ctrlServer(CTRL_TCP_PORT);
WiFiClient ctrlClient;

// ==================== 从机链路实时判定 ====================
// 从机 100ms 一帧遥测，这么久没帧就当它出事了
// 半开连接（主机/从机任一方硬复位不发 FIN）socket 一直报 connected，只有看帧年龄才看得出来
#define LINK_TM_OK_MS       600UL
#define LINK_TM_LOST_MS    1800UL

#define LINK_ST_DOWN   0
#define LINK_ST_SLOW   1
#define LINK_ST_OK     2

static unsigned long    s_lastTmMs    = 0;
static volatile int      g_linkState  = LINK_ST_DOWN;
static volatile uint32_t g_linkAgeMs  = 0;
static uint32_t          g_linkCloses = 0;

static void tzUpdateFromRaw(uint16_t raw);   // 定义在下面，遥测解析先用到

// 从机链路体检 每个 loop 跑一次 只看「最后一帧遥测多老了」
// 状态变了才打日志，别刷屏；半开连接由这儿主动关掉，从机才会去重连
static void linkTick() {
  const unsigned long now = millis();
  const unsigned long age = (s_lastTmMs == 0) ? 0xFFFFFFFFUL : (now - s_lastTmMs);

  const int st = (s_lastTmMs == 0 || age > LINK_TM_LOST_MS) ? LINK_ST_DOWN
               : (age > LINK_TM_OK_MS)                       ? LINK_ST_SLOW
                                                             : LINK_ST_OK;
  g_linkAgeMs = (age > 60000UL) ? 60000UL : age;

  if (st != g_linkState) {
    const int prev = g_linkState;
    g_linkState = st;
    if (st == LINK_ST_OK)        Serial.printf("[LINK] ✓ 从机在线（帧间隔 %lu ms）\n", age);
    else if (st == LINK_ST_SLOW) Serial.printf("[LINK] ⚠ 从机变慢：%lu ms 没新帧\n", age);
    else if (prev != LINK_ST_DOWN || s_lastTmMs != 0)
                                 Serial.printf("[LINK] ✗ 从机掉线（最后一帧 %lu ms 前）\n", age);
  }

  if (st == LINK_ST_DOWN && ctrlClient && ctrlClient.connected() &&
      s_lastTmMs != 0 && age > LINK_TM_LOST_MS) {
    ctrlClient.stop();
    g_linkCloses++;
    Serial.printf("[LINK] ✗ 半开连接已关闭 #%lu，等从机重连\n", (unsigned long)g_linkCloses);
  }
}

static void pollSlaveTelemetry() {
  if (!(ctrlClient && ctrlClient.connected())) return;
  while (ctrlClient.available() > 0) {
    const int ci = ctrlClient.read();
    if (ci < 0) break;
    const uint8_t b = (uint8_t)ci;

    if (s_tmLen == 0) {
      if (b == STEP_TELE_MAGIC0) s_tmBuf[s_tmLen++] = b;
      continue;
    }
    if (s_tmLen == 1) {
      if (b == STEP_TELE_MAGIC1) s_tmBuf[s_tmLen++] = b;
      else s_tmLen = (b == STEP_TELE_MAGIC0) ? 1 : 0;
      continue;
    }
    s_tmBuf[s_tmLen++] = b;

    if (s_tmLen == 3) {
      if (s_tmBuf[2] == CHS_TELE_TYPE)       s_tmExpect = (int)sizeof(ChsTelemetry);
      else if (s_tmBuf[2] == STEP_TELE_TYPE) s_tmExpect = (int)sizeof(StepTelemetry);
      else { s_tmLen = 0; s_tmExpect = 0; continue; }
    }
    if (s_tmExpect == 0 || s_tmLen < s_tmExpect) continue;

    if (s_tmBuf[2] == CHS_TELE_TYPE) {
      ChsTelemetry c;
      memcpy(&c, s_tmBuf, sizeof(c));
      s_tmLen = 0; s_tmExpect = 0;
      const uint8_t* q = (const uint8_t*)&c;
      uint8_t s2 = 0;
      for (size_t i = 2; i + 1 < sizeof(ChsTelemetry); i++) s2 = (uint8_t)(s2 + q[i]);
      if (s2 != c.sum) { s_tmBadSum++; continue; }
      s_chsTm = c;
      s_chsFrames++;
      s_chsLastRxMs = millis();
      Serial.printf("[CHS] enc20 A=%d B=%d C=%d D=%d | yawRaw=%ld\n",
                    (int)c.encA, (int)c.encB, (int)c.encC, (int)c.encD,
                    (long)c.yawRaw);
      continue;
    }

    StepTelemetry t;
    memcpy(&t, s_tmBuf, sizeof(t));
    s_tmLen = 0; s_tmExpect = 0;
    const uint8_t* p = (const uint8_t*)&t;
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < sizeof(StepTelemetry); i++) sum = (uint8_t)(sum + p[i]);
    if (sum != t.sum || t.type != STEP_TELE_TYPE) { s_tmBadSum++; continue; }

    s_lastTmMs = millis();
    tzUpdateFromRaw(t.rawPos);

    s_tmFrames++;
    if (s_tmFrames == 1) {
      Serial.println("[TM] ✓ 已收到从机遥测（此后不再提示首次）");
    }
    const float cmd   = t.dpsCmd10 / 10.0f;
    const float out   = t.dpsOut10 / 10.0f;
    const float ratio = (fabsf(cmd) > 1.0f) ? (fabsf(out) / fabsf(cmd)) : 1.0f;

    const float mot = (float)t.rpmReal / 10.0f;
    s_motorDps   = mot;
    s_motorMoved = (fabsf(mot) > 0.5f);

    Serial.printf("[从机] 令%7.1f 发%7.1f 限%5.0f 令RPM=%3u | 电机实=%7.1f raw=%5u%s | %s 堵转%u 串错%u\n",
                  cmd, out, (float)t.dpsLimit, (unsigned)t.rpm,
                  mot, (unsigned)t.rawPos, s_motorMoved ? "" : " 静止",
                  t.dir ? "正" : "反",
                  (unsigned)t.stallClears, (unsigned)t.rxErr);
  }

  const unsigned long nowMs = millis();
  if (nowMs - s_tmLastWarnMs >= 5000) {
    s_tmLastWarnMs = nowMs;
    if (s_tmFrames == s_tmSeenAtWarn) {
      Serial.printf("[TM] ⚠ 这 5 秒没有收到任何遥测帧（累计帧=%lu，校验失败=%lu）\n"
                    "     从机固件里 STEP_TELEMETRY_EN 是否为 1？链路是否断了？\n",
                    (unsigned long)s_tmFrames, (unsigned long)s_tmBadSum);
    }
    s_tmSeenAtWarn = s_tmFrames;
  }
}

const char* ssid     = "BMPT-90Terminator3";
const char* password = "targetdestory";

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

float GYRO_SCALE = 0.0610352f;

float g_rateCapDps = 110.0f;

float g_outMax = 13.2f;
const float ACCEL_SCALE = 0.000598;

#define GYRO_BIAS_IDLE_DPS   8.0f

#define BIAS_LEARN_CMD_LIMIT  15.0f
#define GYRO_BIAS_TRACK      0.00125f

#define BIAS_LEARN_QUIET_MS   3000

#define BIAS_FAST_N          100
#define BIAS_FAST_MAX_DPS    0.35f
#define BIAS_FAST_GAIN       0.60f
#define BIAS_FAST_MAX_LSB    50.0f

#define YAW_BIAS_TRACK_EN        0

#define YAW_BIAS_MIN_MS          4000

#define YAW_BIAS_MIN_DPS         0.25f

#define YAW_BIAS_STEP_GAIN       0.40f

#define YAW_BIAS_MAX_TRAVEL_LSB  80.0f

volatile float sharedPitch = 0;
volatile float sharedYaw   = 0;

volatile float yawAngle    = 0;
volatile float sharedYawRate = 0;
volatile unsigned long sharedYawMs = 0;
volatile uint32_t      imuReadOk = 0;
volatile uint32_t      imuReadFail = 0;

volatile bool cmdForward  = false;
volatile bool cmdBackward = false;

volatile bool g_layoutGan   = true;
volatile bool cmdStrafeLeft  = false;
volatile bool cmdStrafeRight = false;
volatile bool cmdLeft     = false;
volatile bool cmdRight    = false;
volatile bool cmdShoot    = false;

volatile int joyX = 0;
volatile int joyY = 0;

volatile float g_joyNx = 0;

volatile float g_turretAngleDeg = 0;
// 给网页看的炮口朝向和目标 单位 0.1 度
volatile int g_yawNow01 = 0;
volatile int g_yawTgt01 = 0;

#define YAW_SERVO_FULL_DPS  180.0f

// ==================== 摇杆 → 炮塔/俯仰：两种做法，择一 ====================
// 方法一（当前启用）：网页【右半屏滑动瞄准】，三角洲行动那套。
//   手指在右半区任意位置按下去就能滑，横滑 = 炮塔转，纵滑 = 抬/压炮口，
//   斜着滑两个轴同时出力；手指停住立刻收住，没有惯性。
//   因为 x/y 是两支独立通道，主机侧【不能再做竖直锥门控】—— 否则斜滑时
//   joyY 会被判成「在打舵」整个吞掉，抬不了炮口。见 ctrlTask 里那段 #if。
//   网页侧对应：#joystick 隐藏 + #aimZone 接管右半屏（见 html 里 AIM_PAD_EN 那段）。
//
// 方法二（备用，在 ctrlTask 的 #else 分支里，未参与编译 = 等同于注释掉）：
//   保留右下角固定摇杆，只把「要多接近竖直才允许俯仰」的锥角从 15° 放宽到竖直 ±60°。
//   要换回方法二：这里改 0；网页里 AIM_PAD_EN 改 false；
//   再把 CSS 里 #joystick 的 display:none 那行删掉。三处一起改，别只改一处。
#define AIM_PAD_EN              1

// 方法二专用：atan2(|x|,|y|) ≤ 这个角就照常俯仰，超过就当成纯打舵（只转不抬）
// 15 => 60：竖直方向左右各 60° 的扇区都算「在竖直方向推」，斜着推也能抬炮口
#define PITCH_JOY_MAX_TILT_DEG  60.0f
#define JOY_DEADZONE   8

volatile bool g_imuOk = false;
volatile unsigned long g_lastWsMs  = 0;
volatile unsigned long g_lastJoyMs = 0;
volatile bool          g_joyActive = false;
const    unsigned long JOY_TIMEOUT_MS = 500;

const float CONTROL_DT = 0.005;
const float PITCH_JOY_RATE = 60.0;

bool g_shootFiring = false;

// 发射 见上面占空比那段说明 这里只是初始化
void initShootMotor() {

  pinMode(SHOOT_PWM_PIN, OUTPUT);
  digitalWrite(SHOOT_PWM_PIN, LOW);

  ledcSetup(SHOOT_CH, SHOOT_PWM_FREQ, 8);
  ledcAttachPin(SHOOT_PWM_PIN, SHOOT_CH);
  ledcWrite(SHOOT_CH, 0);
}

// 通电四十毫秒 断六十毫秒 循环就是每秒十发
// 松手立刻断电 不留相位 保证松手就停
void updateShootMotor(bool fire) {

  static unsigned long s_burstT0 = 0;
  static bool          s_burstOn = false;

  if (!fire) {
    if (s_burstOn || s_burstT0 != 0) ledcWrite(SHOOT_CH, 0);
    s_burstOn = false;
    s_burstT0 = 0;
    return;
  }

  const unsigned long now = millis();
  if (s_burstT0 == 0) {
    s_burstT0 = now;
    s_burstOn = true;
    ledcWrite(SHOOT_CH, SHOOT_DUTY);
    return;
  }
  const unsigned long el = now - s_burstT0;
  if (s_burstOn) {

    if (el >= g_shootPulseMs) {
      s_burstOn = false;
      ledcWrite(SHOOT_CH, 0);
    }
  } else {

    if (el >= g_shootPeriodMs) {
      s_burstT0 = now;
      s_burstOn = true;
      ledcWrite(SHOOT_CH, SHOOT_DUTY);
    }
  }
}

// 把控制包通过 TCP 发给从机
void sendToSlave() {
  if (!ctrlClient || !ctrlClient.connected()) return;
  // 明确判掉线了就别再往里灌，半开的 socket 只会越积越多
  if (g_linkState == LINK_ST_DOWN && s_lastTmMs != 0) return;
  ctrlClient.write((const uint8_t*)&ctrlPacket, sizeof(ctrlPacket));
}

static void calKSaveToNvs() {
  Preferences p;
  if (!p.begin(CALK_NVS_NS, false)) {
    Serial.println("[CALK] ✗ NVS 打开失败，本次结果只留在内存");
    return;
  }
  g_calKStore.magic = CALK_STORE_MAGIC;
  g_calKStore.ver   = CALK_STORE_VER;
  p.putBytes(CALK_NVS_KEY, &g_calKStore, sizeof(g_calKStore));
  p.end();
  g_calKStoreValid = true;
  Serial.printf("[CALK] 已存入 flash（第 %u 次标定，低档 n=%lu 高档 n=%lu）\n",
                (unsigned)g_calKStore.runs,
                (unsigned long)g_calKStore.nLow, (unsigned long)g_calKStore.nHigh);
}

static bool calKLoadFromNvs() {
  Preferences p;
  if (!p.begin(CALK_NVS_NS, true)) return false;
  CalKStore tmp{};
  const size_t got = p.getBytes(CALK_NVS_KEY, &tmp, sizeof(tmp));
  p.end();
  if (got != sizeof(tmp) || tmp.magic != CALK_STORE_MAGIC || tmp.ver != CALK_STORE_VER) {
    g_calKStoreValid = false;
    return false;
  }

  if (!(tmp.kLow  >= CALK_K_MIN && tmp.kLow  <= CALK_K_MAX) ||
      !(tmp.kHigh >= CALK_K_MIN && tmp.kHigh <= CALK_K_MAX)) {
    Serial.printf("[CALK] ✗ 存档里的 k 不合理（低 %.2f 高 %.2f，应在 %.0f~%.0f）=> 视为无存档\n",
                  (double)tmp.kLow, (double)tmp.kHigh, (double)CALK_K_MIN, (double)CALK_K_MAX);
    g_calKStoreValid = false;
    return false;
  }
  g_calKStore      = tmp;

  if (g_calKStore.nLow == 0 && g_calKStore.nHigh > 0) {
    g_calKStore.kLow = g_calKStore.kHigh;
    Serial.println("[CALK] 注意：存档里低档无采样，k(低) 已用高档的值代替");
  }
  if (g_calKStore.nHigh == 0 && g_calKStore.nLow > 0) {
    g_calKStore.kHigh = g_calKStore.kLow;
    Serial.println("[CALK] 注意：存档里高档无采样，k(高) 已用低档的值代替");
  }
  g_calKStoreValid = true;
  return true;
}

static void calKDumpFromNvs() {
  Serial.println("[CALK] ================= 上次 k 标定存档 =================");
  if (!calKLoadFromNvs()) {
    Serial.println("[CALK] NVS 里没有存档（还没做过 k 标定）。做一次：关稳定器后按 'k'；做完按 'd' 可随时回放");
    Serial.println("[CALK] ===================================================");
    return;
  }
  const CalKStore& s = g_calKStore;
  Serial.printf("[CALK] 第 %u 次标定\n", (unsigned)s.runs);
  Serial.printf("[CALK] 低档 V_TURN=%.2f  底盘 %6.1f dps  sd=%4.1f  范围 %6.1f~%6.1f  n=%lu\n",
                (double)SPD_LOW_TURN, (double)s.meanLow, (double)s.sdLow,
                (double)s.minLow, (double)s.maxLow, (unsigned long)s.nLow);
  Serial.printf("[CALK]   => 拟合 k(低) = %.2f dps/V_TURN  (预启动前馈用 %.2f)\n",
                (double)s.kLow, (double)g_kDpsPerUnitLow);
  Serial.printf("[CALK] 高档 V_TURN=%.2f  底盘 %6.1f dps  sd=%4.1f  范围 %6.1f~%6.1f  n=%lu\n",
                (double)SPEED_TURN, (double)s.meanHigh, (double)s.sdHigh,
                (double)s.minHigh, (double)s.maxHigh, (unsigned long)s.nHigh);
  Serial.printf("[CALK]   => 拟合 k(高) = %.2f dps/V_TURN  (预启动前馈用 %.2f)\n",
                (double)s.kHigh, (double)g_kDpsPerUnitHigh);
  Serial.printf("[CALK] 预旋转速度(满档时应立即给出的角速度): 低 %.0f dps  高 %.0f dps\n",
                (double)(s.meanLow), (double)(s.meanHigh));
  Serial.println("[CALK] ===================================================");
}

void computeWheelSpeeds();
void sendToSlave();

static void calKForceStopSend() {
  g_calKForceDir = 0;
  computeWheelSpeeds();
  sendToSlave();
}

#if CALK_EN
// k 标定的流程 启动 稳定 转 收数据 拟合 存档 这几步轮着走
static bool calkTick() {
  static int           ph     = 0;
  static unsigned long tAll   = 0;
  static unsigned long t0     = 0;
  static float         acc    = 0.0f;
  static float         accT   = 0.0f;
  static float         meanL  = 0.0f, meanH = 0.0f;

  static uint32_t      stN    = 0;
  static float         stMean = 0.0f, stM2 = 0.0f, stMin = 0.0f, stMax = 0.0f;
  static uint32_t      stNLow = 0, stNHigh = 0;

  static float         riseProf[10] = {0};
  static uint8_t       riseN       = 0;
  static float         sdLow  = 0.0f, sdHigh = 0.0f;
  static float         mnLow  = 0.0f, mxLow = 0.0f, mnHigh = 0.0f, mxHigh = 0.0f;

  const unsigned long now = millis();

  if (g_calKReq > 0 && ph == 0) {
    g_calKReq = 0;
    if (g_stabOn) {
      Serial.println("[CAL-K] ✗ 请先关闭稳定器再标定：稳定器会反向顶住，测到的不是底盘转速");
      return false;
    }
    if (fabsf(g_joyNx) > 0.05f) {
      Serial.println("[CAL-K] ✗ 请先松开摇杆");
      return false;
    }
    Serial.println("[CAL-K] ⚠ 开始原地转向标定：底盘将低速正转约 2 秒，再高速正转约 2 秒");
    Serial.println("[CAL-K] ⚠ 请确保四周空旷、炮塔线缆有余量！要中止请按 'x'");
    ph = 1; t0 = now; acc = 0.0f; accT = 0.0f;
    tAll = 0;
    stN = 0; stMean = 0.0f; stM2 = 0.0f; stMin = stMax = 0.0f;
    riseN = 0; memset(riseProf, 0, sizeof(riseProf));
    g_calKForceDir  = +1;
    g_calKForceGear = SPD_MODE_LOW;
    return true;
  }
  if (ph == 0) {
    if (g_calKReq < 0) g_calKReq = 0;
    return false;
  }

  g_calKLastTickMs = now;

  if (g_calKReq < 0) {
    g_calKReq = 0;
    g_calKForceGear = -1;
    calKForceStopSend();
    ph = 0;
    Serial.println("[CAL-K] 已中止，已强制停轮，k 保持原值");
    return false;
  }

  const unsigned long el = now - t0;

  if (tAll == 0) tAll = now;
  if (now - tAll > CALK_TOTAL_TIMEOUT_MS) {
    g_calKForceGear = -1;
    calKForceStopSend();
    ph = 0; tAll = 0;
    Serial.println("[CAL-K] ✗ 超时保护触发：已强制停轮，k 保持原值");
    return false;
  }
  if (ph == 1 || ph == 3) {

    static unsigned long lastAccMs = 0;
    if (el >= CALK_SETTLE_MS) {
      if (lastAccMs != 0) {
        const float dt = (float)(now - lastAccMs) * 0.001f;
        acc  += g_gyroYawRate * dt;
        accT += dt;

        const float x = g_gyroYawRate;
        stN++;
        const float d1 = x - stMean;
        stMean += d1 / (float)stN;
        stM2   += d1 * (x - stMean);
        if (stN == 1) { stMin = stMax = x; }
        else { if (x < stMin) stMin = x; if (x > stMax) stMax = x; }
      }
      lastAccMs = now;
    } else {
      lastAccMs = 0;

      const uint8_t slot = (uint8_t)(el / 50u);
      if (slot < 10u && slot >= riseN) {
        riseProf[slot] = g_gyroYawRate;
        riseN = (uint8_t)(slot + 1u);
      }
    }
    const unsigned long total = CALK_SETTLE_MS + CALK_TURN_MS;
    if (el >= total) {
      const float meanDps = (accT > 0.05f) ? (acc / accT) : 0.0f;
      const float vTurn   = (ph == 1) ? SPD_LOW_TURN : SPEED_TURN;
      const float mag     = fabsf(meanDps);
      if (meanDps == 0.0f || mag < CALK_MIN_DPS || mag > CALK_MAX_DPS) {
        Serial.printf("[CAL-K] ✗ %s档 实测底盘转速 %.1f dps 不合理（应在 %.0f~%.0f）=> 本档 k 不更新\n",
                      (ph == 1) ? "低" : "高", mag, CALK_MIN_DPS, CALK_MAX_DPS);
      } else {
        const float k = mag / vTurn;
        const float sd = (stN > 1) ? sqrtf(stM2 / (float)(stN - 1)) : 0.0f;

        const float absA = fabsf(stMin), absB = fabsf(stMax);
        const float rMin = (absA < absB) ? absA : absB;
        const float rMax = (absA < absB) ? absB : absA;
        const float kClamped = (k < CALK_K_MIN) ? CALK_K_MIN
                             : (k > CALK_K_MAX) ? CALK_K_MAX : k;
        if (ph == 1) {
          meanL = mag; g_kDpsPerUnitLow = kClamped;
          stNLow = stN; sdLow = sd; mnLow = rMin; mxLow = rMax;
        } else {
          meanH = mag; g_kDpsPerUnitHigh = kClamped;
          stNHigh = stN; sdHigh = sd; mnHigh = rMin; mxHigh = rMax;
        }
        Serial.printf("[CAL-K] ✓ %s档：底盘 %.1f dps  sd=%.1f  范围 %.1f~%.1f  n=%lu / V_TURN %.1f => k = %.2f\n",
                      (ph == 1) ? "低" : "高", mag, sd, stMin, stMax,
                      (unsigned long)stN, vTurn, k);

        {
          Serial.printf("[CAL-K]   起转过程(每50ms, 稳态 %.0f dps):", mag);
          for (uint8_t i = 0; i < riseN; i++) Serial.printf(" %5.0f", riseProf[i]);
          Serial.println();
          int idx63 = -1;
          for (uint8_t i = 0; i < riseN; i++) {
            if (fabsf(riseProf[i]) >= 0.632f * mag) { idx63 = (int)i; break; }
          }
          if (idx63 < 0) {
            Serial.println("[CAL-K]   ⚠ 500ms 内没到 63%，起转很慢 => 斜坡建议加大到 300ms");
          } else {
            Serial.printf("[CAL-K]   => 63%% 在约 %d ms  => 建议 YAW_FF_RISE_MS ≈ %d\n",
                          idx63 * 50, idx63 * 50);
          }
        }
      }
      calKForceStopSend();
      if (ph == 1) { ph = 2; t0 = now; }
      else         { ph = 4; t0 = now; }
    }
  } else if (ph == 2) {
    if (el >= CALK_IDLE_MS) {
      acc = 0.0f; accT = 0.0f;
      stN = 0; stMean = 0.0f; stM2 = 0.0f; stMin = stMax = 0.0f;
    riseN = 0; memset(riseProf, 0, sizeof(riseProf));
      g_calKForceDir  = +1;
      g_calKForceGear = SPD_MODE_HIGH;
      ph = 3; t0 = now;
    }
  } else if (ph == 4) {
    g_calKForceGear = -1;
    calKForceStopSend();

    g_calKStore.runs = (uint16_t)(g_calKStoreValid ? (g_calKStore.runs + 1) : 1);
    g_calKStore.kLow     = g_kDpsPerUnitLow;
    g_calKStore.meanLow  = meanL;
    g_calKStore.sdLow    = sdLow;
    g_calKStore.minLow   = mnLow;
    g_calKStore.maxLow   = mxLow;
    g_calKStore.nLow     = stNLow;
    g_calKStore.kHigh    = g_kDpsPerUnitHigh;
    g_calKStore.meanHigh = meanH;
    g_calKStore.sdHigh   = sdHigh;
    g_calKStore.minHigh  = mnHigh;
    g_calKStore.maxHigh  = mxHigh;
    g_calKStore.nHigh    = stNHigh;
    calKSaveToNvs();

    Serial.printf("[CAL-K] 完成。低档 k=%.2f (底盘 %.1f dps)  高档 k=%.2f (底盘 %.1f dps)\n",
                  g_kDpsPerUnitLow, meanL, g_kDpsPerUnitHigh, meanH);
    Serial.println("[CAL-K] 已存入 flash => 下次开机会在监视器开头自动回放");
    ph = 0;
    return false;
  }

  {
    static unsigned long s_lastWarn = 0;
    if (now - s_lastWarn >= 1000UL) {
      s_lastWarn = now;
      Serial.printf("[CAL-K] ⚠ 标定进行中：底盘正被强制驱动，遥控【暂时无效】。按 'x' 可中止\n");
    }
  }

  static unsigned long lastSendMs = 0;
  if (now - lastSendMs >= 20) {
    lastSendMs = now;
    computeWheelSpeeds();
    sendToSlave();
  }
  return true;
}
#endif

// 低速档「自转+平移」的配比计算，原理见文件上面那一段
// 入参 vx 前后 / va 左右 / vb 自转，都带符号
// 出参：平移放大 lam 倍、自转缩成 mu 倍，使得四轮里最慢那只也到 MIX_MIN_WHEEL_CMD
// 顺序：先只抬平移（mu=1，自转一点不动），抬到 MIX_LIFT_MAX 还不够才退自转
static void mixSpinTranslate(float &vx, float &va, float &vb, float &lamOut, float &muOut) {
  static const float kMu[4] = {1.00f, 0.90f, 0.80f, MIX_SPIN_MIN_RATIO};

  // 四轮 = 平移分量 + 自转分量，自转那一项带左右微调
  const float tBase[4] = { vx + va, vx - va, vx - va, vx + va };
  const float sBase[4] = { vb * MIX_TRIM_LEFT, -vb * MIX_TRIM_RIGHT,
                           vb * MIX_TRIM_LEFT, -vb * MIX_TRIM_RIGHT };

  for (uint8_t k = 0; k < 4; k++) {
    const float mu  = kMu[k];
    float       lam = 1.0f;
    bool        ok  = true;

    for (uint8_t i = 0; i < 4; i++) {
      const float t = tBase[i];
      const float s = mu * sBase[i];
      if (t > 1e-4f) {
        const float need = (MIX_MIN_WHEEL_CMD - s) / t;
        if (need > lam) lam = need;
      } else if (t < -1e-4f) {
        const float need = (-MIX_MIN_WHEEL_CMD - s) / t;
        if (need > lam) lam = need;
      } else if (fabsf(s) < MIX_MIN_WHEEL_CMD) {
        ok = false;
      }
    }

    if (lam < 1.0f) lam = 1.0f;
    if (ok && lam <= MIX_LIFT_MAX) {
      vx *= lam; va *= lam; vb *= mu;
      lamOut = lam; muOut = mu;
      return;
    }
  }

  // 实在抬不动：按最大抬升 + 最大削自转硬给，至少保证四轮都在动
  vx *= MIX_LIFT_MAX; va *= MIX_LIFT_MAX; vb *= MIX_SPIN_MIN_RATIO;
  lamOut = MIX_LIFT_MAX; muOut = MIX_SPIN_MIN_RATIO;
}

// 炮口零点（炮口正对车头时的编码器 rawPos）的存取
// 存 flash 是因为步进编码器一断电就不知道自己在哪了，靠这个零点把「正前」找回来
static void tzSave() {
  Preferences p;
  if (!p.begin("tz", false)) return;
  p.putUShort("raw0", g_turretZeroRaw);
  p.end();
}

static void tzLoad() {
  Preferences p;
  if (!p.begin("tz", true)) return;
  if (p.isKey("raw0")) {
    g_turretZeroRaw = p.getUShort("raw0", 0);
    g_turretZeroSet = true;
  }
  p.end();
  Serial.printf("[FRAME] 炮口零点 %s（%u）\n",
                g_turretZeroSet ? "已载入" : "没有存档，拿第一次遥测位置当正前",
                (unsigned)g_turretZeroRaw);
}

// 每收到一帧步进遥测就更新一次「炮口相对车体」的角度
static float         s_tzMeasDeg  = 0.0f;   // 最新一帧测到的角度
static float         s_tzPrevDeg  = 0.0f;   // 上一帧，用来估角速度
static unsigned long s_tzMeasMs   = 0;      // 最新一帧的时间
static unsigned long s_tzPrevMs   = 0;
static float         s_tzRateDps  = 0.0f;   // 估出来的炮塔相对角速度

static void tzUpdateFromRaw(uint16_t raw) {
  g_turretRawPos   = raw;
  g_turretRawValid = true;
  if (!g_turretZeroSet) { g_turretZeroRaw = raw; g_turretZeroSet = true; }

  int32_t d = (int32_t)raw - (int32_t)g_turretZeroRaw;
  if (d >  32768) d -= 65536;
  if (d < -32768) d += 65536;

  const float         newDeg = MIX_FRAME_SIGN * (float)d * 360.0f / 65536.0f;
  const unsigned long nowMs  = millis();

  if (s_tzPrevMs != 0) {
    const unsigned long dt = nowMs - s_tzPrevMs;
    if (dt >= 15 && dt <= 500) {
      float dd = newDeg - s_tzPrevDeg;
      while (dd >  180.0f) dd -= 360.0f;
      while (dd < -180.0f) dd += 360.0f;
      float r = dd * 1000.0f / (float)dt;
      if (r >  TZ_RATE_MAX_DPS) r =  TZ_RATE_MAX_DPS;
      if (r < -TZ_RATE_MAX_DPS) r = -TZ_RATE_MAX_DPS;
      s_tzRateDps += TZ_RATE_LP_A * (r - s_tzRateDps);
    }
  }

  s_tzPrevDeg  = newDeg;
  s_tzPrevMs   = nowMs;
  s_tzMeasDeg  = newDeg;
  s_tzMeasMs   = nowMs;
  g_turretRelDeg = newDeg;
}

// 当前用的「炮口相对车体」角度：测得的角度 + 角速度 × 已经过去的时间
// 车体在转 → 这个角每一帧都不一样 → 必须每次重算，不能按一次记住一整个动作
static float tzFrameDeg() {
  if (s_tzMeasMs == 0) return g_turretRelDeg;
  float corr = s_tzRateDps * (float)(millis() - s_tzMeasMs) * 0.001f;
  if (corr >  TZ_EXTRAP_MAX_DEG) corr =  TZ_EXTRAP_MAX_DEG;
  if (corr < -TZ_EXTRAP_MAX_DEG) corr = -TZ_EXTRAP_MAX_DEG;
  float a = s_tzMeasDeg + corr;
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// 现场标定：炮口正对车头时调用，把当前位置记成正前
static void tzZeroNow() {
  if (!g_turretRawValid) {
    Serial.println("[FRAME] ✗ 还没收到步进遥测，位置未知，无法标定");
    return;
  }
  g_turretZeroRaw = g_turretRawPos;
  g_turretZeroSet = true;
  tzSave();
  // 零点一变角度会跳一大截，先把角速度估计清掉，别让外推跟着抽一下
  s_tzPrevDeg = 0.0f;
  s_tzPrevMs  = 0;
  s_tzRateDps = 0.0f;
  tzUpdateFromRaw(g_turretRawPos);
  Serial.printf("[FRAME] ✓ 炮口正前已标定 rawPos=%u（存 flash）\n", (unsigned)g_turretZeroRaw);
}

// 把网页按键变成四个轮子的速度
// 原地转向时左右两侧从机会套不同微调 直行和平移不套 保持对称
void computeWheelSpeeds() {

  float fl = 0, fr = 0, rl = 0, rr = 0;
  bool fwd = cmdForward && !cmdBackward;
  bool bwd = cmdBackward && !cmdForward;
  bool left = cmdLeft && !cmdRight;
  bool right = cmdRight && !cmdLeft;

  bool strafeLeft  = cmdStrafeLeft  && !cmdStrafeRight;
  bool strafeRight = cmdStrafeRight && !cmdStrafeLeft;

  const bool calKOverrideAlive =
      (g_calKForceDir != 0) &&
      (millis() - g_calKLastTickMs < (unsigned long)CALK_OVERRIDE_TTL_MS);
  if (calKOverrideAlive) {
    left  = (g_calKForceDir < 0);
    right = (g_calKForceDir > 0);
    fwd = false; bwd = false; strafeLeft = false; strafeRight = false;
  }

  static unsigned long s_turnHoldSince = 0;
  static bool          s_turnAborted   = false;

  static bool          s_chsEverWorked = false;
  {
    const bool fresh0 = (s_chsLastRxMs != 0) &&
                        ((millis() - s_chsLastRxMs) < (unsigned long)CHS_FF_FRESH_MS);
    if (fresh0 && labs((long)s_chsTm.yawRaw) >= (long)CHS_FF_MIN_RAW) s_chsEverWorked = true;
  }
  if (!calKOverrideAlive) {
    if (left || right) {
      if (s_turnHoldSince == 0) s_turnHoldSince = millis();

      const bool chsFresh = (s_chsLastRxMs != 0) &&
                            ((millis() - s_chsLastRxMs) < (unsigned long)CHS_FF_FRESH_MS);
      const bool chsFeedbackOk = chsFresh &&
                                 (labs((long)s_chsTm.yawRaw) >= (long)CHS_FF_MIN_RAW);
      if (s_chsEverWorked && !chsFeedbackOk &&
          (millis() - s_turnHoldSince) >= (unsigned long)CHS_TURN_TIMEOUT_MS) {
        if (!s_turnAborted) {
          s_turnAborted = true;
          Serial.printf("[SAFE] ⚠ 原地转向已持续 %lu ms 且底盘测速无回传"
                        "（yawRaw=%ld）=> 强制中断转向，松开按键可恢复\n",
                        (unsigned long)(millis() - s_turnHoldSince),
                        (long)s_chsTm.yawRaw);

        }
      }
    } else {
      if (s_turnAborted) Serial.println("[SAFE] 转向键已松开，中断解除");
      s_turnHoldSince = 0;
      s_turnAborted   = false;
    }
    if (s_turnAborted) { left = false; right = false; }
  }

  const bool hi = (calKOverrideAlive && g_calKForceGear >= 0)
                      ? (g_calKForceGear == SPD_MODE_HIGH)
                      : (g_speedMode == SPD_MODE_HIGH);
  const float V_FWD    = hi ? SPEED_FWD    : SPD_LOW_FWD;
  const float V_STRAFE = hi ? SPEED_STRAFE : SPD_LOW_STRAFE;
  const float V_TURN   = hi ? SPEED_TURN   : SPD_LOW_TURN;

  // 三个分量：vx 前后、va 左右、vb 自转（符号沿用老代码：原地左 vb 为负）
  float vx = (fwd ? V_FWD : 0.0f) + (bwd ? -V_FWD : 0.0f);
  float va = ((strafeRight ? 1.0f : 0.0f) - (strafeLeft ? 1.0f : 0.0f)) * V_STRAFE;
  float vb = ((right ? 1.0f : 0.0f) - (left ? 1.0f : 0.0f)) * V_TURN;

  const bool wantMove = (fabsf(vx) > 1e-4f) || (fabsf(va) > 1e-4f);
  const bool wantSpin = (fabsf(vb) > 1e-4f);

  // 两个平移方向一起按时（前+左移）按矢量长度归一
  // 不归一的话两个方向各给满，单轮命令直接翻倍，电流也跟着翻倍
  {
    const float vmag = sqrtf(vx * vx + va * va);
    const float vcap = (V_FWD > V_STRAFE) ? V_FWD : V_STRAFE;
    if (vmag > vcap) { const float k = vcap / vmag; vx *= k; va *= k; }
  }

  float lam    = 1.0f;
  float mu     = 1.0f;
  bool  mixNow = false;

  // 只有「干」字布局才有独立的自转键 + 平移键，十字布局的前+左是斜着平移，不参与
  if (g_layoutGan && wantSpin && wantMove) {
    if (hi && !MIX_SPIN_HIGH_EN) {
      // 高速档明确不支持：丢掉自转，只执行平移
      vb = 0.0f;
    } else if (!g_mixSpinLow) {
      vb = 0.0f;
    } else {
      // 基准 = 炮口朝向：把炮口系的 (vx,va) 转到车体系，车体再自己去转
      // 注意这个角是【现算】的，底盘一边转它一边变，所以下面 loop 里 50Hz 重算
      if (MIX_FRAME_TURRET) {
        const float th = tzFrameDeg() * 0.0174532925f;
        const float c  = cosf(th);
        const float s  = sinf(th);
        const float vxr = vx * c - va * s;
        const float var = vx * s + va * c;
        vx = vxr; va = var;
      }
      mixSpinTranslate(vx, va, vb, lam, mu);
      mixNow = true;
    }
  }

  {
    static int s_mixDbg = -1;
    const int dbg = mixNow ? 1
                  : (!(wantSpin && wantMove)) ? 0
                  : (hi ? 2 : 3);
    if (dbg != s_mixDbg) {
      const int prev = s_mixDbg;
      s_mixDbg = dbg;
      if (dbg == 1) {
        Serial.printf("[MIX] 低速档 自转+平移：炮口%+6.1f° 平移×%.2f 自转×%.2f => vx=%.2f va=%.2f vb=%.2f\n",
                      tzFrameDeg(), lam, mu, vx, va, vb);
      } else if (dbg == 2) {
        Serial.println("[MIX] 高速档不支持自转+平移 => 只执行平移（要混合请切低速）");
      } else if (dbg == 3) {
        Serial.println("[MIX] 低速档混合已关闭（串口按 m 打开）=> 只执行平移");
      } else if (prev > 0) {
        Serial.println("[MIX] 自转+平移 结束");
      }
    }
  }

  g_chassisTurning = (fabsf(vb) > 1e-4f);
  g_chassisTurnDps = vb * (hi ? g_kDpsPerUnitHigh : g_kDpsPerUnitLow);

  if (g_layoutGan) {

    // 麦轮四轮 = 平移 + 自转，两个分量直接相加
    // 混合时自转那一项在这里套左右微调（从机只对纯转向 'T' 套，'M' 不套）
    // 纯转向和纯平移时 kL/kR 都是 1.0，输出跟老代码逐位一致
    const float kL = mixNow ? MIX_TRIM_LEFT  : 1.0f;
    const float kR = mixNow ? MIX_TRIM_RIGHT : 1.0f;

    fl = vx + va + vb * kL;
    fr = vx - va - vb * kR;
    rl = vx - va + vb * kL;
    rr = vx + va - vb * kR;
  } else {

    if (fwd && left) {

      fl = -V_STRAFE; fr = V_STRAFE; rl = V_STRAFE; rr = -V_STRAFE;
    } else if (fwd && right) {

      fl = V_STRAFE; fr = -V_STRAFE; rl = -V_STRAFE; rr = V_STRAFE;
    } else if (bwd && left) {

      fl = V_STRAFE; fr = -V_STRAFE; rl = -V_STRAFE; rr = V_STRAFE;
    } else if (bwd && right) {

      fl = -V_STRAFE; fr = V_STRAFE; rl = V_STRAFE; rr = -V_STRAFE;
    } else if (fwd) {
      fl = fr = rl = rr = V_FWD;
    } else if (bwd) {
      fl = fr = rl = rr = -V_FWD;
    } else if (left) {
      fl = -V_TURN; fr = V_TURN; rl = -V_TURN; rr = V_TURN;
    } else if (right) {
      fl = V_TURN; fr = -V_TURN; rl = V_TURN; rr = -V_TURN;
    } else {
      fl = fr = rl = rr = 0;
    }
  }

  {
    static int s_prevNonZero = 0;
    const int nowNonZero = ((fl != 0.0f) || (fr != 0.0f) || (rl != 0.0f) || (rr != 0.0f)) ? 1 : 0;
    if (s_prevNonZero && !nowNonZero) {
      Serial.printf("[CMD] 轮速归零 <- cmdL=%d cmdR=%d cmdF=%d cmdB=%d layout=%d aborted=%d "
                    "stabOn=%d calOff=%d calBusy=%d gear=%d\n",
                    (int)cmdLeft, (int)cmdRight, (int)cmdForward, (int)cmdBackward,
                    (int)g_layoutGan, (int)s_turnAborted,
                    (int)g_stabOn, (int)g_biasCalOff, (int)g_biasCalBusy, (int)g_speedMode);
    }
    s_prevNonZero = nowNonZero;
  }

  {
    static int16_t       pfl = -32768, pfr = -32768, prl = -32768, prr = -32768;
    static unsigned long s_lastChsLog = 0;
    const unsigned long  nowLog = millis();
    if (((int16_t)fl != pfl || (int16_t)fr != pfr ||
         (int16_t)rl != prl || (int16_t)rr != prr) &&
        (nowLog - s_lastChsLog >= 500UL)) {     // 自转+平移时四轮值一直在变，限 2 行/秒
      pfl = fl; pfr = fr; prl = rl; prr = rr;
      s_lastChsLog = nowLog;
      Serial.printf("[CHS] 四轮命令 fl=%d fr=%d rl=%d rr=%d  炮口%+6.1f°%s\n",
                    (int)fl, (int)fr, (int)rl, (int)rr,
                    tzFrameDeg(), mixNow ? " (混合)" : "");
    }
  }

  ctrlPacket.fl = fl;
  ctrlPacket.fr = fr;
  ctrlPacket.rl = rl;
  ctrlPacket.rr = rr;

  // 'T' 纯原地转向：从机套左右微调、走软启动斜坡（老路径，一点没动）
  // 'M' 低速档 自转+平移：主机已经套过自转那一项，从机不能再套，也别打斜坡
  // 'S' 纯平移
  if (mixNow)                                     ctrlPacket.chassis = 'M';
  else if ((fabsf(vb) > 1e-4f) && !wantMove)      ctrlPacket.chassis = 'T';
  else                                            ctrlPacket.chassis = 'S';
}

// 轮速从串口发给底盘驱动板
void sendChassisCommand() {
  computeWheelSpeeds();
  sendToSlave();
}

// 开机测陀螺零偏 要静止 测一千个取平均 残差太大就重测 最多三次
void calibrateGyro() {
  const int N = 1000;
  const int CHK = 300;

  const float RESID_LIMIT_DPS = 0.3f;

  for (int attempt = 1; attempt <= 3; attempt++) {
    float sumX = 0, sumY = 0, sumZ = 0;
    int got = 0;
    for (int i = 0; i < N; i++) {
      if (imu.readData(imuData)) {
        sumX += imuData.gyrX;
        sumY += imuData.gyrY;
        sumZ += imuData.gyrZ;
        got++;
      }
      delay(2);
    }
    if (got <= 0) { return; }

    gyroBiasX = sumX / got;
    gyroBiasY = sumY / got;
    gyroBiasZ = sumZ / got;

    float chkSum = 0; int chkGot = 0;
    for (int i = 0; i < CHK; i++) {
      if (imu.readData(imuData)) { chkSum += imuData.gyrZ; chkGot++; }
      delay(2);
    }
    const float residDps = (chkGot > 0)
        ? ((chkSum / chkGot) - gyroBiasZ) * GYRO_SCALE : 0.0f;

    Serial.printf("[IMU] 零偏校准第%d次(%d 样本)  biasZ=%.1f LSB (~%.3f dps)  残差=%.3f dps\n",
                  attempt, got, gyroBiasZ, gyroBiasZ * GYRO_SCALE, residDps);

    if (fabsf(residDps) <= RESID_LIMIT_DPS) {
      if (attempt > 1) Serial.println("[IMU] ✓ 重试后零偏合格");
      return;
    }
    Serial.printf("[IMU] ⚠ 残差 %.2f dps 偏大（>%.1f）—— 校准时炮塔可能在动，重来\n",
                  residDps, RESID_LIMIT_DPS);
    delay(600);
  }
  Serial.println("[IMU] ✗ 连续 3 次零偏残差都偏大 —— 请确认上电时炮塔静止、I2C 接线牢固");
  Serial.println("[IMU]   稳定器仍可用（积分会顶上），但航向会缓慢漂移");
}

// 200Hz 读陀螺 积分出炮塔现在朝哪
// yaw 是积分出来的 不是传感器直接给的 所以零偏必须校
void imuTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();

  uint32_t stLastMs = 0, stCnt = 0;
  float    stIntegDeg = 0.0f, stPeak = 0.0f, stYaw0 = 0.0f;
  while (1) {
    if (!g_imuOk) { vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5)); continue; }
    if (imu.readData(imuData)) {

      float gx = (imuData.gyrX - gyroBiasX) * GYRO_SCALE;
      float gy = (imuData.gyrY - gyroBiasY) * GYRO_SCALE;
      float gz = (imuData.gyrZ - gyroBiasZ) * GYRO_SCALE;

      {
        static int   s_bcPh  = 0;
        static float s_bcSX = 0, s_bcSY = 0, s_bcSZ = 0;
        static int   s_bcN   = 0;
        static int   s_bcTry = 0;
        static float s_bcChk = 0;
        static int   s_bcChkN = 0;

        if (g_reqBiasCal && s_bcPh == 0) {
          s_bcPh = 1; s_bcSX = s_bcSY = s_bcSZ = 0.0f; s_bcN = 0;
          s_bcTry = 1; s_bcChk = 0.0f; s_bcChkN = 0;
          g_biasCalBusy = 1;
          Serial.println("[CAL] 开始重做零偏校准（约 2 秒，炮塔须静止）");
        }
        if (s_bcPh == 1) {
          s_bcSX += imuData.gyrX; s_bcSY += imuData.gyrY; s_bcSZ += imuData.gyrZ;
          s_bcN++;
          if (s_bcN >= 400) {
            gyroBiasX = s_bcSX / (float)s_bcN;
            gyroBiasY = s_bcSY / (float)s_bcN;
            gyroBiasZ = s_bcSZ / (float)s_bcN;
            s_bcPh = 2; s_bcChk = 0.0f; s_bcChkN = 0;
          }
        } else if (s_bcPh == 2) {
          s_bcChk += imuData.gyrZ; s_bcChkN++;
          if (s_bcChkN >= 100) {
            const float resid = (s_bcChk / (float)s_bcChkN - gyroBiasZ) * GYRO_SCALE;
            Serial.printf("[CAL] 零偏第%d次: biasZ=%.1f LSB (%.3f dps)  残差 %.3f dps\n",
                          s_bcTry, (double)gyroBiasZ, (double)(gyroBiasZ * GYRO_SCALE), (double)resid);
            if (fabsf(resid) <= 0.3f) {
              Serial.println("[CAL] ✓ 零偏校准完成");
              s_bcPh = 0; g_reqBiasCal = 0; g_biasCalBusy = 0;
            } else if (s_bcTry >= 3) {
              Serial.println("[CAL] ✗ 连续 3 次残差偏大 —— 放弃，零偏保持原值（请让炮塔静止后重试）");
              s_bcPh = 0; g_reqBiasCal = 0; g_biasCalBusy = 0;
            } else {
              s_bcTry++; s_bcPh = 1;
              s_bcSX = s_bcSY = s_bcSZ = 0.0f; s_bcN = 0; s_bcChk = 0.0f; s_bcChkN = 0;
              Serial.println("[CAL] 残差偏大 => 重来");
            }
          }
        }
      }

      g_gyroYawRate = -gz;

      g_gyroYawRateLP  += GYRO_RATE_LP_ALPHA  * (g_gyroYawRate   - g_gyroYawRateLP);
      g_gyroYawRateLP2 += GYRO_RATE_LP2_ALPHA * (g_gyroYawRateLP - g_gyroYawRateLP2);
      yawAngle += (-gz) / IMU_SAMPLE_FREQ_HZ;

      float y = yawAngle;
      while (y >  180.0f) y -= 360.0f;
      while (y < -180.0f) y += 360.0f;
      sharedYaw = y;

      {
        const unsigned long nowMs = millis();

        const bool mayLearn = false && (
            fabsf(g_yawOutMag) < BIAS_LEARN_CMD_LIMIT &&
            !g_chassisTurning &&
            (g_joyNx == 0.0f));
        static uint32_t      s_bzN    = 0;
        static float         s_bzSumZ = 0.0f;
        static float         s_bzSumX = 0.0f;
        static float         s_bzSumY = 0.0f;
        static unsigned long s_quietSince = 0;
        static uint32_t      s_bzUpd = 0;

        if (!mayLearn) {

          s_bzN = 0; s_bzSumZ = 0.0f; s_bzSumX = 0.0f; s_bzSumY = 0.0f;
          s_quietSince = 0;
        } else {
          if (s_quietSince == 0) s_quietSince = nowMs;
          s_bzSumX += imuData.gyrX;
          s_bzSumY += imuData.gyrY;
          s_bzSumZ += imuData.gyrZ;
          s_bzN++;
          if (s_bzN >= (uint32_t)BIAS_FAST_N) {
            const float meanLsbZ = s_bzSumZ / (float)s_bzN;
            const float meanLsbX = s_bzSumX / (float)s_bzN;
            const float meanLsbY = s_bzSumY / (float)s_bzN;
            const float meanDpsZ = meanLsbZ * GYRO_SCALE;
            if (fabsf(meanDpsZ) <= BIAS_FAST_MAX_DPS) {
              float dZ = (meanLsbZ - gyroBiasZ) * BIAS_FAST_GAIN;
              float dX = (meanLsbX - gyroBiasX) * BIAS_FAST_GAIN;
              float dY = (meanLsbY - gyroBiasY) * BIAS_FAST_GAIN;
              if (dZ >  BIAS_FAST_MAX_LSB) dZ =  BIAS_FAST_MAX_LSB;
              if (dZ < -BIAS_FAST_MAX_LSB) dZ = -BIAS_FAST_MAX_LSB;
              if (dX >  BIAS_FAST_MAX_LSB) dX =  BIAS_FAST_MAX_LSB;
              if (dX < -BIAS_FAST_MAX_LSB) dX = -BIAS_FAST_MAX_LSB;
              if (dY >  BIAS_FAST_MAX_LSB) dY =  BIAS_FAST_MAX_LSB;
              if (dY < -BIAS_FAST_MAX_LSB) dY = -BIAS_FAST_MAX_LSB;
              gyroBiasX += dX; gyroBiasY += dY; gyroBiasZ += dZ;
              s_bzUpd++;
              Serial.printf("[IMU] 快速重估零偏 #%lu：窗口均值 %+.3f dps => biasZ %+.1f LSB (修正 %+.2f)\n",
                            (unsigned long)s_bzUpd, (double)meanDpsZ,
                            (double)gyroBiasZ, (double)dZ);
            }
            s_bzN = 0; s_bzSumZ = 0.0f; s_bzSumX = 0.0f; s_bzSumY = 0.0f;
            s_quietSince = 0;
          }
        }
        const bool quietLongEnough = (s_bzUpd > 0);

        static unsigned long s_lastBiasLog = 0;
        if (nowMs - s_lastBiasLog >= 30000UL) {
          s_lastBiasLog = nowMs;
          const float wMean = (s_bzN > 0) ? (s_bzSumZ / (float)s_bzN) * GYRO_SCALE : 0.0f;
          Serial.printf("[IMU] 零偏 biasZ=%+.1f LSB (%+.3f dps)  可学=%s  窗口 %u/%d 均值 %+.2f dps  已更新 %lu 次\n",
                        (double)gyroBiasZ, (double)(gyroBiasZ * GYRO_SCALE),
                        mayLearn ? "允许" : "禁止",
                        (unsigned)s_bzN, (int)BIAS_FAST_N, (double)wMean,
                        (unsigned long)s_bzUpd);
        }
      }

      sharedYawMs = millis();
      imuReadOk++;
    } else {
      imuReadFail++;
    }
    #if IMU_SELFTEST
    {
      stCnt++;
      stIntegDeg += g_gyroYawRate / IMU_SAMPLE_FREQ_HZ;

      if (fabsf(g_gyroYawRate) > fabsf(stPeak)) stPeak = g_gyroYawRate;
      uint32_t nowSt = millis();
      if (stLastMs == 0) { stLastMs = nowSt; stYaw0 = sharedYaw; }
      if (nowSt - stLastMs >= 2000) {
        float dtSec = (nowSt - stLastMs) / 1000.0f;

        float dy = sharedYaw - stYaw0;
        while (dy >  180.0f) dy -= 360.0f;
        while (dy < -180.0f) dy += 360.0f;
        Serial.printf("[IMU] %.1fs 采样%lu(%.0fHz) | 陀螺积分%7.1f度 | yaw变化%7.1f度 | 速率峰值%7.1f dps | 失败%lu\n",
                      dtSec, (unsigned long)stCnt, stCnt / dtSec,
                      stIntegDeg, dy, (float)stPeak, (unsigned long)imuReadFail);
        stLastMs = nowSt; stIntegDeg = 0; stCnt = 0; stPeak = 0; stYaw0 = sharedYaw;
      }
    }
    #endif
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
  }
}

#define BIAS_CAL_SUSPEND_MS   600UL
#define BIAS_CAL_SKIP_MS      200UL
#define BIAS_CAL_SETTLE_ERR   1.5f
#define BIAS_CAL_SETTLE_MS    2500UL
#define BIAS_CAL_RMS_MAX_DPS  0.5f
#define BIAS_CAL_N            80
#define BIAS_CAL_GAIN         0.8f

// 停车后顺手再校一次零偏 零偏会跟着温度慢慢变 开机测一次不够
// 校的时候要把稳定器短暂关掉 不然它自己的修正会把测量带偏
// 两条路 一条是转向结束归位后自动校 一条是手动关稳定器时校
static void biasCalTick() {
  static int           s_bc        = 0;
  static unsigned long s_bcT0      = 0;
  static double        s_bcSum     = 0.0, s_bcSum2 = 0.0;
  static uint32_t      s_bcN       = 0;
  static bool          s_bcPrevTrn = false;
  static uint32_t      s_bcRuns    = 0;

  static double        s_b2Sum  = 0.0, s_b2Sum2 = 0.0;
  static uint32_t      s_b2N    = 0;
  static uint32_t      s_b2Runs = 0;
  static uint32_t      s_b2Drop = 0;

  const bool          turnNow = g_chassisTurning;
  const unsigned long now     = millis();
  const bool          abort   = turnNow || (g_joyNx != 0.0f);

  {
    static unsigned long s_hb = 0;
    if (now - s_hb >= 2000UL) {
      s_hb = now;
      Serial.printf("[CALB] hb  stab=%d calOff=%d joyNx=%+.2f  b2N=%u/%d runs=%u drop=%u\n",
                    (int)g_stabOn, (int)(g_biasCalOff ? 1 : 0), (double)g_joyNx,
                    (unsigned)s_b2N, (int)BIAS_CAL_N,
                    (unsigned)s_b2Runs, (unsigned)s_b2Drop);
    }
  }

  if (!g_stabOn && !g_biasCalOff) {
    if (g_joyNx != 0.0f) {
      s_b2Sum = 0.0; s_b2Sum2 = 0.0; s_b2N = 0;
    } else {
      const float v = (float)imuData.gyrZ;
      s_b2Sum  += v;
      s_b2Sum2 += (double)v * (double)v;
      s_b2N++;
      if (s_b2N >= (uint32_t)BIAS_CAL_N) {
        const double mean = s_b2Sum / (double)s_b2N;
        double var = s_b2Sum2 / (double)s_b2N - mean * mean;
        if (var < 0.0) var = 0.0;
        const double rmsDps = sqrt(var) * (double)GYRO_SCALE;
        if (rmsDps < (double)BIAS_CAL_RMS_MAX_DPS) {
          const float d = (float)((mean - (double)gyroBiasZ) * (double)BIAS_CAL_GAIN);
          gyroBiasZ += d;
          s_b2Runs++;
          Serial.printf("[CALB] OK(stab=0) #%lu: %u samples  mean %+.1f LSB  RMS %.3f dps  => biasZ %+.1f LSB (corr %+.2f)\n",
                        (unsigned long)s_b2Runs, (unsigned)s_b2N,
                        mean, rmsDps, (double)gyroBiasZ, (double)d);
        } else {
          if (s_b2Drop < 5) {
            s_b2Drop++;
            Serial.printf("[CALB] DROP(stab=0): window RMS %.2f dps too large (turret still moving)\n", rmsDps);
          }
        }
        s_b2Sum = 0.0; s_b2Sum2 = 0.0; s_b2N = 0;
      }
    }
    return;
  }

  if (s_bc == 0) {

    if (s_bcPrevTrn && !turnNow && g_stabOn) { s_bc = 1; s_bcT0 = now; }
  } else if (s_bc == 1) {

    if (abort) {
      s_bc = 0;
    } else if (fabsf(g_calErr) < BIAS_CAL_SETTLE_ERR) {
      s_bc = 2; s_bcT0 = now; s_bcSum = 0.0; s_bcSum2 = 0.0; s_bcN = 0;
      g_biasCalOff = true;
      Serial.printf("[CALB] 已归位 (err=%.2f°) => 关闭稳定器 %lums 校准零偏\n",
                    (double)g_calErr, (unsigned long)BIAS_CAL_SUSPEND_MS);
    } else if (now - s_bcT0 >= (unsigned long)BIAS_CAL_SETTLE_MS) {
      Serial.printf("[CALB] 归位超时 (err=%.2f°) => 本次跳过校准（绝不带偏差去校）\n", (double)g_calErr);
      s_bc = 0;
    }
  } else {

    if (abort) {
      g_biasCalOff = false; s_bc = 0;
      Serial.println("[CALB] 中止（又开始动作）=> 本次测量丢弃，稳定器已恢复");
    } else {
      const unsigned long el = now - s_bcT0;
      if (el >= (unsigned long)BIAS_CAL_SKIP_MS) {
        const float v = (float)imuData.gyrZ;
        s_bcSum  += v;
        s_bcSum2 += (double)v * (double)v;
        s_bcN++;
      }
      if (el >= (unsigned long)BIAS_CAL_SUSPEND_MS) {
        g_biasCalOff = false; s_bc = 0;
        if (s_bcN >= 20) {
          const double mean = s_bcSum / (double)s_bcN;
          double var = s_bcSum2 / (double)s_bcN - mean * mean;
          if (var < 0.0) var = 0.0;
          const double rmsDps = sqrt(var) * (double)GYRO_SCALE;
          if (rmsDps < (double)BIAS_CAL_RMS_MAX_DPS) {
            const float d = (float)((mean - (double)gyroBiasZ) * (double)BIAS_CAL_GAIN);
            gyroBiasZ += d;
            s_bcRuns++;
            Serial.printf("[CALB] ✓ 零偏重估 #%lu：%u 样本  均值 %+.1f LSB  RMS %.3f dps  =>  biasZ %+.1f LSB (修 %+.2f)\n",
                          (unsigned long)s_bcRuns, (unsigned)s_bcN,
                          mean, rmsDps, (double)gyroBiasZ, (double)d);
          } else {
            Serial.printf("[CALB] ✗ 丢弃：窗口 RMS %.2f dps 偏大（炮塔还在动）\n", rmsDps);
          }
        } else {
          Serial.println("[CALB] ✗ 丢弃：有效样本不足");
        }
      }
    }
  }
  s_bcPrevTrn = turnNow;
}

// 主控制循环 200Hz PID 在这里算 算完 sendToSlave 发出去
// 顺序是 读目标 算误差 算 PID 限幅 加前馈 下发
void ctrlTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  float servoPitch = PITCH_INIT_DEG;
  int   lastYawCmd = -1;
int16_t lastRate01 = 0;
  unsigned long lastYawSentMs = 0;

  float yawTarget  = 0;

  float calAbsIntegral = 0;
  float yawInteg   = 0;
  bool  yawArmed   = false;
  float yawSign    = (YAW_SIGN_FORCE != 0) ? YAW_STAB_SIGN : 1.0f;
  float yawMeasLP  = 0;
  float yawMeasLP2 = 0;
  float yawMeasCont = 0;

  bool  yawMeasInit = false;

  float dbgP = 0, dbgD = 0, dbgI = 0, dbgOut = 0, dbgErr = 0;
  unsigned long lastStabDbgMs = 0;

  int  calPhase   = 0;
  unsigned long calT0 = 0;
  float calYaw0   = 0;
  float calDelta  = 0;

  bool lastChassisTurning = false;

  #define YAW_STOP      90

  while (1) {
    unsigned long now = millis();

    {
      float py = 0.0f;

      #if AIM_PAD_EN
      // ===== 方法一（启用）：右半屏滑动瞄准 =====
      // 网页那边已经把「手指滑动速度」换算成 -100..100 的杆量发过来了，斜滑时 x/y 同时非零。
      // 这里直接吃 y，不按角度筛 —— 两个轴就是两支独立通道，互不吞。
      // 只留 JOY_DEADZONE 挡指头抖动，免得静止时舵机被一点点噪声带着慢慢爬。
      if (abs(joyY) > JOY_DEADZONE) py = (float)joyY / 100.0f;
      #else
      // ===== 方法二（备用，备用时把 AIM_PAD_EN 改成 0）：固定摇杆 + 竖直 ±60° 锥门控 =====
      // 摇杆要多斜还允许俯仰：atan2(|x|,|y|) ≤ PITCH_JOY_MAX_TILT_DEG 就照常俯仰，
      // 超过这个扇区就当成纯打舵（只转不抬）。
      // 60° 的意思：竖直方向左右各 60° 都算「在竖直方向推」，边转炮塔边抬炮口时
      // 杆可以斜着推，不用先掰回正中再抬。
      if (abs(joyY) > JOY_DEADZONE) py = (float)joyY / 100.0f;
      if (py != 0.0f) {
        const float tiltDeg =
            atan2f((float)abs(joyX), (float)abs(joyY)) * 57.2957795f;
        const bool nearVertical = (abs(joyX) <= JOY_DEADZONE) ||
                                  (tiltDeg <= PITCH_JOY_MAX_TILT_DEG);
        if (!nearVertical) py = 0.0f;
      }
      #endif

      if (py != 0.0f) {
        servoPitch += py * PITCH_JOY_RATE * CONTROL_DT;
        servoPitch = constrain(servoPitch, PITCH_MIN_DEG, PITCH_MAX_DEG);
      }
    }
    pitchServo.write((int)servoPitch);

    float measYaw  = sharedYaw;
    bool  imuFresh = g_imuOk && (now - sharedYawMs) < YAW_IMU_TIMEOUT_MS;
    int   yawCmd   = YAW_STOP;

    if (!(ctrlClient && ctrlClient.connected())) {
      if (now - lastYawSentMs >= 100) lastYawSentMs = now;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
      continue;
    }

  #if YAW_SIGN_AUTODETECT && !YAW_SIGN_FORCE

      if (g_reqPolarCal) {
        g_reqPolarCal = 0;
        if (g_stabOn) {
          calPhase = 0;
          Serial.println("[CAL] 网页请求重做极性标定 => 复位 calPhase");
        } else {
          Serial.println("[CAL] 极性标定需要先打开稳定器 —— 本次忽略");
        }
      }

      if (calPhase < 3 && g_stabOn) {
      if (!imuFresh) {
        if (calPhase == 0 && (now % 2000) < 10)
          Serial.println("[STAB] 等待 IMU 数据，暂时无法标定极性...");
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
        continue;
      }
      g_yawRateDps = 0.0f;
      if (calPhase == 0) {
        calYaw0        = measYaw;
        calAbsIntegral = 0;
        calT0          = now;
        calPhase       = 1;
        Serial.println("[STAB] 开始自动极性标定：炮塔会先正转约 1.2 秒，再转回原位，请勿阻挡");
      } else if (calPhase == 1) {
        g_yawRateDps = +YAW_CAL_RATE_DPS;

        calAbsIntegral += fabsf(g_gyroYawRate) * CONTROL_DT;
        if (now - calT0 >= YAW_CAL_MOVE_TIME_MS) {
          float d = measYaw - calYaw0;
          while (d >  180.0f) d -= 360.0f;
          while (d < -180.0f) d += 360.0f;
          if (fabs(d) < YAW_CAL_MIN_DELTA) {
            Serial.printf("[STAB] ✗ 极性标定失败：炮塔几乎没动（只变了 %.1f°）。\n", d);
            Serial.println("        请确认从机在线、步进使能、驱动供电；之后仍可用 YAW_STAB_SIGN 手工设定");
            calPhase = 3;
          } else {

            yawSign = (d > 0) ? 1.0f : -1.0f;
            calDelta = d;

            {
              const float expected = YAW_CAL_RATE_DPS * (YAW_CAL_MOVE_TIME_MS / 1000.0f);
              const float measured = calAbsIntegral;
              const float rr = (measured > 1.0f) ? (expected / measured) : 0.0f;
              Serial.printf("[STAB]   标度核对：|速率|积分应≈%.1f，实测 %.1f，比值 %.3f\n",
                            expected, measured, rr);
              Serial.printf("[STAB]     角度核对(参考)：应转%.1f°，实测%.1f°\n",
                            expected, fabsf(d));

              if (rr > 0.0f && (rr < 0.75f || rr > 1.33f)) {
                GYRO_SCALE *= rr;
                Serial.printf("[STAB]   ⚠ 标度不匹配：修正 %.3f 倍 => GYRO_SCALE = %.7f\n",
                              rr, GYRO_SCALE);
                Serial.printf("[STAB]     推算真实量程约 ±%.0f dps（1 LSB = %.5f dps）\n",
                              32767.0f * GYRO_SCALE, GYRO_SCALE);
              }
            }

            Serial.printf("[STAB] ✓ 极性标定完成：正转 %.1f° => YAW_STAB_SIGN = %+.0f\n", d, yawSign);
            calT0    = now;
            calPhase = 2;
          }
        }
      } else if (calPhase == 2) {
        float d = measYaw - calYaw0;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;

        const bool sameSign = ((d > 0) == (calDelta > 0));
        const float rate = (fabsf(d) < YAW_CAL_BACK_TOL) ? 0.0f
                           : (sameSign ? -YAW_CAL_RATE_DPS : +YAW_CAL_RATE_DPS);
        g_yawRateDps = rate;
        bool backEnough = (now - calT0 >= 300) && (fabsf(d) < YAW_CAL_BACK_TOL);
        bool timeout    = (now - calT0 >= YAW_CAL_BACK_TIME_MS);
        if (backEnough || timeout) {
          g_yawRateDps = 0.0f;
          calT0    = now;
          calPhase = 3;
          Serial.printf("[STAB] 回位结束（残差 %.1f°）\n", d);

          yawTarget = measYaw;
          yawInteg  = 0.0f;
          Serial.printf("[STAB] 目标已对齐回位位置 %.1f°，稳定器开始工作\n", yawTarget);
        }
      }

      {

        int16_t r01 = (int16_t)constrain(g_yawRateDps * 10.0f, -32000.0f, 32000.0f);
        ctrlPacket.targetAngle01deg = r01;

      ctrlPacket.maxSpeedDps      = (uint8_t)constrain((int)(g_rateCapDps / 8.0f + 0.5f), 1, 255);
        sendToSlave();
        lastRate01    = r01;
        lastYawSentMs = now;
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
      continue;
    }
  #endif

    if (imuFresh) {

      if (!yawMeasInit) {
        yawMeasCont = measYaw;

        yawMeasLP   = yawMeasCont;
        yawMeasLP2  = yawMeasCont;
        yawMeasInit = true;
      }
      float dmeas = measYaw - yawMeasCont;
      while (dmeas >  180.0f) dmeas -= 360.0f;
      while (dmeas < -180.0f) dmeas += 360.0f;
      yawMeasCont += dmeas;

      yawMeasLP  += YAW_MEAS_ALPHA  * (yawMeasCont - yawMeasLP);
      yawMeasLP2 += YAW_MEAS_ALPHA2 * (yawMeasLP  - yawMeasLP2);
      const float measF = yawMeasLP2;

      if (!yawArmed) {
        yawTarget = yawMeasCont;
        yawInteg  = 0;
        yawArmed  = true;
      }

      if (g_stabAlignReq) {
        g_stabAlignReq = 0;
        yawTarget = yawMeasCont;
        yawInteg  = 0.0f;
        Serial.printf("[STAB] 目标已对齐当前朝向 %.1f°（不再追陈旧目标）\n", yawTarget);
      }

      {
        static unsigned long s_offSince  = 0;
        static bool          s_iDecaying = false;
        static unsigned long s_decayT0   = 0;
        if (g_chassisTurning) {

          if (YAW_I_PRELOAD_EN && !lastChassisTurning && g_chassisTurnDps != 0.0f) {
            const float preload = -g_chassisTurnDps / YAW_CMD_TO_DPS;
            yawInteg = constrain(preload, -YAW_I_LIMIT, YAW_I_LIMIT);
            Serial.printf("[STAB] 开始转向 => 积分预置 %.1f（底盘 %.0f dps / %.2f）\n",
                          (double)yawInteg, (double)g_chassisTurnDps, (double)YAW_CMD_TO_DPS);
          }
          s_offSince  = 0;
          s_iDecaying = false;
          lastChassisTurning = true;
        } else {
          if (lastChassisTurning && s_offSince == 0) s_offSince = now;

          if (s_offSince != 0 && (now - s_offSince) >= 30UL) {
            if (!s_iDecaying) {
              s_iDecaying = true;
              s_decayT0   = now;
              Serial.printf("[STAB] 底盘停止转向 => 积分按 tau=150ms 衰减 %dms（原值 %.1f）\n",
                            (int)YAW_I_DECAY_MS, (double)yawInteg);
            }
            lastChassisTurning = false;
            s_offSince = 0;
          }
        }

        if (s_iDecaying) {
          if ((now - s_decayT0) < (unsigned long)YAW_I_DECAY_MS) {

            yawInteg -= yawInteg * (CONTROL_DT / 0.150f);
          } else {
            s_iDecaying = false;
          }
          if (fabsf(yawInteg) < 0.5f) { yawInteg = 0.0f; s_iDecaying = false; }
        }
      }

      #if YAW_BIAS_TRACK_EN
      if (g_stabOn && !g_chassisTurning) {
        static float    s_sameMs   = 0.0f;
        static int      s_lastSgn  = 0;
        static float    s_acc      = 0.0f;
        static uint32_t s_accN     = 0;
        static float    s_travel   = 0.0f;

        const float rc = g_yawRateDps - g_ffDps;
        const int   sgn = (rc >  YAW_BIAS_MIN_DPS) ? +1
                        : (rc < -YAW_BIAS_MIN_DPS) ? -1 : 0;
        if (sgn == 0 || sgn != s_lastSgn) {
          s_sameMs = 0.0f; s_acc = 0.0f; s_accN = 0;
        } else {
          s_sameMs += CONTROL_DT * 1000.0f;
          s_acc    += rc;
          s_accN++;
        }
        s_lastSgn = sgn;

        if (s_sameMs >= (float)YAW_BIAS_MIN_MS && s_accN > 0) {
          const float mean  = s_acc / (float)s_accN;
          float dLsb = (mean / GYRO_SCALE) * YAW_BIAS_STEP_GAIN;
          if (fabsf(s_travel + dLsb) <= YAW_BIAS_MAX_TRAVEL_LSB) {
            gyroBiasZ += dLsb;
            s_travel  += dLsb;
            Serial.printf("[IMU] 零偏跟踪 %+.2f LSB（同号 %.1fs，均值 %+.2f dps，累计 %+.1f）\n",
                          dLsb, s_sameMs / 1000.0f, mean, s_travel);
          }
          s_sameMs = 0.0f; s_acc = 0.0f; s_accN = 0;
        }
      }
      #endif

      const float sign = yawSign;

      float nx = 0.0f;
      if (abs(joyX) > JOY_DEADZONE) {
        nx = (float)joyX / 100.0f;
        if (nx >  1.0f) nx =  1.0f;
        if (nx < -1.0f) nx = -1.0f;
      }
      g_joyNx = nx;
      const bool joyYaw = (nx != 0.0f);

      if (joyYaw) yawTarget -= nx * YAW_JOY_RATE * CONTROL_DT;

      const float ffCmd = joyYaw
                          ? (-nx) * YAW_JOY_RATE * YAW_FF_PER_DPS * sign
                          : 0.0f;

      if (joyYaw) {
        if (yawTarget - measF >  YAW_MAX_LEAD) yawTarget = measF + YAW_MAX_LEAD;
        if (yawTarget - measF < -YAW_MAX_LEAD) yawTarget = measF - YAW_MAX_LEAD;
      }

      {
        const float gap = yawTarget - yawMeasCont;
        if (fabsf(gap) > 720.0f) {
          const float turns = floorf(gap / 360.0f);
          yawTarget -= turns * 360.0f;
        }
      }

      float err = (yawTarget - measF) * sign;
      dbgErr = err;
      g_calErr = err;

      if (joyYaw) { /* 冻结：不累积、不清零 */ }

      biasCalTick();

      // 静止锁存：见文件上面 YAW_QUIET_HOLD_EN 那段
      // 只吃「炮塔自己不动」的零偏漂移，摇杆一动或者底盘一转立刻交回环路
      if (YAW_QUIET_HOLD_EN && g_stabOn && !g_biasCalOff && (g_biasCalBusy == 0)) {
        static unsigned long s_quietSince = 0;
        if (!g_chassisTurning && (g_joyNx == 0.0f) && (fabsf(dbgErr) < YAW_QUIET_ERR_DEG)) {
          if (s_quietSince == 0) {
            s_quietSince = now;
          } else if (now - s_quietSince >= YAW_QUIET_MS) {
            yawTarget = yawMeasCont;
            yawInteg  = 0.0f;
          }
        } else {
          s_quietSince = 0;
        }
      }

      if (!g_stabOn || g_biasCalOff || (g_biasCalBusy != 0)) {

        yawInteg = 0;
        const int trim = (int)YAW_MAX_TRIM;

        if (g_joyNx != 0.0f) yawCmd = YAW_STOP - (int)(g_joyNx * trim);

        g_yawRateDps = YAW_STEER_SIGN * (-nx) * YAW_JOY_RATE;
        g_yawRateDps = constrain(g_yawRateDps, -g_rateCapDps, g_rateCapDps);

        g_yawOutMag = g_joyActive ? 1.0e6f : 0.0f;
      } else {

        {

          const float kpEff = joyYaw ? (YAW_KP * YAW_JOY_KP_SCALE) : YAW_KP;
          float pTerm = kpEff * err;

          float rateForD = g_gyroYawRateLP2;
          {
            const float mag = fabsf(rateForD);
            if (mag <= YAW_D_RATE_DEADBAND) {
              rateForD = 0.0f;
            } else {
              rateForD = (rateForD > 0.0f ? 1.0f : -1.0f) * (mag - YAW_D_RATE_DEADBAND);
            }
          }
          float dTerm = -YAW_KD * (rateForD * sign);

          float dLim = fmaxf(fabsf(pTerm) * YAW_KD_MAX_RATIO, YAW_D_LIM_FLOOR_RATIO * g_outMax);
          if (dTerm >  dLim) dTerm =  dLim;
          if (dTerm < -dLim) dTerm = -dLim;

          static float s_prevErrForRate = 0.0f;
          const float errPrev      = s_prevErrForRate;
          s_prevErrForRate         = dbgErr;
          const float errRateDps   = (dbgErr - errPrev) / CONTROL_DT;

          const bool closingFast = YAW_I_SLEW_GATE_OFF ? false :
              ((fabsf(errRateDps) > YAW_I_SLEW_GATE_DPS) &&
               (fabsf(dbgErr) < fabsf(errPrev)) &&
               ((dbgErr * errPrev) >= 0.0f));
          float integStep = (fabsf(err) <= YAW_I_ERR_GATE && !closingFast)
                            ? (YAW_KI * err * CONTROL_DT) : 0.0f;
          yawInteg += integStep;
          yawInteg  = constrain(yawInteg, -YAW_I_LIMIT, YAW_I_LIMIT);

          float out = pTerm + dTerm + yawInteg + ffCmd;

          if ((out > YAW_MAX_TRIM && integStep > 0.0f) ||
              (out < -YAW_MAX_TRIM && integStep < 0.0f)) {
            yawInteg -= integStep;
            yawInteg  = constrain(yawInteg, -YAW_I_LIMIT, YAW_I_LIMIT);
            out = pTerm + dTerm + yawInteg + ffCmd;
          }

          out = constrain(out, -YAW_MAX_TRIM, YAW_MAX_TRIM);
          yawCmd = (int)(YAW_STOP + out + (out >= 0.0f ? 0.5f : -0.5f));

          {

            static float         s_ffSign  = 0.0f;
            static bool          s_ffRun   = false;
            static unsigned long s_ffT0    = 0;
            static float         s_ffYaw0  = 0.0f;
            float ffDps = 0.0f;

            static bool          s_turnDb      = false;
            static unsigned long s_turnDbSince = 0;
            {
              const bool raw = (g_chassisTurning && g_chassisTurnDps != 0.0f);
              if (raw != s_turnDb) {
                if (s_turnDbSince == 0) s_turnDbSince = now;

                else if (now - s_turnDbSince >= 40UL) { s_turnDb = raw; s_turnDbSince = 0; }
              } else {
                s_turnDbSince = 0;
              }
            }
            if (YAW_FF_ENABLE && s_turnDb && g_chassisTurnDps != 0.0f) {
              if (!s_ffRun) {
                s_ffRun  = true;
                s_ffT0   = now;
                s_ffYaw0 = yawMeasCont;
              } else if (s_ffSign == 0.0f && (now - s_ffT0) >= 200) {
                // 学极性这 200ms 里炮塔不能是被人为转着的
                // 不然 dYaw 里混进摇杆自己的转动 符号会学反 前馈一开就往反方向推
                const float dYaw = wrap180f(yawMeasCont - s_ffYaw0);
                if (g_joyNx != 0.0f) {

                  s_ffT0 = now;
                  s_ffYaw0 = yawMeasCont;
                } else if (fabsf(dYaw) > 1.0f) {

                  s_ffSign = ((dYaw * g_chassisTurnDps) > 0.0f) ? -1.0f : +1.0f;
                  Serial.printf("[STAB] 预启动极性已学到: 符号 %+.0f（命令 %.0f dps 时被拖 %.1f°）\n",
                                s_ffSign, g_chassisTurnDps, dYaw);
                }
              }

              if (s_ffSign != 0.0f) {

                const float ffScale = (g_speedMode == SPD_MODE_HIGH) ? YAW_FF_SCALE_HIGH
                                                                    : YAW_FF_SCALE_LOW;

                // 前馈幅度：默认用「命令 × k」，能实测就用实测（陀螺 − 极性×炮塔相对）
                float chsRate = g_chassisTurnDps;
                if (YAW_CHS_MEAS_EN && g_yawChsMeasOn && (g_joyNx == 0.0f)) {
                  const float wMeas = g_gyroYawRateLP - yawSign * g_yawRateDps;
                  g_chsMeasDps = wMeas;
                  const float r = fabsf(wMeas) / fmaxf(fabsf(g_chassisTurnDps), 1.0f);
                  if (r > YAW_CHS_MEAS_R_MIN && r < YAW_CHS_MEAS_R_MAX) {
                    chsRate = g_chassisTurnDps * r;
                  }
                } else {
                  g_chsMeasDps = 0.0f;
                }

                ffDps = s_ffSign * chsRate * ffScale;
              } else if (dbgFfBlkCnt < 20) {
                dbgFfBlkCnt++;
                Serial.println("[CHS] 前馈暂不出力：极性尚未学到（s_ffSign=0）");
              }
            } else {

              if (s_ffRun && s_ffSign == 0.0f) {
                Serial.printf("[STAB] 预启动未学到：转向持续 %.0f ms，期间偏移 %.2f°\n",
                              (float)(now - s_ffT0),
                              wrap180f(yawMeasCont - s_ffYaw0));
              }
              s_ffRun = false;
            }

            {
              static float s_ffLp = 0.0f;
              if (ffDps * s_ffLp < 0.0f) {
                s_ffLp = ffDps;
              } else {
                s_ffLp += YAW_FF_RISE_ALPHA * (ffDps - s_ffLp);
              }
              ffDps = s_ffLp;
            }

            float rateDps = out * YAW_CMD_TO_DPS;
            rateDps += ffDps;

            g_yawRateDps = constrain(rateDps, -g_rateCapDps, g_rateCapDps);
            g_ffDps = ffDps;
          }

          dbgP = pTerm; dbgD = dTerm; dbgI = yawInteg; dbgOut = out;
          g_yawOutMag = fabsf(out);
        }
      }
    } else {

      yawArmed = false;
      yawInteg = 0;

      const int trim = (int)YAW_MAX_TRIM;
      if (g_joyNx != 0.0f) yawCmd = YAW_STOP - (int)(g_joyNx * trim);

      g_yawRateDps = YAW_STEER_SIGN * (-g_joyNx) * YAW_JOY_RATE;
      g_yawRateDps = constrain(g_yawRateDps, -g_rateCapDps, g_rateCapDps);
    }
    yawCmd = constrain(yawCmd, 0, 180);

    {
      const float turretRateDps = (float)(yawCmd - YAW_STOP) * YAW_CMD_TO_DPS;
      g_turretAngleDeg += turretRateDps * CONTROL_DT;
    }

    #if STAB_DIAG
    if (now - lastStabDbgMs >= 250) {
      lastStabDbgMs = now;

      float targetW = yawTarget;
      while (targetW >  180.0f) targetW -= 360.0f;
      while (targetW < -180.0f) targetW += 360.0f;
      g_yawNow01 = (int)(yawMeasCont * 10.0f);
  g_yawTgt01 = (int)(yawTarget   * 10.0f);
Serial.printf("[STAB] yaw=%8.2f 目标w=%8.2f (连续%9.2f) err=%6.2f 陀螺=%6.2f(滤%6.2f) cmd=%3d\n",
                    measYaw, targetW, yawTarget, dbgErr, (float)g_gyroYawRate,
                    (float)g_gyroYawRateLP, yawCmd);
      Serial.printf("       P=%7.2f D=%7.2f I=%7.2f out=%7.2f\n",
                    dbgP, dbgD, dbgI, dbgOut);

      Serial.printf("       stab=%d imu=%d cal=%d sign=%+.0f  下发速率=%+.1f °/s  炮口θ=%+7.2f° 编码ω=%+7.2f dps\n",
                    (int)g_stabOn, (int)imuFresh, calPhase, yawSign,
                    (float)g_yawRateDps, (float)tzFrameDeg(), (float)s_tzRateDps);

      if (g_chassisTurning) {
        Serial.printf("       底盘自转 命令%+7.1f dps  实测%+7.1f dps  前馈%+6.1f  (实测%s)  炮口%+6.1f°\n",
                      (float)g_chassisTurnDps, (float)g_chsMeasDps, (float)g_ffDps,
                      (YAW_CHS_MEAS_EN && g_yawChsMeasOn) ? "开" : "关",
                      (float)g_turretRelDeg);
      }
    }
    #endif

    int16_t newRate01 = (int16_t)constrain(g_yawRateDps * 10.0f, -32000.0f, 32000.0f);

    bool changed = (abs((int)newRate01 - (int)lastRate01) >= (int)(YAW_SEND_MIN_STEP * 10));
    bool due     = (now - lastYawSentMs >= YAW_SEND_MIN_MS);
    if (changed ? due : (now - lastYawSentMs >= 250)) {
      int yawOut = yawCmd;
      if (REVERSE_YAW) yawOut = 180 - yawOut;
      ctrlPacket.yaw = (uint8_t)constrain(yawOut, 0, 180);

      ctrlPacket.targetAngle01deg = newRate01;

      ctrlPacket.maxSpeedDps      = (uint8_t)constrain((int)(g_rateCapDps / 8.0f + 0.5f), 1, 255);
      sendToSlave();
      lastYawCmd    = yawCmd;
      lastRate01    = newRate01;
      lastYawSentMs = now;
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
  }
}

void sendChassisCommand();

// 网页连上来之后 按键 摇杆 开关 校准按钮都从这里进来
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:

      Serial.printf("[WS] 客户端断开 => 清空所有按键、底盘停转（若紧跟 [CMD] 轮速归零 即为此因）\n");
      cmdForward = cmdBackward = cmdLeft = cmdRight = false;
      joyX = joyY = 0;
      cmdShoot = false;
      g_joyActive = false;
      g_shootFiring = false;
      sendChassisCommand();
      break;

    case WStype_CONNECTED:
      g_lastWsMs = millis();
      break;

    case WStype_TEXT: {
      g_lastWsMs = millis();
      String msg = String((char*)payload, length);
      int colon = msg.indexOf(':');
      if (colon > 0) {
        int    id  = msg.substring(0, colon).toInt();
        String val = msg.substring(colon + 1);

        if (id == 40) {

          g_speedMode = (val.toInt() != 0) ? SPD_MODE_HIGH : SPD_MODE_LOW;
          Serial.printf("[档位] %s\n", g_speedMode == SPD_MODE_HIGH ? "高速" : "低速");
          sendChassisCommand();
          sendToSlave();
        } else if (id == 41) {

          ctrlPacket.fl = ctrlPacket.fr = ctrlPacket.rl = ctrlPacket.rr = 0;
          sendToSlave();
        } else if (id == 42) {

          const bool newOn = (val.toInt() != 0);

          if (newOn && !g_stabOn) g_stabAlignReq = 1;
          g_stabOn = newOn;
        } else if (id == 43) {

          g_reqBiasCal = 1;
          Serial.println("[CAL] 网页请求重做零偏校准");
        } else if (id == 44) {

          g_reqPolarCal = 1;
          Serial.println("[CAL] 网页请求重做极性标定");
        } else if (id == 45) {

          tzZeroNow();
        } else if (id == 11) {

          g_layoutGan = (val.toInt() != 0);

          cmdForward = cmdBackward = cmdLeft = cmdRight = false;
          cmdStrafeLeft = cmdStrafeRight = false;
          sendChassisCommand();
        } else if (id == 10) {

          int c1 = val.indexOf(':');
          if (c1 > 0) {
            joyX = constrain(val.substring(0, c1).toInt(), -100, 100);
            joyY = constrain(val.substring(c1 + 1).toInt(), -100, 100);
          } else if (val.length() >= 4) {

            int jl = (val[2] == '1') ? 1 : 0;
            int jr = (val[3] == '1') ? 1 : 0;
            int ju = (val[0] == '1') ? 1 : 0;
            int jd = (val[1] == '1') ? 1 : 0;
            joyX = (jr - jl) * 100;
            joyY = (ju - jd) * 100;
          }
          g_lastJoyMs = millis();
          g_joyActive = (abs(joyX) > JOY_DEADZONE || abs(joyY) > JOY_DEADZONE);
        } else {
          int active = val.toInt();
          switch (id) {
            case 0: cmdForward  = active; break;
            case 1: cmdLeft     = active; break;
            case 2: cmdRight    = active; break;
            case 3: cmdBackward = active; break;
            case 4: cmdStrafeLeft  = active; break;
            case 5: cmdStrafeRight = active; break;
            case 8:

              g_shootFiring = (active != 0);
              break;
          }
          sendChassisCommand();
        }
      }
      break;
    }
  }
}

const char* html PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, minimum-scale=1, user-scalable=no, viewport-fit=cover">
<style>
  /* ==================== 禁止一切缩放 / 浏览器手势（纯触屏）==================== */
  /* 【分两层做】
     ① viewport 元数据 —— 管移动端：不许缩放、不许双击放大
     ② CSS touch-action / overflow —— 管触摸手势：双指缩放、拖动滚动、下拉刷新
     键鼠适配已经整套摘掉了（不再拦 Ctrl+滚轮/快捷键），这个页面只给手机用。 */
  html, body {
    margin: 0; padding: 0;
    width: 100%; height: 100%;
    background: #1a1a1a; color: #fff; font-family: sans-serif;
    overflow: hidden;                 /* 不滚动 => 内容不会被挤到一边 */
    touch-action: none;               /* 禁双指缩放 / 双击缩放 / 拖动 */
    -webkit-user-select: none; user-select: none;
    -webkit-text-size-adjust: 100%;   /* 禁移动端字体自动放大 */
    text-size-adjust: 100%;
    -webkit-touch-callout: none;      /* 禁长按弹出菜单 */
    overscroll-behavior: none;        /* 禁下拉刷新 / 边缘回弹 */
    position: fixed; inset: 0;        /* 锁死视口：地址栏收放时布局不跳 */
  }
  .dpad { position: fixed; bottom: 22px; display: grid; grid-template-columns: 62px 62px 62px; grid-template-rows: 62px 62px 62px; gap: 10px; z-index: 10; }
  /* 【两套方向键共用同一套定位】(#dpad-gan / #dpad-cross 同时选中)
     两套都是 3×3 网格、格子尺寸相同，只是 #dpad-cross 空出四个角，
     所以切换时【整块区域的位置和大小不变】，只有键的分布变。
     切换由右上角 #layoutBtn 控制，见脚本里的 applyLayout()。 */
  /* 【为什么要有安全区】iPhone 横屏时"灵动岛"/刘海占据屏幕一侧，
   最左的按键会被它盖住。safe-area-inset-left 正是浏览器给出的
   该侧被占宽度，加上它就能自动避开；非 iPhone 上该值为 0，无副作用。
   另加 40px 基础内缩，让整块方向键更靠中间、离屏幕边缘更远。 */
#dpad-gan, #dpad-cross { left: calc(env(safe-area-inset-left, 0px) + 49px); }
  /* ==================== 【本版修复】禁止浏览器把控件触摸当成手势 ====================
     浏览器默认把触摸当作「缩放手势」候选。第二根手指一出现，它就会给第一根手指
     派发 pointercancel => 松掉已按住的转向键 => 车停转。
     touch-action:none 明确告诉浏览器：这块区域不要做手势，触摸全部交给页面。 */
  .ctrlBtn, #joystick, #attak, #layoutBtn {
    touch-action: none;
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    user-select: none;
  }
  .ctrlBtn { width: 62px; height: 62px; border-radius: 14px; background: rgba(41, 182, 246, 0.85); color: #fff; font-size: 15px; font-weight: bold; border: none; cursor: pointer; display: flex; align-items: center; justify-content: center; user-select: none; touch-action: none; box-shadow: 0 3px 5px rgba(0,0,0,0.3); transition: background 0.1s, transform 0.1s; }
  .ctrlBtn.pressed, .ctrlBtn:active { background: #0288d1; transform: scale(0.92); box-shadow: inset 0 2px 6px rgba(0,0,0,0.4); }
  /* 原地自转键（干字型那两个）单独上提 20px：
     自转 + 平移要两个拇指同时按，上提之后跟下面一排「左移/右移」拉开，不打架。
     用 position 相对位移而不是 transform，免得把 :active 的缩放盖掉。 */
  #dpad-gan button[data-id="1"], #dpad-gan button[data-id="2"],
  #dpad-cross button[data-id="1"], #dpad-cross button[data-id="2"] { position: relative; top: -20px; }
  .up { grid-column: 2; grid-row: 1; } .left { grid-column: 1; grid-row: 2; } .right { grid-column: 3; grid-row: 2; } .down { grid-column: 2; grid-row: 3; }

  /* ---- 控制栏：竖排，紧贴摇杆左侧 ----
     原来做成"横贯顶部一整条"，实测横屏仍然遮挡。根因是横屏可用高度比我估的小，
     而摇杆+方向键已经吃掉底部一大块，顶部再压一条就撞上了。
     改法：不再占整条顶部，而是【贴在摇杆左边】竖直堆叠 ——
       竖排占宽度小、且高度与摇杆同段，天然不会和摇杆/方向键抢地方。
     right 值 = 摇杆右边距 + 摇杆直径 + 间距，所以它永远紧挨着摇杆左缘。 */
  /* 【方法一改版】固定摇杆隐藏后，控制栏不能再贴着「摇杆左边」—— 那个位置正落在右半屏
     滑动区里，右拇指一滑就撞上去了。现在挪到【左半屏】、紧贴方向键右缘：
     左拇指够得到，右半屏整块让给瞄准。
     left = 方向键左边距(含安全区) + 方向键宽(3×62 + 2×10) + 间距(6)，必须与 .dpad 同步。
     间距取 6 而不是 10：360px 窄屏上 49+206+10+面板98 = 363 会顶出去，6 刚好留一口气。 */
  #topBar { position: fixed;
            left: calc(env(safe-area-inset-left, 0px) + 49px + 206px + 6px);
            bottom: 22px;
            height: 172px;                     /* 与方向键同段高度，视觉对齐 */
            display: flex; flex-direction: column; justify-content: flex-end;
            align-items: stretch; gap: 6px; z-index: 30; }
  #speedPanel { display: flex; flex-direction: column; gap: 5px;
                background: rgba(0,0,0,0.62); padding: 4px; border-radius: 8px;
                border: 1px solid #444; }
  #speedPanel button { padding: 7px 4px; border: none; border-radius: 6px; cursor: pointer;
                       font-size: 13px; font-weight: bold; color: #fff; background: #455a64;
                       width: 88px; transition: background .12s, transform .12s; }
  #speedPanel button:active { transform: scale(.95); }
  #speedPanel button.on { box-shadow: 0 0 0 2px #90caf9 inset; }
  #spdHigh.on { background: #d84315; }
  #spdLow.on  { background: #2e7d32; }
  #stabBtn    { background: #37474f; }
  #stabBtn.stabOn { background: #00838f; }
  /* 发射键：横屏时【贴屏幕顶端 + 靠右】，右缘与摇杆右缘对齐。
     右缘用与摇杆相同的边距值(22px)，两者在同一竖直线上，视觉成一列。
     它和摇杆只是共用右边距，并不互相依赖尺寸 —— 所以不会被摇杆顶到半空。 */
  #attak { position: fixed;
           top: 0; right: calc(env(safe-area-inset-right, 0px) + 22px);
           width: 132px; height: 48px;
           border-radius: 0 0 10px 10px;      /* 贴顶边，上面两角不圆 */
           font-size: 17px; letter-spacing: 0; box-sizing: border-box; z-index: 40; }

  /* ---- 布局切换键：右上角，贴在发射键正下方（原指南针所在位置）----
     作用：在「现版：干字型六键」与「原版：十字四键」之间切换。
     顶端 0~48px 已被发射键占用，所以从 56px 起、与发射键同宽同右缘。 */
  #layoutBtn { position: fixed; top: 56px;
               right: calc(env(safe-area-inset-right, 0px) + 22px);
               width: 132px; height: 38px; z-index: 35;
               border: 1px solid #444; border-radius: 10px;
               background: rgba(0,0,0,0.62); color: #4fc3f7;
               font-size: 14px; font-weight: bold; cursor: pointer;
               box-sizing: border-box; touch-action: none;
               -webkit-user-select: none; user-select: none; }
  #layoutBtn:active { background: rgba(79,195,247,0.28); border-color: #4fc3f7; }

  /* 从机链路小灯：绿=在线 黄=变慢 红=掉线（数据来自主机 100ms 一帧的 A: 消息） */
  #linkBadge { position: fixed; left: calc(env(safe-area-inset-left, 0px) + 49px); top: 8px;
               padding: 4px 9px; border-radius: 9px; font-size: 12px; font-weight: bold;
               background: rgba(0,0,0,0.6); border: 1px solid rgba(255,255,255,0.18);
               color: #9e9e9e; z-index: 55; pointer-events: none;
               -webkit-user-select: none; user-select: none; }
  #linkBadge.ok   { color: #69f0ae; border-color: rgba(105,240,174,0.55); }
  #linkBadge.lag  { color: #ffd54f; border-color: rgba(255,213,79,0.55); }
  #linkBadge.down { color: #ff5252; border-color: rgba(255,82,82,0.55); }



  /* ==================== 方法一：右半屏滑动瞄准的视觉层 ====================
     这三层都 pointer-events:none，纯提示，不吃触摸 —— 命令走脚本里 document 上的监听。
     为什么不做一个盖住右半屏的 div 来收触摸：那样会和发射键/控制栏抢事件，
     而且元素一旦被布局改动就漏触摸；用 document 级监听 + 坐标判定最稳。 */
  #aimZone { position: fixed; left: 50%; top: 0; right: 0; bottom: 0;
             border-left: 1px dashed rgba(41,182,246,0.16);
             pointer-events: none; z-index: 5;
             -webkit-user-select: none; user-select: none; }
  #aimZone::after { content: '右半屏触摸操控'; position: absolute;
                    right: 12px; top: 50%; transform: translateY(-50%);
                    font-size: 12px; letter-spacing: 1px;
                    color: rgba(79,195,247,0.30); }
  /* 按下点：大圈是锚点，小点跟着手指跑。松手一起淡掉。
     模拟摇杆模式下 JS 会把这个圈的直径撑成「满杆直径」= 2 × AIM_STICK_R_PX，
     圈边就是满杆位置，中心那个点是回中位置 —— 一眼看出推了多少。 */
  #aimAnchor { position: fixed; left: 0; top: 0; width: 66px; height: 66px;
               margin: -33px 0 0 -33px; border-radius: 50%;
               border: 1px solid rgba(41,182,246,0.70);
               background: radial-gradient(circle, rgba(41,182,246,0.14) 0%, rgba(41,182,246,0.02) 70%);
               box-shadow: 0 0 12px rgba(41,182,246,0.35) inset;
               pointer-events: none; opacity: 0; z-index: 58;
               transition: opacity .10s linear; }
  #aimAnchor::after { content: ''; position: absolute; left: 50%; top: 50%;
                      width: 8px; height: 8px; margin: -4px 0 0 -4px;
                      border-radius: 50%; background: rgba(41,182,246,0.45); }
  #aimDot { position: fixed; left: 0; top: 0; width: 18px; height: 18px;
            margin: -9px 0 0 -9px; border-radius: 50%;
            background: rgba(41,182,246,0.80);
            box-shadow: 0 0 10px rgba(41,182,246,0.85);
            pointer-events: none; opacity: 0; z-index: 59;
            transition: opacity .10s linear; }

  #joystick { position: fixed; right: calc(env(safe-area-inset-right, 0px) + 22px); bottom: 22px; width: 172px; height: 172px; border-radius: 50%; background: radial-gradient(circle at 50% 50%, rgba(41,182,246,0.22) 0%, rgba(41,182,246,0.10) 65%, rgba(41,182,246,0.04) 100%); border: 2px solid rgba(41,182,246,0.45); box-shadow: inset 0 0 24px rgba(41,182,246,0.18), 0 4px 14px rgba(0,0,0,0.45); z-index: 10; touch-action: none; }
  #joystick::after { content: ''; position: absolute; inset: 20px; border-radius: 50%; border: 1px dashed rgba(41,182,246,0.22); pointer-events: none; }
  #joyKnob { position: absolute; left: 50%; top: 50%; width: 72px; height: 72px; margin-left: -36px; margin-top: -36px; border-radius: 50%; background: rgba(41, 182, 246, 0.85); box-shadow: 0 4px 12px rgba(0,0,0,0.5), inset 0 2px 6px rgba(255,255,255,0.25); will-change: transform; pointer-events: none; z-index: 1; }
  #joyKnob.active { background: #0288d1; box-shadow: 0 0 18px rgba(2,136,209,0.9), inset 0 2px 6px rgba(255,255,255,0.25); }

  /* ==================== 方法一：固定摇杆隐藏 ====================
     右半屏整块交给「滑动瞄准」（见脚本里 AIM_PAD_EN 那段）。摇杆元素留着不删 ——
     它连同上面那套 CSS 一起当方法二的备用件，切回去时只要把下面这一行删掉就行。 */
  #joystick { display: none; }

  /* 横屏矮屏适配。
     【思路变了】以前矮屏是"尽量缩小"以免遮挡；现在方向控件要【放大】，
     所以这里不再往下缩，只做【适度收紧】—— 最小值仍比原来的 46px 大。
     摇杆与方向键是手指操作，宁可略占地方也不能小到按不准。 */
  @media (max-height: 420px) {
    /* 摇杆：172 -> 150（仍大于原来的 132）；右边距含安全区 */
    #joystick { width: 150px; height: 150px; bottom: 14px;
                right: calc(env(safe-area-inset-right, 0px) + 18px); }
    #joyKnob { width: 64px; height: 64px; margin-left: -32px; margin-top: -32px; }
    /* 与 .dpad 同步：left = 安全区 + 37 + 方向键宽(3×54+2×8=178) + 8 */
    #topBar { left: calc(env(safe-area-inset-left, 0px) + 37px + 178px + 8px);
              bottom: 14px; height: 150px; gap: 5px; }
    #speedPanel button { padding: 6px 4px; font-size: 12px; width: 84px; }
    /* 发射键贴顶靠右，右边距跟摇杆一起改（含安全区） */
    #attak { right: calc(env(safe-area-inset-right, 0px) + 18px);
             width: 120px; height: 44px; font-size: 15px; }
    /* 布局切换键跟着发射键下移/缩放（矮屏发射键高 44，故从 50px 起） */
    #layoutBtn { top: 50px; right: calc(env(safe-area-inset-right, 0px) + 18px);
                 width: 120px; height: 34px; font-size: 13px; }

    /* 方向键：62 -> 54（仍明显大于原来的 46） */
    .dpad { bottom: 14px; grid-template-columns: 54px 54px 54px; grid-template-rows: 54px 54px 54px; gap: 8px; }
    /* 矮屏横屏时灵动岛更常见，内缩量略减但要避开安全区 */
    #dpad-gan, #dpad-cross { left: calc(env(safe-area-inset-left, 0px) + 37px); }
    .ctrlBtn { width: 54px; height: 54px; font-size: 13px; }
  }
</style>
</head>
<body>
  <!-- 控制栏：竖排，紧贴摇杆左侧（不再横贯顶部，避免横屏遮挡） -->
  <div id="topBar">
    <div id="speedPanel">
      <button id="spdHigh">高速</button>
      <button id="spdLow">低速</button>
      <button id="stabBtn">稳定器</button>
      <button id="calBootBtn" title="重做开机式陀螺零偏校准（约2秒，炮塔须静止）">零偏校准</button>
      <button id="calStabBtn" title="重做稳定器极性标定（炮塔会先正转约1.2秒再回原位）">极性校准</button>
      <button id="turretZeroBtn" title="炮口正对车头时按一下：把当前位置记成「正前」，自转+平移的方向就以它为基准">炮口正前</button>
    </div>
  </div>
  <button id="attak" class="ctrlBtn" data-id="8">发射</button>
  <!-- 布局切换键：切换方向键为「干字型六键」/「十字四键」 -->
  <button id="layoutBtn">方向键：干字型</button>

  <!-- ==================== 两套方向键（右上角键切换）====================
       两套【都留在 DOM 里】用 display 切换，不做动态生成 ——
       这样网页与固件的改动都最小，切换时也不用重连 WebSocket。
       两套共用同一套"按下动 / 松开停"机制（data-id 不变）。 -->
  <div id="dpad-gan" class="dpad">
    <!-- 现版：「干」字型六键（第一排三键 / 第二排左右两端 / 第三排中间） -->
    <button class="ctrlBtn" data-id="1" style="grid-area:1/1">原地左</button>
    <button class="ctrlBtn" data-id="0" style="grid-area:1/2">前</button>
    <button class="ctrlBtn" data-id="2" style="grid-area:1/3">原地右</button>
    <button class="ctrlBtn" data-id="4" style="grid-area:2/1">左移</button>
    <button class="ctrlBtn" data-id="5" style="grid-area:2/3">右移</button>
    <button class="ctrlBtn" data-id="3" style="grid-area:3/2">后</button>
  </div>
  <div id="dpad-cross" class="dpad" style="display:none">
    <!-- 原版：十字四键（前/后/原地左/原地右），无独立平移 -->
    <button class="ctrlBtn" data-id="0" style="grid-area:1/2">▲</button>
    <button class="ctrlBtn" data-id="1" style="grid-area:2/1">↺</button>
    <button class="ctrlBtn" data-id="2" style="grid-area:2/3">↻</button>
    <button class="ctrlBtn" data-id="3" style="grid-area:3/2">▼</button>
  </div>
  <!-- ==================== 方法一：右半屏滑动瞄准 ====================
       右半区随便按下去就能滑（横滑转炮塔 / 纵滑抬压炮口 / 斜滑两轴同时出力）。
       这三层都是 pointer-events:none 的提示层，真正的监听在脚本里（document 上的
       pointerdown / pointermove），坐标落在窗口右半屏才算命中。
       下面那个 #joystick 留在 DOM 里不删（只是 display:none）—— 它连同那套 CSS
       一起当方法二的备用件，切回去时删掉 CSS 里那行 display:none 就回来了。 -->
  <div id="aimZone"></div>
  <div id="aimAnchor"></div>
  <div id="aimDot"></div>
  <div id="joystick"><div id="joyKnob"></div></div>
<div id="linkBadge">从机 —</div>

<script>
(function () {
  'use strict';

  // ==================== 纯触屏版 ====================
  // 键鼠适配已全部摘掉：没有准星跟随、没有鼠标转头、没有左键开火、
  // 没有 Ctrl+滚轮/快捷键拦截，也没有鼠标拖摇杆。
  // 现在页面只认触摸（pointerType 必须是 touch/pen）和屏幕上的按钮。

  ['gesturestart', 'gesturechange', 'gestureend'].forEach(function (n) {
    document.addEventListener(n, function (e) { e.preventDefault(); }, { passive: false });
  });

  function isControlTarget(t) {
    return !!(t && t.closest && (t.closest('.ctrlBtn') || t.closest('#joystick') ||
                                 t.closest('#topBar') || t.closest('#speedPanel') ||
                                 t.closest('#attak') || t.closest('#layoutBtn')));
  }
  // 【方法一注意】右半屏滑动是「左手按方向键 + 右手滑」的双指操作。
  // 原来这两行会在双指时对非控件区域 preventDefault —— 有些 WebView/浏览器会因此
  // 给第一根手指派 pointercancel，把按住的转向键松掉。防缩放本来已经有
  // html/body 的 touch-action:none + viewport 兜住了，所以方法一启用时这里不再拦。
  document.addEventListener('touchstart', function (e) {
    if (!AIM_PAD_EN && e.touches.length > 1 && !isControlTarget(e.target)) e.preventDefault();
  }, { passive: false });
  document.addEventListener('touchmove', function (e) {
    if (!AIM_PAD_EN && e.touches.length > 1 && !isControlTarget(e.target)) e.preventDefault();
  }, { passive: false });

  let lastTapMs = 0, lastTapX = 0, lastTapY = 0;
  document.addEventListener('touchend', function (e) {
    const now = Date.now();
    const t = e.changedTouches && e.changedTouches[0];
    if (t) {
      if (now - lastTapMs < 300 && Math.abs(t.clientX - lastTapX) < 30 &&
          Math.abs(t.clientY - lastTapY) < 30) {
        e.preventDefault();
      }
      lastTapMs = now; lastTapX = t.clientX; lastTapY = t.clientY;
    }
  }, { passive: false });

  let ws = null;
  function wsOpen() { return ws && ws.readyState === WebSocket.OPEN; }
  function wsSend(s) { if (wsOpen()) ws.send(s); }

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen  = function () {
      console.log('[WS] connected');

      wsSend('11:' + (ganLayout ? 1 : 0));
    };
    ws.onmessage = function (ev) {
      const m = String(ev.data || '');
      if (m.charAt(0) !== 'A') return;
      const p = m.split(':');
      if (p.length < 3) return;
      if (p.length >= 4) linkState = parseInt(p[3], 10);
      if (p.length >= 5) linkAge   = parseInt(p[4], 10);
      if (p.length >= 6) frameDeg  = parseInt(p[5], 10) / 10;
      linkRedraw();
    };
    ws.onclose = function () { releaseAll(); setTimeout(connect, 1000); };
    ws.onerror = function () { };

  }
  connect();
  setInterval(function () { wsSend('ping'); }, 200);

  const activeCount = new Map();
  function acquire(id) { const n = (activeCount.get(id) || 0) + 1; activeCount.set(id, n); if (n === 1) wsSend(id + ':1'); return n; }
  function releaseById(id) { const n = (activeCount.get(id) || 1) - 1; if (n <= 0) { activeCount.delete(id); wsSend(id + ':0'); } else { activeCount.set(id, n); } }

  const btnPointers = new Map();
  function btnPress(pid, el) { if (!el) return; if (btnPointers.has(pid)) btnRelease(pid); const id = el.dataset.id; btnPointers.set(pid, { id: id, el: el }); el.classList.add('pressed'); acquire(id); }
  function btnRelease(pid) { const rec = btnPointers.get(pid); if (!rec) return; btnPointers.delete(pid); rec.el.classList.remove('pressed'); releaseById(rec.id); }
  function releaseAll() { Array.from(btnPointers.keys()).forEach(btnRelease); joyReset(); }

  let ganLayout = true;
  function applyLayout() {
    const gan = document.getElementById('dpad-gan');
    const crs = document.getElementById('dpad-cross');
    const btn = document.getElementById('layoutBtn');
    if (gan) gan.style.display = ganLayout ? 'grid' : 'none';
    if (crs) crs.style.display = ganLayout ? 'none' : 'grid';
    if (btn) btn.textContent = ganLayout ? '方向键：干字型' : '方向键：十字';

    wsSend('11:' + (ganLayout ? 1 : 0));
  }
  const layoutBtnEl = document.getElementById('layoutBtn');
  if (layoutBtnEl) {
    layoutBtnEl.addEventListener('click', function (ev) {
      ev.preventDefault();
      ev.stopPropagation();
      releaseAll();
      ganLayout = !ganLayout;
      applyLayout();
    });
  }
  applyLayout();
  function btnAt(x, y) { const el = document.elementFromPoint(x, y); return el ? el.closest('.ctrlBtn') : null; }

  document.addEventListener('pointerdown', function (e) { const el = e.target.closest ? e.target.closest('.ctrlBtn') : null; if (!el) return; e.preventDefault(); btnPress(e.pointerId, el); }, { passive: false });

  const BTN_SLIP_PX = 18;
  function outsideBy(el, x, y) {
    const r = el.getBoundingClientRect();
    const dx = Math.max(r.left - x, 0, x - r.right);
    const dy = Math.max(r.top - y, 0, y - r.bottom);
    return Math.max(dx, dy);
  }
  document.addEventListener('pointermove', function (e) {
    const rec = btnPointers.get(e.pointerId);
    if (!rec) return;
    e.preventDefault();
    const el = btnAt(e.clientX, e.clientY);
    if (el === rec.el) return;
    if (el) {
      btnRelease(e.pointerId);
      btnPress(e.pointerId, el);
      return;
    }
    if (outsideBy(rec.el, e.clientX, e.clientY) > BTN_SLIP_PX) {
      btnRelease(e.pointerId);
    }
  }, { passive: false });
  document.addEventListener('pointerup',     function (e) { btnRelease(e.pointerId); });
  document.addEventListener('pointercancel', function (e) { btnRelease(e.pointerId); });

  const joyEl = document.getElementById('joystick'), knobEl = document.getElementById('joyKnob');
  const MAX_TRAVEL = 0.62, TRIGGER_ZONE = 0.35, SEND_MIN_MS = 50;
  let joyCenter = { x: 0, y: 0 }, joyRadius = 0, activeId = null, lastJoyMsg = '0:0', lastSendTs = 0, pendingTimer = null;

  function joySendThrottled(s) { if (!wsOpen()) return; const now = Date.now(); const elapsed = now - lastSendTs; if (elapsed >= SEND_MIN_MS) { lastSendTs = now; wsSend('10:' + s); if (pendingTimer) { clearTimeout(pendingTimer); pendingTimer = null; } } else { if (pendingTimer) clearTimeout(pendingTimer); pendingTimer = setTimeout(function () { pendingTimer = null; lastSendTs = Date.now(); wsSend('10:' + lastJoyMsg); }, SEND_MIN_MS - elapsed); } }
  function joyForceStop() { if (pendingTimer) { clearTimeout(pendingTimer); pendingTimer = null; } lastJoyMsg = '0:0'; lastSendTs = Date.now(); wsSend('10:0:0'); }
  setInterval(function () { if (wsOpen() && !pendingTimer) wsSend('10:' + lastJoyMsg); }, 100);
  function joyMeasure() { const r = joyEl.getBoundingClientRect(); joyCenter = { x: r.left + r.width / 2, y: r.top + r.height / 2 }; joyRadius = r.width / 2; }

  function joyUpdateAnalog(kx, ky) {
    const maxPx = joyRadius * MAX_TRAVEL;
    let x = 0, y = 0;
    if (maxPx > 0) {
      x = Math.round(Math.max(-100, Math.min(100, kx / maxPx * 100)));
      y = Math.round(Math.max(-100, Math.min(100, -ky / maxPx * 100)));
    }
    const s = x + ':' + y;
    if (s !== lastJoyMsg) { lastJoyMsg = s; joySendThrottled(s); }
  }
  function joyUpdate(cx, cy) {
    const dx = cx - joyCenter.x, dy = cy - joyCenter.y;
    const dist = Math.hypot(dx, dy);
    const maxPx = joyRadius * MAX_TRAVEL;
    let kx = dx, ky = dy;
    if (dist > maxPx && dist > 0) { const k = maxPx / dist; kx = dx * k; ky = dy * k; }
    knobEl.style.transform = 'translate(' + kx + 'px, ' + ky + 'px)';
    joyUpdateAnalog(kx, ky);
  }
  function joyReset() { activeId = null; knobEl.classList.remove('active'); knobEl.style.transform = 'translate(0px, 0px)'; joyForceStop(); }
  function joyStart(cx, cy, id) { if (activeId !== null) return; activeId = id; joyMeasure(); knobEl.classList.add('active'); joyUpdate(cx, cy); }
  function joyMove(cx, cy, id) { if (activeId === id) joyUpdate(cx, cy); }
  function joyEnd(id) { if (activeId === id) joyReset(); }

  // 从机链路小灯用的状态（A: 消息的第 3、4、5 段）
  let linkState = -1, linkAge = 0;   // 从机链路：0 断 / 1 慢 / 2 在线
  let frameDeg = 0;                  // 炮口相对车体：自转+平移时「前」就是这个方向

  // 从机链路小灯：主机每 100ms 把状态塞在 A: 消息第 3、4 段里带过来
  function linkRedraw() {
    const el = document.getElementById('linkBadge');
    if (!el) return;
    const f = (frameDeg >= 0 ? '+' : '') + frameDeg.toFixed(0) + '°';
    if (linkState === 2) {
      el.textContent = '从机 在线 · 炮口 ' + f;
      el.className = 'ok';
    } else if (linkState === 1) {
      el.textContent = '从机 慢 ' + (linkAge > 60000 ? 60 : Math.round(linkAge / 100) / 10) + 's · 炮口 ' + f;
      el.className = 'lag';
    } else if (linkState === 0) {
      el.textContent = '从机 断开';
      el.className = 'down';
    } else {
      el.textContent = '从机 —';
      el.className = '';
    }
  }

  // ---- 固定摇杆（方法二备用件）的触摸操作。摇杆现在是 display:none，
  //      这里的监听只在切回方法二时才用得上；纯触屏，没有鼠标分支。----
  joyEl.addEventListener('touchstart', function (e) { if (activeId !== null) return; if (e.changedTouches.length === 0) return; const t = e.changedTouches[0]; e.preventDefault(); joyStart(t.clientX, t.clientY, t.identifier); }, { passive: false });
  document.addEventListener('touchmove', function (e) { if (activeId === null) return; for (let i = 0; i < e.changedTouches.length; i++) { const t = e.changedTouches[i]; if (t.identifier === activeId) { e.preventDefault(); joyMove(t.clientX, t.clientY, t.identifier); return; } } }, { passive: false });
  document.addEventListener('touchend', function (e) { if (activeId === null) return; for (let i = 0; i < e.changedTouches.length; i++) { const t = e.changedTouches[i]; if (t.identifier === activeId) { joyEnd(t.identifier); return; } } });
  document.addEventListener('touchcancel', function (e) { if (activeId === null) return; for (let i = 0; i < e.changedTouches.length; i++) { const t = e.changedTouches[i]; if (t.identifier === activeId) { joyEnd(t.identifier); return; } } });

  // ==================== 方法一：右半屏滑动瞄准（三角洲式）====================
  // 按下位置不受限：右半区随便按，按下点只当视觉锚点，不当摇杆圆心
  //   —— 三角洲那套就是这样，拇指在哪按下去不重要，重要的是往哪划。
  // 【与 C++ 联动】AIM_PAD_EN 两边必须一致：这里 false + C++ 改 0 = 完全退回方法二。
  const AIM_PAD_EN        = true;    // 方法一总开关

  // ==================== 横轴映射：三种，AIM_MAP_MODE 选 ====================
  // 'stick' = 模拟摇杆【当前启用】。以按下点为圆心，手指相对圆心偏了多少就是「推了多少」。
  //           推住不放，炮塔就一直按那个速度转；手指回圆心附近就停；松手回中。
  //           跟真摇杆一模一样的语义，只是圆心跟着手指走 —— 不用去瞄屏幕角落那个固定摇杆。
  // 'px'    = 位移映射【备用】。手指横滑多少像素 = 炮塔转多少度，跟划多快无关。
  // 'speed' = 速度映射【备用】。初版：手指划多快就给多大杆量。
  const AIM_MAP_MODE      = 'stick';

  // ---- 'stick' 模式用的 ----
  const AIM_STICK_R_PX      = 110;    // 横轴：偏离圆心多少 px 算满杆（屏幕窄时自动收，见 aimStickRadius）
  const AIM_STICK_R_Y_PX    = 90;     // 纵轴：满杆偏移，比横轴短一点（俯仰行程本来就小）
  const AIM_STICK_DEAD_PX   = 10;     // 圆心死区：手指在圆心附近抖不算推
  const AIM_STICK_PITCH     = true;   // true  = 纵轴也用偏移量（真摇杆语义：推住就一直抬）
                                      // false = 纵轴退回「划多快就抬多快」的速度映射

  // ★【'px' 模式】每滑 1 像素，炮塔转多少度。0.60 => 横滑 100px 转 60°。
  //   参考刻度（滑 100px 转多少）：初版速度映射 ≈ 11°，3 倍灵敏度 ≈ 33°，0.60 位移 ≈ 60°。
  //   上限提醒：主机满杆只给 120 dps，快甩的角度要靠记账兑现，超过 ~1.0 会明显「手停了还在追」。
  const AIM_DEG_PER_PX    = 0.60;    // 度 / 像素
  const AIM_RATE_MAX_DPS  = 120;     // 必须与 C++ 的 YAW_JOY_RATE 一致：满杆时的转速
  const AIM_BACKLOG_MAX_DEG = 60;    // 记账上限：60° 正好是 500ms 能追完的量（见 AIM_RELEASE_MS）
  const AIM_RELEASE_MS    = 500;     // 松手后最多再追 500ms，把欠账兑现完就停

  // ---- 下面这组只给 'speed' 模式和纵轴用，保留上一版数值 ----
  const AIM_SENS_X_SCALE = 3.0;      // 横轴灵敏度倍率：1.0 = 初版（1100 px/s 满杆）
  const AIM_SENS_PX_PER_S = 1100 / AIM_SENS_X_SCALE;   // 横轴满杆速度 ≈ 367 px/s
  const AIM_SENS_PX_PER_S_Y = 750;   // 纵轴不吃位移映射，仍走速度：抬手/压炮口幅度小
  const AIM_DEAD_PX_PER_S   = 90;    // 横轴速度死区（'speed' 模式）
  const AIM_DEAD_PX_PER_S_Y = 45;    // 纵轴速度死区

  const AIM_LP            = 0.50;    // 速度一阶低通
  const AIM_DECAY         = 0.35;    // 停手后速度每 tick 乘这个
  const AIM_STOP_MS       = 70;      // 停手多久开始往回收
  const AIM_TICK_MS       = 40;      // 发送节奏；主机 200Hz 在算，不用更快
  const AIM_SIDE_RATIO    = 0.50;    // 从窗口宽度的 50% 往右算「右半屏」

  const aimAnchorEl = document.getElementById('aimAnchor');
  const aimDotEl    = document.getElementById('aimDot');

  let aimPid = null;                 // 正在滑的那根手指（pointerId）
  let aimAnchorX = 0, aimAnchorY = 0;   // 摇杆圆心 = 按下点
  let aimCurX = 0, aimCurY = 0;         // 手指当前位置
  let aimStickRx = 110, aimStickRy = 90; // 本次按下实际用的满杆偏移（按屏幕尺寸钳过）
  let aimLastX = 0, aimLastY = 0, aimLastT = 0;
  let aimVX = 0, aimVY = 0;          // 平滑后的手指速度 px/s（速度映射用）
  let aimAccDx = 0;                  // 本 tick 内累计的横向像素位移（位移映射用）
  let aimBacklog = 0;                // 还没兑现的角度（度），正 = 还得往右转
  let aimTimer = null;
  let aimReleasing = false, aimReleaseT0 = 0;

  function aimInZone(x) {
    const w = window.innerWidth || document.documentElement.clientWidth || 0;
    return x >= w * AIM_SIDE_RATIO;
  }

  // 满杆偏移：按「按下点到最近屏幕边缘」自适应。
  // 靠边按下时半径自动收小，否则往外推根本推不满（手指出不了屏幕）。
  // 下限 60：半径再小的话一点点手抖就是大幅度，微调没法做。
  function aimStickRadius(pref, x, y, horizontal) {
    const w = window.innerWidth  || 390;
    const h = window.innerHeight || 700;
    const space = horizontal ? (Math.min(x, w - x) - 12) : (Math.min(y, h - y) - 12);
    return Math.max(60, Math.min(pref, space));
  }

  // 偏移 -> 杆量比例（-1..1）。死区做成「减掉」而不是「置零」：
  // 置零的话刚过死区就会从 0 直接跳到 死区/满杆 那一档，起步一顿一顿的。
  function stickAxis(d, full, dead) {
    const mag = Math.abs(d);
    if (mag <= dead) return 0;
    const span = Math.max(1, full - dead);
    return ((d > 0) ? 1 : -1) * Math.min(1, (mag - dead) / span);
  }

  // 纵轴速度映射（'stick' 模式关掉 AIM_STICK_PITCH 时、以及 'px'/'speed' 模式用）
  function aimPitchBySpeed(now) {
    if (aimPid === null) { aimVY = 0; return 0; }
    if (now - aimLastT > AIM_STOP_MS) {
      aimVY *= AIM_DECAY;
      if (Math.abs(aimVY) < 8) aimVY = 0;
    }
    const sgY = aimVY > 0 ? 1 : (aimVY < 0 ? -1 : 0);
    const vy  = sgY * Math.max(0, Math.abs(aimVY) - AIM_DEAD_PX_PER_S_Y);
    return -vy / (AIM_SENS_PX_PER_S_Y - AIM_DEAD_PX_PER_S_Y) * 100;
  }

  // 横轴速度映射（'speed' 模式用）
  function aimYawBySpeed(now) {
    if (aimPid === null) { aimVX = 0; return 0; }
    if (now - aimLastT > AIM_STOP_MS) {
      aimVX *= AIM_DECAY;
      if (Math.abs(aimVX) < 8) aimVX = 0;
    }
    const sgX = aimVX > 0 ? 1 : (aimVX < 0 ? -1 : 0);
    const vx  = sgX * Math.max(0, Math.abs(aimVX) - AIM_DEAD_PX_PER_S);
    return vx / (AIM_SENS_PX_PER_S - AIM_DEAD_PX_PER_S) * 100;
  }

  // 摇杆模式：把锚点圈撑到「满杆直径」，一眼看得出推到哪儿是满
  function aimVisualBase() {
    if (!aimAnchorEl) return;
    const R = (AIM_MAP_MODE === 'stick') ? aimStickRx : 33;
    aimAnchorEl.style.width  = (R * 2) + 'px';
    aimAnchorEl.style.height = (R * 2) + 'px';
    aimAnchorEl.style.marginLeft = (-R) + 'px';
    aimAnchorEl.style.marginTop  = (-R) + 'px';
  }

  function aimVisual(x, y, on) {
    if (aimAnchorEl) {
      aimAnchorEl.style.opacity = on ? '1' : '0';
      if (on) { aimAnchorEl.style.left = x + 'px'; aimAnchorEl.style.top = y + 'px'; }
    }
    if (aimDotEl) {
      aimDotEl.style.opacity = on ? '1' : '0';
      if (on) { aimDotEl.style.left = x + 'px'; aimDotEl.style.top = y + 'px'; }
    }
  }

  // 发出去：x 右为正（和原来「摇杆右推」同向），y 上为正（上滑抬炮口）
  function aimSend(r, p) {
    r = Math.round(Math.max(-100, Math.min(100, r)));
    p = Math.round(Math.max(-100, Math.min(100, p)));
    const s = r + ':' + p;
    if (s !== lastJoyMsg) { lastJoyMsg = s; joySendThrottled(s); }
  }

  function aimReleaseDone() {
    if (aimTimer) { clearInterval(aimTimer); aimTimer = null; }
    aimReleasing = false;
    aimBacklog = 0; aimAccDx = 0; aimVX = 0; aimVY = 0;
    joyForceStop();          // 立刻发 10:0:0，不等主机那 500ms 超时
  }

  function aimTick() {
    if (aimPid === null && !aimReleasing) return;
    const now = performance.now();
    if (aimPid === null && (now - aimReleaseT0) > AIM_RELEASE_MS) { aimReleaseDone(); return; }

    let r = 0, p = 0;

    if (AIM_MAP_MODE === 'stick') {
      // ===== 模拟摇杆：看「推多远」，不看「划多快」=====
      // 手指偏向哪边、偏多少，就是杆量大小；推住不放就一直按这个速度转。
      if (aimPid !== null) {
        r = stickAxis(aimCurX - aimAnchorX, aimStickRx, AIM_STICK_DEAD_PX) * 100;
        if (AIM_STICK_PITCH) {
          p = -stickAxis(aimCurY - aimAnchorY, aimStickRy, AIM_STICK_DEAD_PX) * 100;
        }
      }
      if (!AIM_STICK_PITCH) p = aimPitchBySpeed(now);

    } else if (AIM_MAP_MODE === 'px') {
      p = aimPitchBySpeed(now);
      // 1) 把这一 tick 手指横移的像素折成角度记账
      if (aimPid !== null) {
        aimBacklog += aimAccDx * AIM_DEG_PER_PX;
        aimAccDx = 0;
        if (aimBacklog >  AIM_BACKLOG_MAX_DEG) aimBacklog =  AIM_BACKLOG_MAX_DEG;
        if (aimBacklog < -AIM_BACKLOG_MAX_DEG) aimBacklog = -AIM_BACKLOG_MAX_DEG;
      }
      // 2) 一个 tick 物理上最多转这么多度（120 dps × 0.04 s = 4.8°），剩下的留在账上
      const capDeg = AIM_RATE_MAX_DPS * AIM_TICK_MS * 0.001;
      let step = aimBacklog;
      if (step >  capDeg) step =  capDeg;
      if (step < -capDeg) step = -capDeg;
      r = (capDeg > 0) ? (step / capDeg * 100) : 0;
      aimBacklog -= step;

    } else {
      p = aimPitchBySpeed(now);
      r = aimYawBySpeed(now);
    }

    // 松手后：欠账兑现完、纵向也归零了，彻底停
    if (aimPid === null && Math.abs(r) < 0.6 && Math.abs(p) < 0.6) { aimReleaseDone(); return; }

    aimSend(r, p);
  }

  function aimStart(pid, x, y) {
    if (aimTimer) { clearInterval(aimTimer); aimTimer = null; }
    aimPid = pid;
    aimReleasing = false;
    aimAnchorX = x; aimAnchorY = y;                 // 摇杆圆心 = 按下点（浮动摇杆）
    aimCurX = x;    aimCurY = y;
    aimStickRx = aimStickRadius(AIM_STICK_R_PX,   x, y, true);
    aimStickRy = aimStickRadius(AIM_STICK_R_Y_PX, x, y, false);
    aimLastX = x; aimLastY = y;
    aimLastT = performance.now();
    aimVX = 0; aimVY = 0;
    aimAccDx = 0; aimBacklog = 0;
    aimVisualBase();
    aimVisual(x, y, true);
    aimTimer = setInterval(aimTick, AIM_TICK_MS);
  }

  function aimMove(pid, x, y) {
    if (aimPid === null || pid !== aimPid) return;
    const now = performance.now();
    let dt = now - aimLastT;
    if (dt < 8) dt = 8;        // 采样太密按 8ms 算，别把瞬时速度算炸
    if (dt > 200) dt = 200;    // 卡了一帧就当慢速处理，别甩出一记满舵
    const dx = x - aimLastX, dy = y - aimLastY;
    aimCurX = x; aimCurY = y;                    // 摇杆模式看的是「相对圆心的位置」
    aimAccDx += dx;                              // 位移模式：像素直接记账，跟时间无关
    aimVX += AIM_LP * (dx * 1000 / dt - aimVX);  // 速度模式仍然要速度
    aimVY += AIM_LP * (dy * 1000 / dt - aimVY);
    aimLastX = x; aimLastY = y; aimLastT = now;
    if (aimDotEl) { aimDotEl.style.left = x + 'px'; aimDotEl.style.top = y + 'px'; }
  }

  function aimEnd(pid) {
    if (aimPid === null || pid !== aimPid) return;
    aimPid = null;
    aimVX = 0; aimVY = 0; aimAccDx = 0;
    aimVisual(0, 0, false);
    // 账上还有没兑现的角度：让它追一小段。否则快甩一下就被砍掉一大块，
    // 又变成「甩得越快转得越少」了。
    if (AIM_MAP_MODE === 'px' && Math.abs(aimBacklog) > 0.8 && aimTimer) {
      aimReleasing = true;
      aimReleaseT0 = performance.now();
    } else {
      aimReleaseDone();
    }
  }

  if (AIM_PAD_EN) {
    document.addEventListener('pointerdown', function (e) {
      if (aimPid !== null) return;                        // 只认第一根手指，第二根不管
      if (e.pointerType === 'mouse') return;              // 纯触屏版：鼠标事件一律不认
      if (isControlTarget(e.target)) return;              // 按钮/面板/方向键/发射键不抢
      if (!aimInZone(e.clientX)) return;                  // 只吃右半屏
      e.preventDefault();
      aimStart(e.pointerId, e.clientX, e.clientY);
    }, { passive: false });

    document.addEventListener('pointermove', function (e) {
      if (aimPid === null || e.pointerId !== aimPid) return;
      e.preventDefault();
      aimMove(e.pointerId, e.clientX, e.clientY);
    }, { passive: false });

    document.addEventListener('pointerup',     function (e) { aimEnd(e.pointerId); });
    document.addEventListener('pointercancel', function (e) { aimEnd(e.pointerId); });

    // 切窗口 / 息屏：别让最后那个速度值一直挂着转炮塔
    window.addEventListener('blur', function () { if (aimPid !== null) aimEnd(aimPid); });
    document.addEventListener('visibilitychange', function () {
      if (document.hidden && aimPid !== null) aimEnd(aimPid);
    });
  }

  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', function () { if (document.hidden) releaseAll(); });

  const $ = function (id) { return document.getElementById(id); };

  let stabOn = false;
  function setStab(on) {
    stabOn = !!on;
    wsSend('42:' + (stabOn ? 1 : 0));
    $('stabBtn').className = stabOn ? 'stabOn' : '';
    $('stabBtn').textContent = stabOn ? '稳定器 开' : '稳定器 关';
  }

  function flashBtn(el, text) {
    if (!el) return;
    const old = el.textContent;
    el.textContent = text;
    setTimeout(function () { el.textContent = old; }, 2000);
  }
  $('calBootBtn').onclick = function () {
    wsSend('43:1');
    flashBtn(this, '请保持静止…');
  };
  $('calStabBtn').onclick = function () {
    if (!stabOn) { alert('请先打开稳定器，再做极性校准'); return; }
    wsSend('44:1');
    flashBtn(this, '标定中…');
  };
  $('turretZeroBtn').onclick = function () {
    wsSend('45:1');
    flashBtn(this, '已标定');
  };

  let speedMode = -1;

  function setSpeedMode(mode, send) {
    if (mode === speedMode) return;
    speedMode = mode;
    if (send) wsSend('40:' + mode);
    $('spdHigh').className = (mode === 1) ? 'on' : '';
    $('spdLow').className  = (mode === 0) ? 'on' : '';
  }

  $('spdHigh').addEventListener('click', function () { setSpeedMode(1, true); });
  $('spdLow').addEventListener('click',  function () { setSpeedMode(0, true); });

  $('stabBtn').addEventListener('click', function () { setStab(!stabOn); });
  setStab(false);

})();</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", html); }

// 开机初始化 串口 陀螺 电机 WiFi 网页 两个任务
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BOOT (MASTER) ===");

  WiFi.softAP(ssid, password, 1, 0);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  ctrlServer.begin();
  ctrlServer.setNoDelay(true);

  ESP32PWM::allocateTimer(0);
  pitchServo.setPeriodHertz(50);

  pitchServo.setTimerWidth(16);
  pitchServo.attach(PITCH_SERVO_PIN, 500, 2500);
  pitchServo.write(PITCH_INIT_DEG);
  delay(500);

  initShootMotor();

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {
    g_imuOk = false;
    Serial.println("[WARN] BMI323 初始化失败，程序继续");
  } else {
    g_imuOk = true;
    Serial.println("[OK] BMI323 初始化成功");

    {
      const uint16_t gc = imu.readGyrConf();
      const unsigned range = (gc >> 4) & 0x07;
      const char* rn[5] = {"±125","±250","±500","±1000","±2000"};
      Serial.printf("[IMU] GYR_CONF 回读 = 0x%04X  => 量程 %s dps  (期望 0x70CB / ±2000)\\n",
                    gc, (range <= 4) ? rn[range] : "未知");
      if (gc != 0x70CB) {
        Serial.println("[IMU] ⚠ 量程与预期不符！库的 configure() 可能没生效");
      }
    }

    GYRO_SCALE     = 0.0610352f;
    g_rateCapDps   = 1200.0f;

    g_outMax = g_rateCapDps / YAW_CMD_TO_DPS;

    Serial.printf("[IMU] 最终配置: GYRO_SCALE=%.6f  转速上限=%.0f °/s  (量程约 ±%.0f dps)\n",
                  GYRO_SCALE, g_rateCapDps, 32767.0f * GYRO_SCALE);
    Serial.printf("[IMU] 输出可达上限=%.1f 度命令  => D 限幅 = |P|（与舵机版一致）\n",
                  g_outMax);

    calibrateGyro();

    {
      const float biasDps = gyroBiasZ * GYRO_SCALE;

      {
        int16_t mn = 32767, mx = -32768; int n = 0;
        for (int i = 0; i < 200; i++) {
          BMI323_SensorData t;
          if (imu.readData(t)) {
            if (t.gyrZ < mn) mn = t.gyrZ;
            if (t.gyrZ > mx) mx = t.gyrZ;
            n++;
          }
          delay(2);
        }
        const int spread = (n > 0) ? (mx - mn) : 0;
        Serial.printf("[IMU] 陀螺本底抖动: 极差 %d LSB（%d 个样本，min=%d max=%d）\n",
                      spread, n, mn, mx);
        if (spread < 2) {
          Serial.println("[IMU] ✗ 陀螺输出被钉死（本底抖动 < 2 LSB）—— 传感器没在测！");
          Serial.println("[IMU]   稳定器在此状态下不可用，请断电重启；仍不行则检查 I2C 接线");
        }

        if (spread > 60) {
          Serial.printf("[IMU] ⚠ 本底抖动偏大（%d LSB ≈ %.1f dps，正常约 5 LSB）\n",
                        spread, spread * GYRO_SCALE);
          Serial.println("[IMU]   请检查 I2C 接线/供电，并确认上电时炮塔静止、无振动");
        }
      }
      if (fabsf(biasDps) > 3.0f) {
        Serial.printf("[IMU] ⚠ 零偏异常 %.2f dps（正常应 < 5），说明校准时炮塔在动，重新校准...\n",
                      biasDps);
        delay(1200);
        calibrateGyro();
        const float b2 = gyroBiasZ * GYRO_SCALE;
        if (fabsf(b2) > 3.0f) {
          Serial.printf("[IMU] ✗ 零偏仍异常 %.2f dps —— 请确认开机时炮塔静止！\n", b2);
          Serial.println("[IMU]   稳定器可能失灵，建议断电后保持静止再上电");
        } else {
          Serial.printf("[IMU] ✓ 重校后零偏正常 %.2f dps\n", b2);
        }
      }
    }

    {
      BMI323_SensorData d;
      if (imu.readData(d)) {
        Serial.printf("[IMU] 自检 raw gyrZ=%d LSB  零偏=%.1f LSB  差值=%.1f LSB (=%.3f dps)\n",
                      d.gyrZ, gyroBiasZ, d.gyrZ - gyroBiasZ,
                      (d.gyrZ - gyroBiasZ) * GYRO_SCALE);
        Serial.println("[IMU] 差值应接近 0（静止）。若差值约等于零偏，说明传感器没在测。");
      }
    }
    filter.begin(200);
      }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  server.on("/", handleRoot);
  server.begin();

  if (g_imuOk) xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 21, NULL, 0);
  xTaskCreatePinnedToCore(ctrlTask, "Ctrl", 4096, NULL, 22, NULL, 0);

  #if STEP_TELEMETRY_EN
  Serial.println("[TM] 反向遥测接收已启用（等从机回传 [从机] 行）");
  #endif

  calKDumpFromNvs();
  tzLoad();
  g_calkBootEchoAt = millis() + 2000;
  if (g_calKStoreValid) {

    g_kDpsPerUnitLow  = g_calKStore.kLow;
    g_kDpsPerUnitHigh = g_calKStore.kHigh;
    Serial.printf("[CALK] 已启用存档 k：低档 %.2f  高档 %.2f（预启动前馈）\n",
                  (double)g_kDpsPerUnitLow, (double)g_kDpsPerUnitHigh);
  } else {
    Serial.printf("[CALK] 使用默认 k = %.2f（还没标定过）\n", (double)g_kDpsPerUnitLow);
  }
  Serial.println("=== MASTER READY ===");
}

// 主循环 串口命令 网页服务 发射 底盘 都在这儿转
void loop() {

  WiFiClient newCli = ctrlServer.available();
  if (newCli) {
    if (ctrlClient && ctrlClient.connected()) ctrlClient.stop();
    ctrlClient = newCli;
    ctrlClient.setNoDelay(true);
    s_lastTmMs = millis();     // 新连接给一个宽限窗口，别一上来就判掉线
    Serial.println("[LINK] 从机已连入，等第一帧遥测 ...");
  }
  if (ctrlClient && !ctrlClient.connected()) ctrlClient.stop();

  #if STEP_TELEMETRY_EN
  pollSlaveTelemetry();
  linkTick();
  #endif

  static unsigned long s_kArmMs = 0;
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch == 'k' || ch == 'K') {
      if (s_kArmMs != 0 && (millis() - s_kArmMs) <= 3000UL) {
        s_kArmMs = 0;
        g_calKReq = 1;
      } else {
        s_kArmMs = millis();
        Serial.println("[CAL-K] 已待命：3 秒内【再按一次 k】才会开始标定（底盘会强制转，遥控暂时无效）");
        Serial.println("[CAL-K] 不想标定请按任意其他键取消");
      }
    } else if (ch == 'x' || ch == 'X') {
      s_kArmMs = 0;
      g_calKReq = -1;
    } else if (ch == '<') {
      s_kArmMs = 0;
      if (g_shootPulseMs > 5UL) g_shootPulseMs -= 5UL;
      Serial.printf("[SHOOT] 周期=%lums 脉冲=%lums => 约 %.1f 次/秒 (按 > 加长脉冲)\n",
                    g_shootPeriodMs, g_shootPulseMs,
                    g_shootPeriodMs > 0 ? 1000.0 / (double)g_shootPeriodMs : 0.0);
    } else if (ch == '>') {
      s_kArmMs = 0;
      if (g_shootPulseMs + 5UL < g_shootPeriodMs) g_shootPulseMs += 5UL;
      Serial.printf("[SHOOT] 周期=%lums 脉冲=%lums => 约 %.1f 次/秒 (按 < 缩短脉冲)\n",
                    g_shootPeriodMs, g_shootPulseMs,
                    g_shootPeriodMs > 0 ? 1000.0 / (double)g_shootPeriodMs : 0.0);
    } else if (ch == 'd' || ch == 'D') {
      s_kArmMs = 0;
      calKDumpFromNvs();
      Serial.printf("[CALK] 当前生效: 低档 k=%.2f  高档 k=%.2f\n",
                    (double)g_kDpsPerUnitLow, (double)g_kDpsPerUnitHigh);
    } else if (ch == 'm' || ch == 'M') {
      s_kArmMs = 0;
      g_mixSpinLow = !g_mixSpinLow;
      Serial.printf("[MIX] 低速档「自转+平移」%s（高速档始终不支持）\n",
                    g_mixSpinLow ? "开" : "关");
    } else if (ch == 'v' || ch == 'V') {
      s_kArmMs = 0;
      g_yawChsMeasOn = !g_yawChsMeasOn;
      Serial.printf("[STAB] 底盘自转实测前馈 %s（关掉就退回 命令×k）\n",
                    g_yawChsMeasOn ? "开" : "关");
    } else if (ch == 'f' || ch == 'F') {
      s_kArmMs = 0;
      const float th   = tzFrameDeg();
      const float extr = th - s_tzMeasDeg;
      const unsigned long age = (s_tzMeasMs == 0) ? 0 : (millis() - s_tzMeasMs);
      Serial.println("[FRAME] ============== 基准角诊断 ==============");
      Serial.printf("  炮口θ   : %+8.2f°  = 实测 %+8.2f° + 外推 %+6.2f°\n",
                    (double)th, (double)s_tzMeasDeg, (double)extr);
      Serial.printf("  编码器  : raw=%u 零点=%u 帧龄=%lu ms 编码ω=%+7.2f dps\n",
                    (unsigned)g_turretRawPos, (unsigned)g_turretZeroRaw,
                    age, (double)s_tzRateDps);
      Serial.printf("  稳定器  : stab=%d err=%+6.2f° 陀螺=%+7.2f dps 下发=%+8.2f dps\n",
                    (int)g_stabOn, (double)g_calErr,
                    (double)g_gyroYawRate, (double)g_yawRateDps);
      Serial.printf("  底 盘   : 在转=%d 摇杆=%+.2f 混合开关=%d 链路=%d\n",
                    (int)g_chassisTurning, (double)g_joyNx,
                    (int)g_mixSpinLow, (int)g_linkState);
      Serial.println("[FRAME] ==========================================");
    } else if (ch == 'z' || ch == 'Z') {
      s_kArmMs = 0;
      tzZeroNow();
      Serial.printf("[FRAME] 当前炮口相对车体 %+.1f°\n", (float)g_turretRelDeg);
    } else {
      s_kArmMs = 0;
    }
  }
  #if CALK_EN
  calkTick();
  #endif

  if (g_calkBootEchoN < 3 && g_calkBootEchoAt != 0 &&
      (long)(millis() - g_calkBootEchoAt) >= 0) {
    g_calkBootEchoN++;
    Serial.printf("[CALK] ---- 开机补回放 %u/3（随时可按 'd' 再看）----\n",
                  (unsigned)g_calkBootEchoN);
    calKDumpFromNvs();

    const unsigned long gaps[3] = { 2000UL, 5000UL, 8000UL };
    g_calkBootEchoAt = (g_calkBootEchoN < 3)
                         ? millis() + gaps[g_calkBootEchoN] : 0;
  }

  if (g_shootFiring && webSocket.connectedClients() == 0) {
    g_shootFiring = false;
  }
  updateShootMotor(g_shootFiring);

  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();

  static bool wdArmed = false;
  if (g_lastWsMs != 0 && now - g_lastWsMs > 800) {
    if (!wdArmed) {
      wdArmed = true;
      cmdForward = cmdBackward = cmdLeft = cmdRight = false;
      joyX = joyY = 0;
      cmdShoot = false;
      g_shootFiring = false;
      g_joyActive = false;
      ctrlPacket.fl = ctrlPacket.fr = ctrlPacket.rl = ctrlPacket.rr = 0;
      sendChassisCommand();
    }
  } else {
    wdArmed = false;
  }

  if (g_joyActive && now - g_lastJoyMs > JOY_TIMEOUT_MS) {
    joyX = joyY = 0;
    g_joyActive = false;
  }

  static unsigned long lastAimSend = 0;
  if (now - lastAimSend >= 100) {
    lastAimSend = now;
    if (webSocket.connectedClients() > 0) {
      char abuf[56];
      snprintf(abuf, sizeof(abuf), "A:%d:%d:%d:%lu:%d",
               (int)g_yawNow01, (int)g_yawTgt01,
               (int)(g_linkState), (unsigned long)g_linkAgeMs,
               (int)(tzFrameDeg() * 10.0f));
      webSocket.broadcastTXT(abuf);
    }
  }

  // 底盘命令实时重算 50Hz
  // 自转时「炮口相对车体」的角度一直在变（车体在转），平移方向必须跟着变
  // 以前只在按键那一刻算一次，方向是冻住的 —— 车体转过去以后就成了斜着推
  {
    static unsigned long s_chsTickMs = 0;
    if (now - s_chsTickMs >= 20UL) {
      s_chsTickMs = now;
      if (cmdForward || cmdBackward || cmdLeft || cmdRight ||
          cmdStrafeLeft || cmdStrafeRight) {
        sendChassisCommand();
      }
    }
  }

  static unsigned long lastHbSend = 0;
  if (now - lastHbSend >= 100) {
    lastHbSend = now;
    sendToSlave();
  }

  delay(2);
}

