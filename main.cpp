// main.cpp — Spine（SPI 从机 + CAN 桥）完整修复版
#include "mbed.h" //
#include "math_ops.h"
#include <cstring>
#include "leg_message.h" //

// 1: test one motor over CAN without an SPI master; 0: normal SPIne firmware.
#define SINGLE_MOTOR_CAN_TEST 0
#define TEST_CAN_BUS_DEFAULT 2  // STM32: PB8=RX, PB9=TX; module: PB8->TX, PB9->RX
#define TEST_MOTOR_ID_DEFAULT 1
#define TEST_COMMAND_PERIOD_US 2000

// Safe Jetson -> SPI -> STM32 -> CAN bring-up mode.  Keep the full two-leg SPI
// packet, but allow CAN traffic to reach only the selected physical motor.
#define SPI_SINGLE_MOTOR_BRINGUP 0
#define SPI_BRINGUP_CAN_BUS      2
#define SPI_BRINGUP_MOTOR_ID     1

// 文件概览：腿控板作为 SPI 从机接收主机命令并通过两路 CAN 驱动电机，同时返回状态。
// Google 风格：简要说明文件职责，便于快速理解模块边界。

// -------------------- 配置 --------------------
#define RX_LEN   66    // SPI 收包 16bit 字数
#define TX_LEN   66    // SPI 发包 16bit 字数

#define DATA_LEN 30    // 30 x 16bit = 60B（12 个 float + flags + 32-bit 校验）
#define CMD_LEN  66    // 66 x 16bit（32 个 u32 + 32-bit 校验）

#define CAN_ID   0x0

/// Value Limits（物理约束） ///
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -65.0f
#define V_MAX  65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -16.0f
#define T_MAX  16.0f

/// Joint Soft Stops（软限位） ///
#define A_LIM_P 0.70f
#define A_LIM_N -0.70f
#define H_LIM_P 2.00f
#define H_LIM_N -2.00f
// Knee is zero with the leg straight. The joint may flex to about 150 deg.
// The encoder is motor-side of the additional 1.5:1 belt reduction, so the
// corresponding motor-angle limit is 2.618 rad * 1.5 = 3.93 rad.
// These limits assume knee feedback increases while the leg is flexed.
#define K_LIM_P 3.93f
#define K_LIM_N -0.10f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f

#define STRICT_DROP_ON_CRC_FAIL 1   // CRC 失败是否直接丢弃命令
#define CUT_MOTOR_ON_BAD_SPI    1   // 连续 CRC 失败是否触发断电
#define SPI_BAD_STREAK_MAX      5   // 触发断电的连续失败阈值
#define UART_LOG_PERIOD_MS      200 // 5 Hz: readable without blocking control too much
#define CAN_FEEDBACK_TIMEOUT_MS 100

// -------------------- 全局对象 --------------------
spi_data_t    spi_data;         // spine -> host：回传的两腿状态与校验
spi_command_t spi_command;      // host  -> spine：经校验后的最新指令
static spi_command_t spi_rx_shadow;    // SPI ISR 暂存的命令，待主循环搬运

static uint16_t rx_buff[RX_LEN];       // SPI 半字接收缓存
static uint16_t tx_buff[TX_LEN];       // SPI 半字发送缓存

DigitalOut led(PC_5);                   // 急停指示灯（亮=急停触发）
Serial     pc(PA_2, PA_3);              // 上位机调试串口

// 与硬件一致的 CAN 引脚
CAN can1(PB_12, PB_13, 1000000);        // CAN1：第一条腿
// API order: (STM32 RX, STM32 TX). User module wiring: PB8->TX, PB9->RX.
CAN can2(PB_8,  PB_9,  1000000);        // STM32 CAN1 on PB8/PB9

CANMessage rxMsg1, rxMsg2;              // CAN 反馈暂存
CANMessage a1_can, a2_can, h1_can, h2_can, k1_can, k2_can; // 六个关节的命令帧

InterruptIn cs(PA_4);                   // SPI 片选，下降沿触发 ISR
DigitalIn   estop(PB_15);               // 硬件急停输入（上拉，低有效）

leg_state   l1_state, l2_state;         // 两条腿当前状态
leg_control l1_control, l2_control;     // 两条腿待发送的控制量

volatile int enabled = 0;              // 当前是否已进入力矩模式
volatile uint32_t g_spi_frames = 0;    // SPI 已处理的帧数
volatile uint32_t g_cmd_bad    = 0;    // CRC 失败的帧数
volatile int      g_last_len   = 0;    // 上一帧实际接收的半字数
static volatile int spi_ready  = 0;    // SPI 是否完成初始化
static volatile int g_cmd_pending = 0; // 是否有待处理的 SPI 命令帧
static volatile int g_need_cut_motors = 0; // 是否需要立即退出力矩模式
static volatile int g_crc_bad_streak = 0;  // 连续 CRC 失败次数
static volatile int counter2 = 0;      // 站立模式计数器（预留）
static volatile int is_standing = 0;   // 站立模式标志位

// Per-leg/per-motor diagnostics. Index 0/1/2 corresponds to CAN ID 1/2/3.
static uint32_t g_can_rx_count[2][3] = {{0}};
static uint32_t g_can_last_rx_ms[2][3] = {{0}};
static bool     g_can_seen[2][3] = {{false}};
static uint32_t g_can_bad_id = 0;
static uint32_t g_can_bad_len = 0;
static uint32_t g_can_tx_ok = 0;
static uint32_t g_can_tx_fail = 0;

// 将 TX 缓冲区的第一个 16bit 预写入 SPI DR，确保片选拉低时立即输出。
static inline void spi_prime_first_word(void){
  if(!spi_ready) return;
  SPI1->DR = tx_buff[0];
}

// 计算 n 个 32-bit 字的逐字 XOR，用于 SPI 命令校验。
static inline uint32_t xor_checksum_u32(const uint32_t* p, size_t n){
  uint32_t s = 0;
  for(size_t i = 0; i < n; i++) {
    s ^= p[i];
  }
  return s;
}

// 根据 TX buffer 前 30 个 16-bit 半字生成 32-bit 序列（低 16bit 在前）并计算 XOR 校验。
static inline uint32_t checksum_from_tx_words30(const uint16_t* w16){
  uint32_t s = 0;
  for (int i = 0; i < 14; i++){
    uint32_t lo = w16[2*i + 0];
    uint32_t hi = w16[2*i + 1];
    s ^= (hi << 16) | lo;  // 与 Jetson 端 calc_lohi 一致
  }
  return s;
}

// ===== CAN 打包/解包 =====
// 将单关节控制量压缩为 8 字节 CAN 帧，遵循电机驱动的打包协议。
void pack_cmd(CANMessage * msg, joint_control joint){
  // 期望值按物理上限限幅，防止驱动接收异常值。
  float p_des = math_ops_fminf(math_ops_fmaxf(P_MIN, joint.p_des), P_MAX);
  float v_des = math_ops_fminf(math_ops_fmaxf(V_MIN, joint.v_des), V_MAX);
  float kp    = math_ops_fminf(math_ops_fmaxf(KP_MIN, joint.kp),  KP_MAX);
  float kd    = math_ops_fminf(math_ops_fmaxf(KD_MIN, joint.kd),  KD_MAX);
  float t_ff  = math_ops_fminf(math_ops_fmaxf(T_MIN, joint.t_ff), T_MAX);

  uint16_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
  uint16_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
  uint16_t t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

  msg->data[0] = p_int>>8;
  msg->data[1] = p_int&0xFF;
  msg->data[2] = v_int>>4;
  msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  msg->data[4] = kp_int&0xFF;
  msg->data[5] = kd_int>>4;
  msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  msg->data[7] = t_int&0xff;
  msg->len     = 8;
}
// 将电机驱动返回的 8 字节状态帧解包到指定腿的状态结构体。
void unpack_reply(CANMessage msg, leg_state * leg){
  uint16_t id    = msg.data[0];
  uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
  uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
  uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];

  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  // 1:髋外展，2:髋，3:膝；写入对应腿状态。
  if(id==1){      leg->a.p = p; leg->a.v = v; leg->a.t = t; }
  else if(id==2){ leg->h.p = p; leg->h.v = v; leg->h.t = t; }
  else if(id==3){ leg->k.p = p; leg->k.v = v; leg->k.t = t; }
}

static void record_can_reply(CANMessage msg, leg_state *leg, int leg_index,
                             uint32_t now_ms) {
  if (msg.len < 6) {
    ++g_can_bad_len;
    return;
  }
  const int motor_id = msg.data[0];
  if (motor_id < 1 || motor_id > 3) {
    ++g_can_bad_id;
    return;
  }
  unpack_reply(msg, leg);
  const int joint_index = motor_id - 1;
  ++g_can_rx_count[leg_index][joint_index];
  g_can_last_rx_ms[leg_index][joint_index] = now_ms;
  g_can_seen[leg_index][joint_index] = true;
}

static bool can_write_logged(CAN& bus, CANMessage& msg) {
  const bool ok = bus.write(msg);
  if (ok) ++g_can_tx_ok;
  else ++g_can_tx_fail;
  return ok;
}

// 发一轮 6 条 CAN（两腿）
// 将每个关节的期望控制量打包，并设置对应的报文 ID。
void PackAll(){
  pack_cmd(&a1_can, l1_control.a);
  pack_cmd(&a2_can, l2_control.a);
  pack_cmd(&h1_can, l1_control.h);
  pack_cmd(&h2_can, l2_control.h);
  pack_cmd(&k1_can, l1_control.k);
  pack_cmd(&k2_can, l2_control.k);

  a1_can.id = a2_can.id = 0x1;
  h1_can.id = h2_can.id = 0x2;
  k1_can.id = k2_can.id = 0x3;
}
// 依次向两条 CAN 总线写入关节命令，150us 间隔避免突发阻塞。
void WriteAll(){
  can_write_logged(can1, a1_can); wait_us(150);
  can_write_logged(can2, a2_can); wait_us(150);
  can_write_logged(can1, h1_can); wait_us(150);
  can_write_logged(can2, h2_can); wait_us(150);
  can_write_logged(can1, k1_can); wait_us(150);
  can_write_logged(can2, k2_can); wait_us(150);
}

static CAN& bringup_can() {
  return (SPI_BRINGUP_CAN_BUS == 1) ? can1 : can2;
}

static CANMessage* bringup_message() {
  if (SPI_BRINGUP_CAN_BUS == 1) {
    if (SPI_BRINGUP_MOTOR_ID == 1) return &a1_can;
    if (SPI_BRINGUP_MOTOR_ID == 2) return &h1_can;
    return &k1_can;
  }
  if (SPI_BRINGUP_MOTOR_ID == 1) return &a2_can;
  if (SPI_BRINGUP_MOTOR_ID == 2) return &h2_can;
  return &k2_can;
}

static void send_bringup_special(uint8_t code) {
  CANMessage* msg = bringup_message();
  msg->id = SPI_BRINGUP_MOTOR_ID;
  msg->len = 8;
  for (int i = 0; i < 7; ++i) msg->data[i] = 0xFF;
  msg->data[7] = code;
  can_write_logged(bringup_can(), *msg);
}

static void WriteSelectedMotor() {
  CANMessage* msg = bringup_message();
  msg->id = SPI_BRINGUP_MOTOR_ID;
  can_write_logged(bringup_can(), *msg);
}

// 电机模式控制
// Zero: 发送 0xFF...0xFE，让驱动清零偏置（立即广播一次）。
void Zero(CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFE; msg->len=8; WriteAll(); }
// EnterMotorMode: 0xFF...0xFC，进入力矩/闭环模式。
void EnterMotorMode(CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFC; msg->len=8; }
// ExitMotorMode: 0xFF...0xFD，退出力矩模式并松刹车。
void ExitMotorMode (CANMessage * msg){ for (int i=0;i<7;i++) msg->data[i]=0xFF; msg->data[7]=0xFD; msg->len=8; }

#if SINGLE_MOTOR_CAN_TEST
static int test_bus = TEST_CAN_BUS_DEFAULT;
static int test_motor_id = TEST_MOTOR_ID_DEFAULT;
static float test_velocity = 0.0f;
static float test_kd = 1.0f;
static bool test_enabled = false;
static joint_state test_feedback = {0.0f, 0.0f, 0.0f};
static uint32_t test_tx_ok = 0;
static uint32_t test_tx_fail = 0;
static uint32_t test_rx_ok = 0;

static CAN& test_can() { return (test_bus == 1) ? can1 : can2; }

static void test_send_special(unsigned char code) {
  CANMessage msg;
  msg.id = test_motor_id;
  msg.len = 8;
  for (int i = 0; i < 7; ++i) msg.data[i] = 0xFF;
  msg.data[7] = code;
  if (test_can().write(msg)) ++test_tx_ok;
  else ++test_tx_fail;
}

static void test_send_velocity() {
  joint_control command;
  command.p_des = 0.0f;
  command.v_des = test_velocity;
  command.kp = 0.0f;
  command.kd = test_kd;
  command.t_ff = 0.0f;
  CANMessage msg;
  pack_cmd(&msg, command);
  msg.id = test_motor_id;
  if (test_can().write(msg)) ++test_tx_ok;
  else ++test_tx_fail;
}

static void test_read_feedback() {
  CANMessage msg;
  CAN& bus = test_can();
  while (bus.read(msg)) {
    if (msg.len < 6 || msg.data[0] != test_motor_id) continue;
    uint16_t p = (msg.data[1] << 8) | msg.data[2];
    uint16_t v = (msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t t = ((msg.data[4] & 0xF) << 8) | msg.data[5];
    test_feedback.p = uint_to_float(p, P_MIN, P_MAX, 16);
    test_feedback.v = uint_to_float(v, V_MIN, V_MAX, 12);
    test_feedback.t = uint_to_float(t, T_MIN, T_MAX, 12);
    ++test_rx_ok;
  }
}

static void test_print_help() {
  pc.printf("\n\rSingle motor CAN test, UART PA2/PA3 115200 8N1\n\r");
  pc.printf("m=enable, x=stop+disable, v <rad/s>, d <kd>\n\r");
  pc.printf("b <1|2>=CAN bus, i <1..3>=motor ID, z=zero, ?=help\n\r");
  pc.printf("Current: bus=%d id=%d v=%.3f kd=%.3f\n\r",
            test_bus, test_motor_id, test_velocity, test_kd);
}

static void test_process_line(char* line) {
  float value;
  int number;
  if (line[0] == 'm' && line[1] == '\0') {
    test_send_special(0xFC);
    wait_ms(100);
    test_enabled = true;
    pc.printf("OK enabled: bus=%d id=%d\n\r", test_bus, test_motor_id);
  } else if (line[0] == 'x' && line[1] == '\0') {
    test_velocity = 0.0f;
    if (test_enabled) { test_send_velocity(); wait_ms(20); }
    test_send_special(0xFD);
    test_enabled = false;
    pc.printf("OK stopped and disabled\n\r");
  } else if (sscanf(line, "v %f", &value) == 1) {
    test_velocity = math_ops_fminf(math_ops_fmaxf(value, V_MIN), V_MAX);
    pc.printf("OK velocity=%.3f rad/s\n\r", test_velocity);
  } else if (sscanf(line, "d %f", &value) == 1) {
    test_kd = math_ops_fminf(math_ops_fmaxf(value, KD_MIN), KD_MAX);
    pc.printf("OK kd=%.3f\n\r", test_kd);
  } else if (sscanf(line, "b %d", &number) == 1) {
    if (test_enabled) pc.printf("ERR: send x before changing bus\n\r");
    else if (number != 1 && number != 2) pc.printf("ERR: bus must be 1 or 2\n\r");
    else { test_bus = number; pc.printf("OK bus=%d\n\r", test_bus); }
  } else if (sscanf(line, "i %d", &number) == 1) {
    if (test_enabled) pc.printf("ERR: send x before changing ID\n\r");
    else if (number < 1 || number > 3) pc.printf("ERR: ID must be 1..3\n\r");
    else { test_motor_id = number; pc.printf("OK id=%d\n\r", test_motor_id); }
  } else if (line[0] == 'z' && line[1] == '\0') {
    if (test_enabled || test_velocity != 0.0f)
      pc.printf("ERR: zero requires disabled motor and velocity=0\n\r");
    else { test_send_special(0xFE); pc.printf("OK zero sent\n\r"); }
  } else if (line[0] == '?' && line[1] == '\0') {
    test_print_help();
  } else if (line[0] != '\0') {
    pc.printf("ERR unknown command; send ? for help\n\r");
  }
}
#endif


// 串口中断：处理调试键盘指令，快速切换电机模式或站立状态。
#if !SINGLE_MOTOR_CAN_TEST
void serial_isr(){
     /// 串口调试指令处理 ///
     while(pc.readable()){
        char c = pc.getc();
        //led = !led;
        // 调试快捷键：Esc 退出力矩，m 进入力矩，s 站立模式，z 零点校准。
        switch(c){
            case(27):
                //loop.detach();
                pc.printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&a1_can);
                ExitMotorMode(&a2_can);
                ExitMotorMode(&h1_can);
                ExitMotorMode(&h2_can);
                ExitMotorMode(&k1_can);
                ExitMotorMode(&k2_can);
                enabled = 0;
                break;
            case('m'):
                pc.printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&a2_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&h2_can);
                EnterMotorMode(&k1_can);
                EnterMotorMode(&k2_can);
                wait(.5);
                enabled = 1;
                //loop.attach(&sendCMD, .001);
                break;
            case('s'):
                pc.printf("\n\r standing \n\r");
                counter2 = 0;
                is_standing = 1;
                //stand();
                break;
            case('z'):
                pc.printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&a2_can);
                Zero(&h1_can);
                Zero(&h2_can);
                Zero(&k1_can);
                Zero(&k2_can);
                break;
            }
        }
        WriteAll();
        
    }
#endif

// 软限位：超出阈值时清零速度与比例增益，并用固定 KD+力矩将关节推回安全区。返回 1 表示已触发。
int softstop_joint(joint_state s, joint_control * c, float limit_p, float limit_n){
  if(s.p >= limit_p){
    c->v_des = 0; c->kp = 0; c->kd = KD_SOFTSTOP; c->t_ff += KP_SOFTSTOP*(limit_p - s.p); return 1;
  } else if(s.p <= limit_n){
    c->v_des = 0; c->kp = 0; c->kd = KD_SOFTSTOP; c->t_ff += KP_SOFTSTOP*(limit_n - s.p); return 1;
  }
  return 0;
}

static void print_motor_log(int leg_number, int motor_id, const char *joint_name,
                            joint_state state, uint32_t now_ms, bool is_knee) {
  const int leg_index = leg_number - 1;
  const int joint_index = motor_id - 1;
  const bool seen = g_can_seen[leg_index][joint_index];
  const uint32_t age_ms = seen ? now_ms - g_can_last_rx_ms[leg_index][joint_index]
                               : now_ms;
  const bool timed_out = enabled && (!seen || age_ms > CAN_FEEDBACK_TIMEOUT_MS);
  const char *status = timed_out ? "TIMEOUT" : (seen ? "OK" : "WAIT");
  const char *prefix = timed_out ? "[WARN] " : "       ";

  if (is_knee) {
    pc.printf("%sL%d ID%d %-4s p=%+.3f joint=%+.3f v=%+.3f t=%+.3f rx=%lu age=%lums %s\n\r",
              prefix, leg_number, motor_id, joint_name, state.p, state.p / 1.5f,
              state.v, state.t,
              (unsigned long)g_can_rx_count[leg_index][joint_index],
              (unsigned long)age_ms, status);
  } else {
    pc.printf("%sL%d ID%d %-4s p=%+.3f v=%+.3f t=%+.3f rx=%lu age=%lums %s\n\r",
              prefix, leg_number, motor_id, joint_name, state.p, state.v, state.t,
              (unsigned long)g_can_rx_count[leg_index][joint_index],
              (unsigned long)age_ms, status);
  }
}

static void print_full_diagnostics(uint32_t now_ms, uint32_t frame_delta,
                                   uint32_t bad_delta) {
  pc.printf("\n\r========== STM32 SPINE: 2 LEGS / 6 MOTORS ==========\n\r");
  pc.printf("SYS enabled=%d estop=%s | SPI frames=%lu (+%lu) bad=%lu (+%lu) len=%d\n\r",
            enabled, estop.read() ? "OK" : "ACTIVE",
            (unsigned long)g_spi_frames, (unsigned long)frame_delta,
            (unsigned long)g_cmd_bad, (unsigned long)bad_delta, g_last_len);
  print_motor_log(1, 1, "ABAD", l1_state.a, now_ms, false);
  print_motor_log(1, 2, "HIP",  l1_state.h, now_ms, false);
  print_motor_log(1, 3, "KNEE", l1_state.k, now_ms, true);
  print_motor_log(2, 1, "ABAD", l2_state.a, now_ms, false);
  print_motor_log(2, 2, "HIP",  l2_state.h, now_ms, false);
  print_motor_log(2, 3, "KNEE", l2_state.k, now_ms, true);
  pc.printf("LIMIT L1=0x%02lX L2=0x%02lX | CAN tx_ok=%lu tx_fail=%lu bad_id=%lu bad_len=%lu\n\r",
            (unsigned long)(spi_data.flags[0] & 0xFF),
            (unsigned long)(spi_data.flags[1] & 0xFF),
            (unsigned long)g_can_tx_ok, (unsigned long)g_can_tx_fail,
            (unsigned long)g_can_bad_id, (unsigned long)g_can_bad_len);
  if (!estop.read()) pc.printf("[EMERGENCY] ESTOP ACTIVE (PB15=0)\n\r");
  if (bad_delta) pc.printf("[WARN] SPI rejected %lu frame(s) in this period\n\r",
                           (unsigned long)bad_delta);
  if (g_can_tx_fail) pc.printf("[WARN] CAN transmit failures detected: %lu\n\r",
                               (unsigned long)g_can_tx_fail);
}

// 控制主逻辑：根据最新的 spi_command 生成 l1/l2 控制，把反馈写入 spi_data，并构造 tx_buff。
// 步骤：1) 根据 flags 切换力矩模式；2) 将 CAN 反馈写入 spi_data；
//      3) 急停时清零并打出标志；4) 正常时填充主机期望并做软限位；5) 打包 + 校验到 tx_buff。
void control_and_build_tx(){
  // 入/退力矩模式
  const uint32_t torque_en =
#if SPI_SINGLE_MOTOR_BRINGUP
      spi_command.flags[SPI_BRINGUP_CAN_BUS - 1] & 0x1;
#else
      (spi_command.flags[0] | spi_command.flags[1]) & 0x1;
#endif
  if((torque_en == 1) && (enabled==0)){
    enabled = 1;
#if SPI_SINGLE_MOTOR_BRINGUP
    send_bringup_special(0xFC);
#else
    EnterMotorMode(&a1_can); can_write_logged(can1, a1_can);
    EnterMotorMode(&a2_can); can_write_logged(can2, a2_can);
    EnterMotorMode(&k1_can); can_write_logged(can1, k1_can);
    EnterMotorMode(&k2_can); can_write_logged(can2, k2_can);
    EnterMotorMode(&h1_can); can_write_logged(can1, h1_can);
    EnterMotorMode(&h2_can); can_write_logged(can2, h2_can);
#endif
    pc.printf("[MOTOR] enter torque mode (estop=%d)\n\r", estop.read());
  } else if((torque_en == 0) && (enabled==1)){
    enabled = 0;
#if SPI_SINGLE_MOTOR_BRINGUP
    send_bringup_special(0xFD);
#else
    ExitMotorMode(&a1_can); can_write_logged(can1, a1_can);
    ExitMotorMode(&a2_can); can_write_logged(can2, a2_can);
    ExitMotorMode(&h1_can); can_write_logged(can1, h1_can);
    ExitMotorMode(&h2_can); can_write_logged(can2, h2_can);
    ExitMotorMode(&k1_can); can_write_logged(can1, k1_can);
    ExitMotorMode(&k2_can); can_write_logged(can2, k2_can);
#endif
    pc.printf("[MOTOR] exit torque mode\n\r");
  }

  // CAN -> spi_data（host 会做坐标还原）
  spi_data.q_abad[0] = l1_state.a.p;  spi_data.q_abad[1] = l2_state.a.p;
  spi_data.q_hip [0] = l1_state.h.p;  spi_data.q_hip [1] = l2_state.h.p;
  spi_data.q_knee[0] = l1_state.k.p;  spi_data.q_knee[1] = l2_state.k.p;
  spi_data.qd_abad[0]= l1_state.a.v;  spi_data.qd_abad[1]= l2_state.a.v;
  spi_data.qd_hip [0]= l1_state.h.v;  spi_data.qd_hip [1]= l2_state.h.v;
  spi_data.qd_knee[0]= l1_state.k.v;  spi_data.qd_knee[1]= l2_state.k.v;

  // 急停按下：清零控制输出，写入特殊标志并点亮 LED。
  if(estop==0){
    memset(&l1_control, 0, sizeof(l1_control));
    memset(&l2_control, 0, sizeof(l2_control));
    spi_data.flags[0] = 0xDEAD;  // 用特殊码提醒上位机：急停触发
    spi_data.flags[1] = 0xDEAD;
    led = 1;
  } else {
    // 正常工作：刷新期望、应用软限位，关闭 LED。
    led = 0;
    memset(&l1_control, 0, sizeof(l1_control));
    memset(&l2_control, 0, sizeof(l2_control));

    // 主机发送的期望值（按既定 SPI 协议顺序）。
    l1_control.a.p_des = spi_command.q_des_abad[0];
    l1_control.a.v_des = spi_command.qd_des_abad[0];
    l1_control.a.kp    = spi_command.kp_abad[0];
    l1_control.a.kd    = spi_command.kd_abad[0];
    l1_control.a.t_ff  = spi_command.tau_abad_ff[0];

    l1_control.h.p_des = spi_command.q_des_hip[0];
    l1_control.h.v_des = spi_command.qd_des_hip[0];
    l1_control.h.kp    = spi_command.kp_hip[0];
    l1_control.h.kd    = spi_command.kd_hip[0];
    l1_control.h.t_ff  = spi_command.tau_hip_ff[0];

    l1_control.k.p_des = spi_command.q_des_knee[0];
    l1_control.k.v_des = spi_command.qd_des_knee[0];
    l1_control.k.kp    = spi_command.kp_knee[0];
    l1_control.k.kd    = spi_command.kd_knee[0];
    l1_control.k.t_ff  = spi_command.tau_knee_ff[0];

    l2_control.a.p_des = spi_command.q_des_abad[1];
    l2_control.a.v_des = spi_command.qd_des_abad[1];
    l2_control.a.kp    = spi_command.kp_abad[1];
    l2_control.a.kd    = spi_command.kd_abad[1];
    l2_control.a.t_ff  = spi_command.tau_abad_ff[1];

    l2_control.h.p_des = spi_command.q_des_hip[1];
    l2_control.h.v_des = spi_command.qd_des_hip[1];
    l2_control.h.kp    = spi_command.kp_hip[1];
    l2_control.h.kd    = spi_command.kd_hip[1];
    l2_control.h.t_ff  = spi_command.tau_hip_ff[1];

    l2_control.k.p_des = spi_command.q_des_knee[1];
    l2_control.k.v_des = spi_command.qd_des_knee[1];
    l2_control.k.kp    = spi_command.kp_knee[1];
    l2_control.k.kd    = spi_command.kd_knee[1];
    l2_control.k.t_ff  = spi_command.tau_knee_ff[1];

    // 软限位 -> flags
    spi_data.flags[0]  = 0;
    spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
    spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N))<<1;
    spi_data.flags[0] |= (softstop_joint(l1_state.k, &l1_control.k, K_LIM_P, K_LIM_N))<<2;

    spi_data.flags[1]  = 0;
    spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
    spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N))<<1;
    spi_data.flags[1] |= (softstop_joint(l2_state.k, &l2_control.k, K_LIM_P, K_LIM_N))<<2;
  }

  // ===== 关键修复：按“线上半字流”计算校验，并回填到 tx_buff[28..29] =====
  // 先把有效负载(28 半字 = 14×u32)写入 tx_buff[0..27]
  for (int i = 0; i < 28; i++) {
    tx_buff[i] = ((uint16_t*)(&spi_data))[i];
  }
  // 基于“将要上线”的 28 半字计算 32-bit XOR
  uint32_t chk = checksum_from_tx_words30(tx_buff);
  tx_buff[28]  = (uint16_t)(chk & 0xFFFF);      // 低 16
  tx_buff[29]  = (uint16_t)(chk >> 16);         // 高 16
  spi_data.checksum = chk;                      // 仅作串口观测（线上以 tx_buff 为准）
  for (int i = 30; i < TX_LEN; i++) tx_buff[i] = 0;  // 清理尾部

  // 将首个 16bit 预写入 SPI1->DR，确保主机采到当前帧。
  spi_prime_first_word();

}

// ===== SPI 中断（全双工搬运）=====
// 片选为低期间：TXE 时塞下一个半字，RXNE 时收主机半字；结束后校验长度与 XOR。
// 校验通过则将 spi_rx_shadow 搬到命令缓冲并置 g_cmd_pending，交由主循环消费。
void spi_isr(void)
{
  int rx_i = 0;
  int tx_i = 1;  // tx_buff[0] 已由 spi_prime_first_word 预写入

  while (cs == 0) {
    uint32_t sr = SPI1->SR;

    if ((sr & SPI_SR_TXE) && (tx_i < TX_LEN)) {
      SPI1->DR = tx_buff[tx_i++];
    }
    if (sr & SPI_SR_RXNE) {
      if (rx_i < RX_LEN) {
        rx_buff[rx_i++] = SPI1->DR;
      } else {
        volatile uint16_t dump = SPI1->DR;
        (void)dump;
      }
    }
  }
  g_last_len = rx_i;

  // 处理溢出：读 DR + SR 清除 OVR 标志，避免下一帧异常。
  if (SPI1->SR & SPI_SR_OVR) {
    volatile uint16_t dump = SPI1->DR;
    (void)dump;
    dump = SPI1->SR;
    (void)dump;
  }
  // 校验 host 命令（按线上 32×u32 = 64 半字 + checksum 的前 32×u32 计算）
  const bool len_ok = (g_last_len == CMD_LEN);
  uint32_t calc_checksum = 0;
  if(len_ok){
    for(int i = 0; i < CMD_LEN; i++){
      ((uint16_t*)(&spi_rx_shadow))[i] = rx_buff[i];
    }
    calc_checksum = xor_checksum_u32((uint32_t*)rx_buff, 32);
  }

  // 严格 CRC：长度或校验失败则丢帧，必要时累计错误触发断电。
  if (!len_ok || (spi_rx_shadow.checksum != calc_checksum)) {
#if STRICT_DROP_ON_CRC_FAIL
    g_cmd_bad++;
#if CUT_MOTOR_ON_BAD_SPI
    if(++g_crc_bad_streak >= SPI_BAD_STREAK_MAX){
      g_need_cut_motors = 1;
    }
#endif
    // 不更新 spi_command/tx_buff，只返回，继续发送上一帧缓存。
    g_spi_frames++;
    spi_prime_first_word();
    return;
#else
    // 宽松模式：即便 CRC 失败也放行（仅调试用）。
#endif
  }

  g_crc_bad_streak = 0;

  // CRC pass: notify main loop to process command
  g_cmd_pending = 1;

  g_spi_frames++;
}

// ===== SPI 初始化 =====
// 配置 16bit Mode0 从机，绑定 CS 下降沿触发 ISR，并预写首个半字。
void init_spi(void){
  pc.printf("SPI Init ...\n\r");
  SPISlave *spi = new SPISlave(PA_7, PA_6, PA_5, PA_4); // MOSI, MISO, SCK, CS
  spi->format(16, 0);       // 16bit, Mode 0
  spi->frequency(5000000);  // 与 Jetson 保持 1MHz
  spi->reply(0xff);
  cs.fall(&spi_isr);
  spi_ready = 1;
  spi_prime_first_word();
  pc.printf("SPI Init done\n\r");
}

// ===== 主入口 =====
// 初始化串口/CAN/SPI，先构造一帧空数据，随后主循环依次：
// 1) 轮询 CAN 反馈；2) 处理 SPI 新命令（如有）；3) 定期输出统计。
#if !SINGLE_MOTOR_CAN_TEST
int main() {
  wait(1);  // 上电等待，确保驱动和主机稳定
  pc.baud(921600);
  pc.attach(&serial_isr, Serial::RxIrq); // 绑定串口接收中断，处理键盘指令（含 z -> zeroing）
  estop.mode(PullUp);
  // Keep SPI CS (PA4) at a defined HIGH level while the Jetson CS output is
  // idle or temporarily high-impedance during boot/reset.
  cs.mode(PullUp);

  // 允许所有标准帧，便于驱动通信与调试。
  can1.filter(0, 0, CANStandard, 0);
  can2.filter(0, 0, CANStandard, 0);

  memset(&tx_buff,   0, sizeof(tx_buff));
  memset(&spi_data,  0, sizeof(spi_data));
  memset(&spi_command, 0, sizeof(spi_command));
  memset(&spi_rx_shadow,0, sizeof(spi_rx_shadow));

  a1_can.len = a2_can.len = h1_can.len = h2_can.len = k1_can.len = k2_can.len = 8; // 关节命令帧 8 字节
  rxMsg1.len = rxMsg2.len = 6;  // 驱动反馈帧 6 字节

  a1_can.id = a2_can.id = 0x1;
  h1_can.id = h2_can.id = 0x2;
  k1_can.id = k2_can.id = 0x3;

  // 先构建一个初始 tx 帧，避免首帧全 0
  control_and_build_tx();

  // Wait for the active-low SPI CS to return HIGH, but never block startup
  // forever if the Jetson pinmux or SPI mode is temporarily holding CS LOW.
  uint32_t cs_wait_count = 0;
  while (cs.read() == 0 && cs_wait_count < 100000) {
    wait_us(10);
    cs_wait_count++;
  }
  if (cs.read() == 0) {
    pc.printf("WARNING: SPI CS PA4 is stuck LOW; check Jetson CS polarity/pinmux\n\r");
  } else {
    pc.printf("SPI CS PA4 ready HIGH\n\r");
  }
  init_spi();

  // Periodic full diagnostics over the ST-Link virtual COM port.
  Timer t; t.start();
  uint32_t last_ms = 0, last_frames = 0, last_bad = 0;

  while(1) {
    uint32_t now = t.read_ms();
    // 1) 轮询 CAN 反馈。
#if SPI_SINGLE_MOTOR_BRINGUP
    if (SPI_BRINGUP_CAN_BUS == 1) {
      if (can1.read(rxMsg1) && rxMsg1.data[0] == SPI_BRINGUP_MOTOR_ID)
        record_can_reply(rxMsg1, &l1_state, 0, now);
    } else {
      if (can2.read(rxMsg2) && rxMsg2.data[0] == SPI_BRINGUP_MOTOR_ID)
        record_can_reply(rxMsg2, &l2_state, 1, now);
    }
#else
    if (can2.read(rxMsg2)) record_can_reply(rxMsg2, &l2_state, 1, now);
    if (can1.read(rxMsg1)) record_can_reply(rxMsg1, &l1_state, 0, now);
#endif
    wait_us(50);

    // 2) CRC 连续异常时的保护：必要时立即退出力矩。
    if (g_need_cut_motors) {
      g_need_cut_motors = 0;
#if CUT_MOTOR_ON_BAD_SPI
      if (enabled) {
#if SPI_SINGLE_MOTOR_BRINGUP
        send_bringup_special(0xFD);
#else
        ExitMotorMode(&a1_can); can_write_logged(can1, a1_can);
        ExitMotorMode(&a2_can); can_write_logged(can2, a2_can);
        ExitMotorMode(&h1_can); can_write_logged(can1, h1_can);
        ExitMotorMode(&h2_can); can_write_logged(can2, h2_can);
        ExitMotorMode(&k1_can); can_write_logged(can1, k1_can);
        ExitMotorMode(&k2_can); can_write_logged(can2, k2_can);
#endif
        enabled = 0;
      }
#endif
    }

    // 3) 有新的 SPI 命令帧：原子搬运 shadow -> command，再刷新 TX/CAN。
    if (g_cmd_pending) {
      __disable_irq();
      spi_command = spi_rx_shadow;
      g_cmd_pending = 0;
      __enable_irq();

      control_and_build_tx();
      PackAll();
#if SPI_SINGLE_MOTOR_BRINGUP
      WriteSelectedMotor();
#else
      WriteAll();
#endif
    }

    // 4) Print all six motor feedback channels and communication health at 5 Hz.
    if (now - last_ms >= UART_LOG_PERIOD_MS) {
      uint32_t df = g_spi_frames - last_frames;
      uint32_t db = g_cmd_bad    - last_bad;
      last_frames = g_spi_frames;
      last_bad    = g_cmd_bad;
      last_ms     = now;

      print_full_diagnostics(now, df, db);
    }
  }
}
#else
int main() {
  wait(1);
  pc.baud(115200);
  estop.mode(PullUp);
  can1.filter(0, 0, CANStandard, 0);
  can2.filter(0, 0, CANStandard, 14);

  // Make the selected motor passive at startup; no motion command is sent yet.
  test_send_special(0xFD);

  test_print_help();
  pc.printf("PB15 ESTOP is active-low. Motor starts only after m.\n\r> ");

  Timer timer;
  timer.start();
  uint32_t last_command_us = timer.read_us();
  uint32_t last_status_ms = timer.read_ms();
  char line[48];
  int line_length = 0;

  while (1) {
    while (pc.readable()) {
      char c = pc.getc();
      if (c == '\r' || c == '\n') {
        if (line_length > 0) {
          line[line_length] = '\0';
          test_process_line(line);
          line_length = 0;
          pc.printf("> ");
        }
      } else if ((c == 8 || c == 127) && line_length > 0) {
        --line_length;
      } else if (line_length < (int)sizeof(line) - 1) {
        line[line_length++] = c;
      }
    }

    if (estop.read() == 0 && test_enabled) {
      test_velocity = 0.0f;
      test_send_velocity();
      wait_ms(10);
      test_send_special(0xFD);
      test_enabled = false;
      pc.printf("\n\rESTOP: motor disabled\n\r> ");
    }

    uint32_t now_us = timer.read_us();
    if (test_enabled &&
        (uint32_t)(now_us - last_command_us) >= TEST_COMMAND_PERIOD_US) {
      last_command_us = now_us;
      test_send_velocity();
    }

    test_read_feedback();
    uint32_t now_ms = timer.read_ms();
    if ((uint32_t)(now_ms - last_status_ms) >= 200) {
      last_status_ms = now_ms;
      pc.printf("STAT en=%d bus=%d id=%d cmd_v=%.3f p=%.3f v=%.3f t=%.3f rx=%lu tx=%lu fail=%lu estop=%d\n\r",
                test_enabled ? 1 : 0, test_bus, test_motor_id, test_velocity,
                test_feedback.p, test_feedback.v, test_feedback.t,
                (unsigned long)test_rx_ok, (unsigned long)test_tx_ok,
                (unsigned long)test_tx_fail, estop.read());
    }
    wait_us(50);
  }
}
#endif
