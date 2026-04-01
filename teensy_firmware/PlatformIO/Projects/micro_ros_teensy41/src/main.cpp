/*
 * dog_teensy — main.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * micro-ROS firmware for Teensy 4.1
 * Drives CubeMars AK45-36 brushless actuators via CAN bus (MIT mini-cheetah
 * protocol), reads the BNO085 9-DOF IMU, and reads the u-blox SAM-M10Q GPS.
 *
 * Hardware wiring:
 * ─────────────────────────────────────────────────────────────────────────────
 *   CAN1  TX=22, RX=23  →  SN65HVD230 transceiver  →  Front legs (FR + FL)
 *   CAN2  TX=1,  RX=0   →  SN65HVD230 transceiver  →  Rear  legs (RR + RL)
 *
 *   Motor IDs (set via CubeMars R-Link software):
 *     FR hip=111 FR sho=112 FR kne=113
 *     FL hip=114 FL sho=115 FL kne=116
 *     RR hip=120 RR sho=121 RR kne=122
 *     RL hip=117 RL sho=118 RL kne=119
 *
 *   Termination: 120Ω between CANH and CANL at both ends of each bus.
 *
 *   BNO085 IMU    →  Wire1  (SDA=17, SCL=16)  I2C address 0x4A
 *   SAM-M10Q GPS  →  Serial2 (RX=7,  TX=8)   9600 baud NMEA
 *   micro-ROS     →  USB Serial (native Teensy USB)
 *
 * Power:
 *   mjbots power_dist r4.5b motor outputs → AK45-36 motors (24 V)
 *   mjbots power_dist 5V output           → Teensy VIN
 *
 * ROS topics
 * ─────────────────────────────────────────────────────────────────────────────
 * Subscribed:
 *   /joint_angles    (std_msgs/Float32MultiArray, 12 floats)
 *     Motor position commands in RADIANS.
 *     Order: FR_hip, FR_sho, FR_kne, FL_hip, FL_sho, FL_kne,
 *            RR_hip, RR_sho, RR_kne, RL_hip, RL_sho, RL_kne
 *     Sent with g_kp / g_kd (runtime gains); vel_ff=0, t_ff=0.
 *
 *   /joint_gains     (std_msgs/Float32MultiArray, 2 floats)
 *     [kp (N·m/rad), kd (N·m·s/rad)] — updates g_kp / g_kd.
 *     Published by gait_node on every gait/state transition.
 *
 *   /can_calibrate   (std_msgs/UInt8)
 *     Motor ID → send "set zero position" MIT frame (permanent) to that motor.
 *
 *   /can_enable      (std_msgs/Bool)
 *     true  → enter MIT motor mode on all 12 motors
 *     false → exit  MIT motor mode on all 12 motors
 *
 * Published:
 *   /imu/data        (sensor_msgs/Imu)            — 50 Hz
 *   /imu/euler       (geometry_msgs/Vector3)       — 50 Hz (roll/pitch/yaw deg)
 *   /fix             (sensor_msgs/NavSatFix)        — 1 Hz
 *   /joint_states    (std_msgs/Float32MultiArray)  — updated on motor reply
 *     36 floats: position[12] (rad), velocity[12] (rad/s), current[12] (A)
 *
 * CubeMars AK45-36 MIT mini-cheetah CAN protocol
 * ─────────────────────────────────────────────────────────────────────────────
 * Standard 11-bit CAN frames. CAN ID = motor_id.
 *
 * Special commands (8 bytes each):
 *   Enter motor mode : 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
 *   Exit  motor mode : 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
 *   Set zero position: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE
 *
 * Command frame (host → motor, 8 bytes):
 *   Bits [63:48] pos_des   — 16-bit, maps [-P_MAX, +P_MAX] rad
 *   Bits [47:36] vel_des   — 12-bit, maps [-V_MAX, +V_MAX] rad/s
 *   Bits [35:24] kp        — 12-bit, maps [0, KP_MAX]
 *   Bits [23:12] kd        — 12-bit, maps [0, KD_MAX]
 *   Bits [11: 0] t_ff      — 12-bit, maps [-T_MAX, +T_MAX] Nm
 *
 * Response frame (motor → host, 8 bytes):
 *   data[0]       motor_id
 *   data[1:2]     position  — 16-bit
 *   data[3]       vel[11:4]
 *   data[4]       vel[3:0] | cur[11:8]
 *   data[5]       cur[7:0]
 *   data[6]       temperature (raw - 40 = °C)
 *   data[7]       error flags
 *
 * LED indicator:
 *   Slow blink (500 ms) — waiting for micro-ROS agent
 *   Solid ON            — agent connected
 */

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <geometry_msgs/msg/vector3.h>

#include <math.h>
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>

// ─────────────────────────────────────────────────────────────────────────────
// CAN CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────

#define CAN_BAUD   1000000UL   // 1 Mbps — AK45-36 default

// Motor ID table: index = flat motor index (0–11), value = CAN motor ID
// Order: FR_hip, FR_sho, FR_kne, FL_hip, FL_sho, FL_kne,
//        RR_hip, RR_sho, RR_kne, RL_hip, RL_sho, RL_kne
static const uint8_t MOTOR_ID[12] = { 111, 112, 113, 114, 115, 116, 120, 121, 122, 117, 118, 119 };

// CAN bus assignment per motor index (1 = CAN1 front, 2 = CAN2 rear)
// NOTE: During single-motor testing motor 122 is wired to the CAN1 transceiver.
//       Restore to { 1,1,1,1,1,1, 2,2,2,2,2,2 } when all 12 motors are wired.
static const uint8_t MOTOR_BUS[12] = { 1, 1, 1, 1, 1, 1,   // FR + FL → CAN1
                                        2, 2, 2, 2, 2, 2 }; // RR + RL → CAN2

// ─────────────────────────────────────────────────────────────────────────────
// ROS DOMAIN ID
// Must match ROS_DOMAIN_ID set in ~/.bashrc on the Jetson (currently 7).
// If you change ROS_DOMAIN_ID on the Jetson, update this and reflash.
// ─────────────────────────────────────────────────────────────────────────────
#define ROS_DOMAIN_ID_VAL  7

// ─────────────────────────────────────────────────────────────────────────────
// MIT MINI-CHEETAH PROTOCOL PARAMETERS (AK45-36)
// ─────────────────────────────────────────────────────────────────────────────

#define MIT_P_MAX    12.5f    // rad      — position range
#define MIT_V_MAX    45.0f    // rad/s    — velocity range
#define MIT_KP_MAX  500.0f    // N·m/rad  — position gain range
#define MIT_KD_MAX    5.0f    // N·m·s/rad— damping gain range
#define MIT_T_MAX    18.0f    // N·m      — torque range

// Default PD gains — used as startup values until /joint_gains is received.
// At runtime these are overridden per gait by the Jetson gait_node.
#define DEFAULT_KP   35.0f    // position gain  (N·m/rad)
#define DEFAULT_KD    1.5f    // damping gain   (N·m·s/rad)

// Runtime gains — written by gains_callback, read by send_joint_cmds.
static float g_kp = DEFAULT_KP;
static float g_kd = DEFAULT_KD;

// Soft joint limits (radians)
#define JOINT_MIN   -2.618f   // −150°
#define JOINT_MAX    2.618f   //  150°

// ─────────────────────────────────────────────────────────────────────────────
// HOMING SWEEP CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
// On every agent connection the firmware drives each joint slowly toward its
// mechanical end-stop, detects the stall, backs off to the sitting position,
// then calls set-zero so that position 0 = sitting for all subsequent MIT cmds.
//
// *** YOU MUST CONFIGURE HOME_DIR AND HOME_BACKOFF_RAD FOR YOUR ROBOT. ***
//
// HOME_DIR : direction to drive toward the mechanical stop.
//   +1 = drive toward +MIT_P_MAX,  -1 = drive toward -MIT_P_MAX,  0 = skip
//
// HOME_BACKOFF_RAD : how far to back off from the end-stop (OUTPUT shaft, rad)
//   to reach the sitting / zero position.  Always positive.
//   Measure once: with HOME_BACKOFF_RAD all zeros run a homing sweep; the
//   motors will stop at their end-stops.  Observe the sitting angle for each
//   joint (via /joint_states or RViz), set that value here, reflash.
//
// Motor order: FR_hip, FR_sho, FR_kne, FL_hip, FL_sho, FL_kne,
//              RR_hip, RR_sho, RR_kne, RL_hip, RL_sho, RL_kne
// ─────────────────────────────────────────────────────────────────────────────
static const int8_t HOME_DIR[12] = {
    0, 0, 0,    // FR hip, sho, kne  ← set to +1 or -1 per joint
    0, 0, 0,    // FL hip, sho, kne
    0, 0, 0,    // RR hip, sho, kne
    0, 0, 0,    // RL hip, sho, kne
};
static const float HOME_BACKOFF_RAD[12] = {
    0.0f, 0.0f, 0.0f,   // FR
    0.0f, 0.0f, 0.0f,   // FL
    0.0f, 0.0f, 0.0f,   // RR
    0.0f, 0.0f, 0.0f,   // RL
};

#define HOME_KP                 2.0f    // N·m/rad — gentle stiffness during seek
#define HOME_KD                 1.5f    // N·m·s/rad
#define HOME_SEEK_TARGET        14.0f   // rad — well past any physical stop
#define HOME_STALL_CUR_THRESH   4.0f    // A   — current above this = stalled
#define HOME_STALL_VEL_THRESH   0.15f   // rad/s — velocity below this = stalled
#define HOME_STALL_CONFIRM      8       // consecutive 20 ms ticks to confirm stall
#define HOME_SEEK_TIMEOUT_MS    10000UL // give up seeking after 10 s
#define HOME_BACKOFF_TIMEOUT_MS  5000UL // give up backing off after 5 s
#define HOME_BACKOFF_THRESH_RAD  0.05f  // within 0.05 rad = reached backoff target
#define HOME_ZERO_SETTLE_MS      80UL   // wait after set-zero for flash write

// ─────────────────────────────────────────────────────────────────────────────
// BNO085 IMU
// ─────────────────────────────────────────────────────────────────────────────
#define BNO085_ADDR   0x4A    // default; 0x4B if PS1 is pulled high
#define IMU_RATE_HZ   50
// Report interval in microseconds (must be ≤ 1 000 000 / IMU_RATE_HZ)
#define IMU_INTERVAL_US  (1000000 / IMU_RATE_HZ)

static Adafruit_BNO08x  bno08x;
static bool             bno_ok = false;

// Latest values from BNO085 — updated by bno_poll(), published by timer
static float bno_qw = 1.0f, bno_qx = 0.0f, bno_qy = 0.0f, bno_qz = 0.0f;
static float bno_ax = 0.0f, bno_ay = 0.0f, bno_az = 0.0f;
static float bno_gx = 0.0f, bno_gy = 0.0f, bno_gz = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// SAM-M10Q GPS
// ─────────────────────────────────────────────────────────────────────────────
#define GPS_BAUD   9600       // SAM-M10Q factory default
#define GPS_RATE_HZ  1        // GNSS fix rate (SAM-M10Q default: 1 Hz)

static TinyGPSPlus gps;

// Latest GPS state — updated in loop(), published by GPS timer
static double gps_lat       = 0.0;
static double gps_lon       = 0.0;
static double gps_alt       = 0.0;
static bool   gps_fix_valid = false;

// ─────────────────────────────────────────────────────────────────────────────
// HARDWARE OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

// ─────────────────────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────────────────────
static float joint_cmd[12] = {0.0f};   // desired motor positions (rad)
static bool  motors_enabled = false;

// Latest motor feedback — updated whenever a response frame arrives
static float fb_pos[12]  = {0.0f};  // rad
static float fb_vel[12]  = {0.0f};  // rad/s
static float fb_cur[12]  = {0.0f};  // A (proportional to torque)
static float fb_temp[12] = {0.0f};  // °C  (raw byte - 40)

// ── Homing state ──────────────────────────────────────────────────────────────
enum MotorHomePhase : uint8_t {
    MHOME_SEEK,      // driving toward end-stop
    MHOME_BACKOFF,   // backing away to sitting position
    MHOME_ZERO,      // waiting for set-zero flash write
    MHOME_DONE,      // complete
};
static MotorHomePhase  home_phase[12];
static float           home_stall_pos[12];        // fb_pos when stall detected
static float           home_backoff_tgt[12];      // fb_pos target after backoff
static uint8_t         home_stall_count[12];      // stall debounce counter
static unsigned long   home_seek_start_ms[12];
static unsigned long   home_backoff_start_ms[12];
static bool            home_zero_sent[12];
static unsigned long   home_zero_time_ms[12];
static unsigned long   home_last_tick_ms  = 0;    // 50 Hz homing tick

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS HANDLES
// ─────────────────────────────────────────────────────────────────────────────
rcl_node_t          node;
rclc_support_t      support;
rcl_allocator_t     allocator;
rclc_executor_t     executor;

rcl_subscription_t  angles_sub;
rcl_subscription_t  gains_sub;
rcl_subscription_t  calibrate_sub;
rcl_subscription_t  enable_sub;
rcl_publisher_t     imu_pub;
rcl_publisher_t     euler_pub;
rcl_publisher_t     fix_pub;
rcl_publisher_t     joint_states_pub;
rcl_publisher_t     homed_pub;
rcl_timer_t         imu_timer;
rcl_timer_t         gps_timer;

std_msgs__msg__Float32MultiArray angles_msg;
std_msgs__msg__Float32MultiArray gains_msg;
std_msgs__msg__UInt8             calibrate_msg;
std_msgs__msg__Bool              enable_msg;
std_msgs__msg__Bool              homed_msg;
sensor_msgs__msg__Imu            imu_msg;
geometry_msgs__msg__Vector3      euler_msg;
sensor_msgs__msg__NavSatFix      fix_msg;
std_msgs__msg__Float32MultiArray joint_states_msg;

float angles_buf[12];
float gains_buf[2];           // [kp, kd]
float joint_states_buf[48];   // pos[12] + vel[12] + cur[12] + temp[12]

// ─────────────────────────────────────────────────────────────────────────────
// AGENT STATE MACHINE
// ─────────────────────────────────────────────────────────────────────────────
enum AgentState { WAITING_AGENT, HOMING, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agent_state = WAITING_AGENT;

// ─────────────────────────────────────────────────────────────────────────────
// MIT PROTOCOL HELPERS
// ─────────────────────────────────────────────────────────────────────────────

static inline float constrain_f(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Map a float in [x_min, x_max] to an unsigned integer in [0, 2^bits - 1]
static inline uint32_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_val = (1u << bits) - 1;
    int32_t raw = (int32_t)(offset / span * (float)max_val);
    if (raw < 0)            raw = 0;
    if (raw > (int32_t)max_val) raw = (int32_t)max_val;
    return (uint32_t)raw;
}

// Map an unsigned integer in [0, 2^bits - 1] back to float in [x_min, x_max]
static inline float uint_to_float(uint32_t raw, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    uint32_t max_val = (1u << bits) - 1;
    return (float)raw / (float)max_val * span + x_min;
}

// Send a standard-frame CAN message on the given bus
static void can_send_std(uint8_t bus, uint8_t motor_id,
                         const uint8_t *data, uint8_t len) {
    CAN_message_t msg;
    memset(&msg, 0, sizeof(msg));  // zero all flags to avoid garbage bits
    msg.flags.extended = 0;   // standard 11-bit ID
    msg.flags.remote   = 0;
    msg.id  = motor_id;
    msg.len = len;
    for (int i = 0; i < len; i++) msg.buf[i] = data[i];

    if (bus == 1) can1.write(msg);
    else          can2.write(msg);
}

// Enter MIT motor mode
static void motor_enter_mode(uint8_t bus, uint8_t motor_id) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    can_send_std(bus, motor_id, d, 8);
}

// Exit MIT motor mode (safe / coast)
static void motor_exit_mode(uint8_t bus, uint8_t motor_id) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    can_send_std(bus, motor_id, d, 8);
}

// Set current position as zero (permanent, saved to motor flash)
static void motor_set_zero(uint8_t bus, uint8_t motor_id) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    can_send_std(bus, motor_id, d, 8);
}

// Send an MIT mini-cheetah command frame
// p_des  [rad]        desired position
// v_des  [rad/s]      desired velocity (feedforward)
// kp     [N·m/rad]    position gain
// kd     [N·m·s/rad]  damping gain
// t_ff   [N·m]        feedforward torque
static void motor_send_mit(uint8_t bus, uint8_t motor_id,
                            float p_des, float v_des,
                            float kp,   float kd,
                            float t_ff) {
    p_des = constrain_f(p_des, -MIT_P_MAX,  MIT_P_MAX);
    v_des = constrain_f(v_des, -MIT_V_MAX,  MIT_V_MAX);
    kp    = constrain_f(kp,    0.0f,        MIT_KP_MAX);
    kd    = constrain_f(kd,    0.0f,        MIT_KD_MAX);
    t_ff  = constrain_f(t_ff,  -MIT_T_MAX,  MIT_T_MAX);

    uint32_t p_int = float_to_uint(p_des, -MIT_P_MAX,  MIT_P_MAX,  16);
    uint32_t v_int = float_to_uint(v_des, -MIT_V_MAX,  MIT_V_MAX,  12);
    uint32_t kp_int= float_to_uint(kp,    0.0f,        MIT_KP_MAX, 12);
    uint32_t kd_int= float_to_uint(kd,    0.0f,        MIT_KD_MAX, 12);
    uint32_t t_int = float_to_uint(t_ff,  -MIT_T_MAX,  MIT_T_MAX,  12);

    uint8_t d[8];
    d[0] = (uint8_t)(p_int  >> 8);
    d[1] = (uint8_t)(p_int  & 0xFF);
    d[2] = (uint8_t)(v_int  >> 4);
    d[3] = (uint8_t)((v_int  & 0x0F) << 4) | (uint8_t)(kp_int >> 8);
    d[4] = (uint8_t)(kp_int & 0xFF);
    d[5] = (uint8_t)(kd_int >> 4);
    d[6] = (uint8_t)((kd_int & 0x0F) << 4) | (uint8_t)(t_int  >> 8);
    d[7] = (uint8_t)(t_int  & 0xFF);

    can_send_std(bus, motor_id, d, 8);
}

// Decode a MIT response frame into motor index feedback arrays.
// Returns true if the frame belonged to a known motor.
static bool parse_mit_response(const CAN_message_t &msg) {
    // Response is 8 bytes (manual §5.3):
    //   buf[0]      motor ID
    //   buf[1:2]    position  — 16-bit
    //   buf[3]      vel[11:4]
    //   buf[4]      vel[3:0] | cur[11:8]
    //   buf[5]      cur[7:0]
    //   buf[6]      temperature (raw - 40 = °C)
    //   buf[7]      error flags
    if (msg.len < 8) return false;

    uint8_t id = msg.buf[0];   // motor ID is in data[0]

    int idx = -1;
    for (int i = 0; i < 12; i++) {
        if (MOTOR_ID[i] == id) { idx = i; break; }
    }
    if (idx < 0) return false;

    uint32_t pos_raw = ((uint32_t)msg.buf[1] << 8)  |  msg.buf[2];
    uint32_t vel_raw = ((uint32_t)msg.buf[3] << 4)  | (msg.buf[4] >> 4);
    uint32_t cur_raw = (((uint32_t)msg.buf[4] & 0x0F) << 8) | msg.buf[5];

    fb_pos[idx]  = uint_to_float(pos_raw, -MIT_P_MAX, MIT_P_MAX, 16);
    fb_vel[idx]  = uint_to_float(vel_raw, -MIT_V_MAX, MIT_V_MAX, 12);
    fb_cur[idx]  = uint_to_float(cur_raw, -MIT_T_MAX, MIT_T_MAX, 12);
    fb_temp[idx] = (float)msg.buf[6] - 40.0f;   // raw - 40 = °C
    return true;
}

static void all_motors_enter_mode() {
    for (int i = 0; i < 12; i++) {
        motor_enter_mode(MOTOR_BUS[i], MOTOR_ID[i]);
        delay(2);
    }
}

static void all_motors_exit_mode() {
    for (int i = 0; i < 12; i++) {
        motor_exit_mode(MOTOR_BUS[i], MOTOR_ID[i]);
        delay(2);
    }
}

static void send_joint_cmds(const float *cmds) {
    for (int i = 0; i < 12; i++) {
        float pos = constrain_f(cmds[i], JOINT_MIN, JOINT_MAX);
        motor_send_mit(MOTOR_BUS[i], MOTOR_ID[i],
                       pos, 0.0f, g_kp, g_kd, 0.0f);
    }
}

// Poll both CAN buses and process any waiting response frames
static void poll_can_rx() {
    CAN_message_t msg;
    bool updated = false;
    while (can1.read(msg)) { if (parse_mit_response(msg)) updated = true; }
    while (can2.read(msg)) { if (parse_mit_response(msg)) updated = true; }

    if (updated && agent_state == AGENT_CONNECTED) {
        for (int i = 0; i < 12; i++) {
            joint_states_buf[i]      = fb_pos[i];
            joint_states_buf[i + 12] = fb_vel[i];
            joint_states_buf[i + 24] = fb_cur[i];
            joint_states_buf[i + 36] = fb_temp[i];
        }
        joint_states_msg.data.size = 48;
        rcl_publish(&joint_states_pub, &joint_states_msg, NULL);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BNO085 HELPERS
// ─────────────────────────────────────────────────────────────────────────────

// Drain the BNO085 SHTP event queue into the static state variables.
// Call frequently (every loop iteration) to avoid internal queue overflow.
static void bno_poll() {
    if (!bno_ok) return;
    sh2_SensorValue_t val;
    while (bno08x.getSensorEvent(&val)) {
        switch (val.sensorId) {
            case SH2_ROTATION_VECTOR:
                // Library returns (i, j, k, real) → map to (x, y, z, w)
                bno_qx = val.un.rotationVector.i;
                bno_qy = val.un.rotationVector.j;
                bno_qz = val.un.rotationVector.k;
                bno_qw = val.un.rotationVector.real;
                break;
            case SH2_ACCELEROMETER:
                bno_ax = val.un.accelerometer.x;   // m/s²
                bno_ay = val.un.accelerometer.y;
                bno_az = val.un.accelerometer.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                bno_gx = val.un.gyroscope.x;       // rad/s
                bno_gy = val.un.gyroscope.y;
                bno_gz = val.un.gyroscope.z;
                break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS CALLBACKS
// ─────────────────────────────────────────────────────────────────────────────

void angles_callback(const void *msg_in) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size != 12) return;
    for (int i = 0; i < 12; i++)
        joint_cmd[i] = msg->data.data[i];
    if (motors_enabled)
        send_joint_cmds(joint_cmd);
}

// Receives [kp, kd] from gait_node and updates the runtime gains.
// Called once per gait transition — low frequency, no performance concern.
void gains_callback(const void *msg_in) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size != 2) return;
    g_kp = constrain_f(msg->data.data[0], 0.0f, MIT_KP_MAX);
    g_kd = constrain_f(msg->data.data[1], 0.0f, MIT_KD_MAX);
}

void calibrate_callback(const void *msg_in) {
    const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msg_in;
    uint8_t target_id = msg->data;
    for (int i = 0; i < 12; i++) {
        if (MOTOR_ID[i] == target_id) {
            motor_set_zero(MOTOR_BUS[i], MOTOR_ID[i]);
            return;
        }
    }
}

void enable_callback(const void *msg_in) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
    if (msg->data) {
        all_motors_enter_mode();
        motors_enabled = true;
    } else {
        all_motors_exit_mode();
        motors_enabled = false;
    }
}

void imu_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/) {
    if (timer == NULL) return;

    // Drain BNO085 queue right before publish to get the freshest data
    bno_poll();

    int64_t stamp_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec     = (int32_t)(stamp_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

    // Orientation quaternion (from BNO085 rotation vector — fuses accel+gyro+mag)
    imu_msg.orientation.x = bno_qx;
    imu_msg.orientation.y = bno_qy;
    imu_msg.orientation.z = bno_qz;
    imu_msg.orientation.w = bno_qw;

    imu_msg.angular_velocity.x    = bno_gx;
    imu_msg.angular_velocity.y    = bno_gy;
    imu_msg.angular_velocity.z    = bno_gz;

    imu_msg.linear_acceleration.x = bno_ax;
    imu_msg.linear_acceleration.y = bno_ay;
    imu_msg.linear_acceleration.z = bno_az;

    rcl_publish(&imu_pub, &imu_msg, NULL);

    // Euler angles derived from quaternion (degrees)
    float sinr = 2.0f * (bno_qw * bno_qx + bno_qy * bno_qz);
    float cosr = 1.0f - 2.0f * (bno_qx * bno_qx + bno_qy * bno_qy);
    float sinp = 2.0f * (bno_qw * bno_qy - bno_qz * bno_qx);
    sinp = sinp >  1.0f ?  1.0f : (sinp < -1.0f ? -1.0f : sinp);
    float siny = 2.0f * (bno_qw * bno_qz + bno_qx * bno_qy);
    float cosy = 1.0f - 2.0f * (bno_qy * bno_qy + bno_qz * bno_qz);

    euler_msg.x = degrees(atan2f(sinr, cosr));   // roll
    euler_msg.y = degrees(asinf(sinp));           // pitch
    euler_msg.z = degrees(atan2f(siny, cosy));    // yaw (absolute, magnetometer-corrected)
    rcl_publish(&euler_pub, &euler_msg, NULL);
}

void gps_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/) {
    if (timer == NULL) return;

    int64_t stamp_ns = rmw_uros_epoch_nanos();
    fix_msg.header.stamp.sec     = (int32_t)(stamp_ns / 1000000000LL);
    fix_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

    if (gps_fix_valid) {
        fix_msg.latitude  = gps_lat;
        fix_msg.longitude = gps_lon;
        fix_msg.altitude  = gps_alt;
        fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
        // SAM-M10Q autonomous CEP ~2.0 m → 1-σ ≈ 1.7 m → variance ≈ 3 m²
        fix_msg.position_covariance[0] = 3.0;
        fix_msg.position_covariance[4] = 3.0;
        fix_msg.position_covariance[8] = 9.0;   // altitude ~3× worse
        fix_msg.position_covariance_type =
            sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
        fix_msg.status.status =
            sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
        fix_msg.position_covariance_type =
            sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
    }

    rcl_publish(&fix_pub, &fix_msg, NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS ENTITY LIFECYCLE
// ─────────────────────────────────────────────────────────────────────────────

bool create_entities() {
    allocator = rcl_get_default_allocator();

    // Use domain ID matching ROS_DOMAIN_ID on the Jetson so that the proxy
    // DDS entities created by the micro-ROS agent are visible to all nodes.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID_VAL);
    rcl_ret_t ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rcl_init_options_fini(&init_options);
    if (ret != RCL_RET_OK) return false;

    if (rclc_node_init_default(&node, "teensy_node", "", &support) != RCL_RET_OK) return false;

    // Subscriptions
    if (rclc_subscription_init_default(
            &angles_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "joint_angles") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &gains_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "joint_gains") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &calibrate_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            "can_calibrate") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &enable_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "can_enable") != RCL_RET_OK) return false;

    // Publishers
    if (rclc_publisher_init_default(
            &imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu/data") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &euler_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "imu/euler") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &fix_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
            "fix") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &joint_states_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "joint_states") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &homed_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "motors_homed") != RCL_RET_OK) return false;

    // IMU timer — 50 Hz
    if (rclc_timer_init_default(
            &imu_timer, &support,
            RCL_MS_TO_NS(1000 / IMU_RATE_HZ),
            imu_timer_callback) != RCL_RET_OK) return false;

    // GPS timer — 1 Hz
    if (rclc_timer_init_default(
            &gps_timer, &support,
            RCL_MS_TO_NS(1000 / GPS_RATE_HZ),
            gps_timer_callback) != RCL_RET_OK) return false;

    // Executor: 4 subscriptions + 2 timers
    if (rclc_executor_init(&executor, &support.context, 6, &allocator) != RCL_RET_OK)
        return false;

    rclc_executor_add_subscription(&executor, &angles_sub,    &angles_msg,
                                   &angles_callback,    ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &gains_sub,     &gains_msg,
                                   &gains_callback,     ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &calibrate_sub, &calibrate_msg,
                                   &calibrate_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &enable_sub,    &enable_msg,
                                   &enable_callback,    ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &imu_timer);
    rclc_executor_add_timer(&executor, &gps_timer);

    rmw_uros_sync_session(1000);
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    rcl_publisher_fini(&imu_pub,          &node);
    rcl_publisher_fini(&euler_pub,        &node);
    rcl_publisher_fini(&fix_pub,          &node);
    rcl_publisher_fini(&joint_states_pub, &node);
    rcl_publisher_fini(&homed_pub,        &node);
    rcl_subscription_fini(&angles_sub,    &node);
    rcl_subscription_fini(&gains_sub,     &node);
    rcl_subscription_fini(&calibrate_sub, &node);
    rcl_subscription_fini(&enable_sub,    &node);
    rcl_timer_fini(&imu_timer);
    rcl_timer_fini(&gps_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP / LOOP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // USB serial for micro-ROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // CAN1 — front legs (FR + FL)
    can1.begin();
    can1.setBaudRate(CAN_BAUD);
    can1.setMaxMB(16);
    can1.enableFIFO();

    // CAN2 — rear legs (RR + RL)
    can2.begin();
    can2.setBaudRate(CAN_BAUD);
    can2.setMaxMB(16);
    can2.enableFIFO();

    delay(100);   // let transceivers settle

    // BNO085 on Wire1 (SDA=17, SCL=16)
    Wire1.begin();
    Wire1.setClock(400000);
    bno_ok = bno08x.begin_I2C(BNO085_ADDR, &Wire1);
    if (bno_ok) {
        // Rotation vector fuses accel + gyro + magnetometer → absolute heading
        bno08x.enableReport(SH2_ROTATION_VECTOR,        IMU_INTERVAL_US);
        bno08x.enableReport(SH2_ACCELEROMETER,          IMU_INTERVAL_US);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,   IMU_INTERVAL_US);
    }

    // SAM-M10Q on Serial2 (RX=7, TX=8)
    Serial2.begin(GPS_BAUD);

    delay(100);

    // micro-ROS message buffers
    angles_msg.data.data     = angles_buf;
    angles_msg.data.size     = 0;
    angles_msg.data.capacity = 12;

    gains_msg.data.data     = gains_buf;
    gains_msg.data.size     = 0;
    gains_msg.data.capacity = 2;

    joint_states_msg.data.data     = joint_states_buf;
    joint_states_msg.data.size     = 48;
    joint_states_msg.data.capacity = 48;

    static char imu_frame[]  = "imu_link";
    imu_msg.header.frame_id.data     = imu_frame;
    imu_msg.header.frame_id.size     = strlen(imu_frame);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame);

    static char gps_frame[]  = "gps_link";
    fix_msg.header.frame_id.data     = gps_frame;
    fix_msg.header.frame_id.size     = strlen(gps_frame);
    fix_msg.header.frame_id.capacity = sizeof(gps_frame);

    fix_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    // IMU covariances — BNO085 specs
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]         = 0.0;
        imu_msg.angular_velocity_covariance[i]    = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    // Orientation: ~1° RMS = 0.017 rad → var ≈ 3e-4 rad²
    imu_msg.orientation_covariance[0] = 3e-4;
    imu_msg.orientation_covariance[4] = 3e-4;
    imu_msg.orientation_covariance[8] = 3e-4;
    // Gyro noise density integrated at 50 Hz
    imu_msg.angular_velocity_covariance[0] = 1e-4;
    imu_msg.angular_velocity_covariance[4] = 1e-4;
    imu_msg.angular_velocity_covariance[8] = 1e-4;
    // Accelerometer noise
    imu_msg.linear_acceleration_covariance[0] = 8e-3;
    imu_msg.linear_acceleration_covariance[4] = 8e-3;
    imu_msg.linear_acceleration_covariance[8] = 8e-3;
}

void loop() {
    static unsigned long last_blink_ms = 0;
    static unsigned long last_ping_ms  = 0;

    // Always feed GPS chars to TinyGPSPlus regardless of agent state
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) {
            // New sentence parsed — update GPS state
            if (gps.location.isValid()) {
                gps_lat       = gps.location.lat();
                gps_lon       = gps.location.lng();
                gps_alt       = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
                gps_fix_valid = true;
            } else {
                gps_fix_valid = false;
            }
        }
    }

    // Always poll BNO085 to keep its SHTP queue drained
    bno_poll();

    switch (agent_state) {

        case WAITING_AGENT:
            if (millis() - last_blink_ms > 500) {
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                last_blink_ms = millis();
            }
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (create_entities()) {
                    // Initialise homing state for all motors
                    for (int i = 0; i < 12; i++) {
                        home_phase[i]       = MHOME_SEEK;
                        home_stall_pos[i]   = 0.0f;
                        home_backoff_tgt[i] = 0.0f;
                        home_stall_count[i] = 0;
                        home_seek_start_ms[i]    = millis();
                        home_backoff_start_ms[i] = 0;
                        home_zero_sent[i]   = false;
                        home_zero_time_ms[i]= 0;
                    }
                    home_last_tick_ms = millis();
                    // Enter MIT mode so we can send position commands during homing
                    all_motors_enter_mode();
                    agent_state  = HOMING;
                    last_ping_ms = millis();
                    digitalWrite(LED_BUILTIN, HIGH);
                }
            }
            break;

        case HOMING: {
            poll_can_rx();
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

            // Run at 50 Hz (every 20 ms)
            if (millis() - home_last_tick_ms < 20) break;
            home_last_tick_ms = millis();

            bool all_done = true;

            for (int i = 0; i < 12; i++) {
                if (home_phase[i] == MHOME_DONE) continue;
                all_done = false;

                // Motors with HOME_DIR == 0 are skipped immediately
                if (HOME_DIR[i] == 0) {
                    home_phase[i] = MHOME_DONE;
                    continue;
                }

                switch (home_phase[i]) {

                    case MHOME_SEEK: {
                        // Drive gently toward the end-stop
                        float tgt = (float)HOME_DIR[i] * HOME_SEEK_TARGET;
                        motor_send_mit(MOTOR_BUS[i], MOTOR_ID[i],
                                       tgt, 0.0f, HOME_KP, HOME_KD, 0.0f);

                        bool stalled = (fabsf(fb_vel[i]) < HOME_STALL_VEL_THRESH &&
                                        fabsf(fb_cur[i]) > HOME_STALL_CUR_THRESH);
                        bool timedout = (millis() - home_seek_start_ms[i] > HOME_SEEK_TIMEOUT_MS);

                        if (stalled) home_stall_count[i]++;
                        else         home_stall_count[i] = 0;

                        if (home_stall_count[i] >= HOME_STALL_CONFIRM || timedout) {
                            home_stall_pos[i]   = fb_pos[i];
                            home_backoff_tgt[i] = fb_pos[i]
                                                  - (float)HOME_DIR[i] * HOME_BACKOFF_RAD[i];
                            home_backoff_start_ms[i] = millis();
                            home_phase[i] = MHOME_BACKOFF;
                        }
                        break;
                    }

                    case MHOME_BACKOFF: {
                        // Move to sitting position (stored-zero reference)
                        motor_send_mit(MOTOR_BUS[i], MOTOR_ID[i],
                                       home_backoff_tgt[i], 0.0f,
                                       DEFAULT_KP, DEFAULT_KD, 0.0f);

                        bool arrived  = fabsf(fb_pos[i] - home_backoff_tgt[i])
                                        < HOME_BACKOFF_THRESH_RAD;
                        bool timedout = (millis() - home_backoff_start_ms[i]
                                         > HOME_BACKOFF_TIMEOUT_MS);

                        if (arrived || timedout) {
                            home_phase[i] = MHOME_ZERO;
                        }
                        break;
                    }

                    case MHOME_ZERO: {
                        // Send set-zero once, then wait for flash write, then re-enter MIT
                        if (!home_zero_sent[i]) {
                            motor_set_zero(MOTOR_BUS[i], MOTOR_ID[i]);
                            home_zero_sent[i]  = true;
                            home_zero_time_ms[i] = millis();
                        } else if (millis() - home_zero_time_ms[i] > HOME_ZERO_SETTLE_MS) {
                            // set-zero exits MIT mode — re-enter
                            motor_enter_mode(MOTOR_BUS[i], MOTOR_ID[i]);
                            home_phase[i] = MHOME_DONE;
                        }
                        break;
                    }

                    default: break;
                }
            }

            if (all_done) {
                memset(joint_cmd, 0, sizeof(joint_cmd));
                motors_enabled = true;
                homed_msg.data = true;
                rcl_publish(&homed_pub, &homed_msg, NULL);
                agent_state = AGENT_CONNECTED;
            }

            // Keep pinging agent so disconnect is detected during homing
            if (millis() - last_ping_ms > 1000) {
                last_ping_ms = millis();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    all_motors_exit_mode();
                    motors_enabled = false;
                    destroy_entities();
                    agent_state = AGENT_DISCONNECTED;
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
            break;
        }

        case AGENT_CONNECTED:
            poll_can_rx();
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if (millis() - last_ping_ms > 500) {
                last_ping_ms = millis();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    if (motors_enabled) {
                        all_motors_exit_mode();
                        motors_enabled = false;
                    }
                    destroy_entities();
                    agent_state = AGENT_DISCONNECTED;
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
            break;

        case AGENT_DISCONNECTED:
            agent_state = WAITING_AGENT;
            break;
    }
}
