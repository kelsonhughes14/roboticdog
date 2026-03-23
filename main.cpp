#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ============================================================
//  CubeMars AK45-36 KV80 — MIT Control Mode, 3-DOF Leg Test
//  Teensy 4.1, CAN1, 1 Mbps
//
//  KEY DIFFERENCES FROM SERVO MODE:
//    - MIT mode uses STANDARD frames (flags.extended = 0)
//    - CAN ID = Motor CAN ID (just the bare ID, no packet type shift)
//    - 8-byte payload packs position, velocity, Kp, Kd, and feedforward
//      torque into fixed-point bitfields (not big-endian int32s)
//    - You MUST send the "enter MIT mode" magic packet before any
//      motion commands, or the motor will not respond.
//    - Motor feedback is also a standard frame with a specific layout.
//
//  RLINK PREPARATION (do this once per motor before running this code):
//    1. Connect via R-Link + CubeMars Tool software.
//    2. Open "Mode Switch" tab → click "Enter MIT Mode".
//       The motor LED will change colour to confirm.
//    3. The mode is saved to flash, so it persists across power cycles.
//    4. If you ever need servo mode again, click "Enter Servo Mode" and
//       reboot the motor.
//    NOTE: You do NOT need to re-calibrate if you already calibrated in
//    MIT mode previously. If this is the first time in MIT mode, follow
//    the MIT calibration procedure in the manual (4.2.2) first.
// ============================================================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// Motor CAN IDs — adjust to match your actual motor IDs set in R-Link
static constexpr uint8_t MOTOR_ID_HIP_ROLL   = 0x02;  // change as needed
static constexpr uint8_t MOTOR_ID_HIP_PITCH  = 0x7A;  // change as needed
static constexpr uint8_t MOTOR_ID_KNEE_PITCH = 0x03;  // change as needed

// ============================================================
//  AK45-36 KV80 MIT Parameter Limits
//
//  The AK45-36 is not explicitly listed in the manual's MIT table.
//  These limits are derived from the motor's published specs:
//    - 36:1 gear ratio, ~8 Nm rated / ~24 Nm peak torque output
//    - KV80 motor
//  Position range matches all AK-series motors in the manual.
//  Speed and torque are conservative — increase only after bench testing.
//  Kp/Kd ranges are fixed at 0-500 / 0-5 for all AK-series.
//
//  *** TUNE THESE VALUES ON THE BENCH BEFORE FULL OPERATION ***
// ============================================================
static constexpr float P_MIN   = -12.5f;  // rad
static constexpr float P_MAX   =  12.5f;  // rad  (~±716°)
static constexpr float V_MIN   = -10.0f;  // rad/s — conservative for 36:1
static constexpr float V_MAX   =  10.0f;  // rad/s
static constexpr float T_MIN   = -24.0f;  // N·m  (peak torque)
static constexpr float T_MAX   =  24.0f;  // N·m
static constexpr float KP_MIN  =   0.0f;
static constexpr float KP_MAX  = 500.0f;
static constexpr float KD_MIN  =   0.0f;
static constexpr float KD_MAX  =   5.0f;

// ============================================================
//  MIT special command payloads (standard frame, 8 bytes)
// ============================================================
static const uint8_t CMD_ENTER_MIT[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_EXIT_MIT[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_SET_ZERO[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

// ============================================================
//  Fixed-point conversion helpers
// ============================================================

// Convert float in [x_min, x_max] to unsigned int with 'bits' resolution
static inline int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  x = constrain(x, x_min, x_max);
  return (int)((x - x_min) * (float)((1 << bits) - 1) / span);
}

// Convert unsigned int back to float
static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  return (float)x_int * span / (float)((1 << bits) - 1) + x_min;
}

// ============================================================
//  Send a raw 8-byte standard CAN frame to a motor ID
// ============================================================
static void send_std_frame(uint8_t motor_id, const uint8_t data[8]) {
  CAN_message_t tx;
  tx.len          = 8;
  tx.flags.extended = 0;   // MIT mode uses STANDARD frames
  tx.flags.remote   = 0;
  tx.id           = motor_id;
  memcpy(tx.buf, data, 8);
  Can1.write(tx);
}

// ============================================================
//  MIT special commands
// ============================================================
static void send_enter_mit(uint8_t motor_id) {
  send_std_frame(motor_id, CMD_ENTER_MIT);
}

static void send_exit_mit(uint8_t motor_id) {
  send_std_frame(motor_id, CMD_EXIT_MIT);
}

// Set current position as zero (temporary, cleared on reboot)
static void send_set_zero_mit(uint8_t motor_id) {
  send_std_frame(motor_id, CMD_SET_ZERO);
}

// ============================================================
//  Core MIT motion command
//
//  p_des  : desired position (rad)
//  v_des  : desired velocity feedforward (rad/s), use 0 for position hold
//  kp     : position gain (N·m/rad), e.g. 50–200 for stiff position control
//  kd     : velocity damping gain (N·m·s/rad), e.g. 1–3
//  t_ff   : feedforward torque (N·m), use 0 for pure PD control
//
//  The motor output is: τ = kp*(p_des - p) + kd*(v_des - v) + t_ff
// ============================================================
static void send_mit_cmd(uint8_t motor_id,
                          float p_des, float v_des,
                          float kp,    float kd,
                          float t_ff) {
  // Clamp to limits
  p_des = constrain(p_des, P_MIN, P_MAX);
  v_des = constrain(v_des, V_MIN, V_MAX);
  kp    = constrain(kp,    KP_MIN, KP_MAX);
  kd    = constrain(kd,    KD_MIN, KD_MAX);
  t_ff  = constrain(t_ff,  T_MIN,  T_MAX);

  // Pack into fixed-point integers
  int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);   // 16-bit
  int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);   // 12-bit
  int kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12); // 12-bit
  int kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12); // 12-bit
  int t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12); // 12-bit

  // Pack bits into 8-byte payload (matches manual section 5.3)
  uint8_t buf[8];
  buf[0] = (uint8_t)(p_int  >> 8);                         // Position [15:8]
  buf[1] = (uint8_t)(p_int  & 0xFF);                       // Position [7:0]
  buf[2] = (uint8_t)(v_int  >> 4);                         // Speed    [11:4]
  buf[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8)); // Speed[3:0] | Kp[11:8]
  buf[4] = (uint8_t)(kp_int & 0xFF);                       // Kp       [7:0]
  buf[5] = (uint8_t)(kd_int >> 4);                         // Kd       [11:4]
  buf[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));// Kd[3:0] | Torque[11:8]
  buf[7] = (uint8_t)(t_int  & 0xFF);                       // Torque   [7:0]

  send_std_frame(motor_id, buf);
}

// Convenience: position-only hold (zero velocity, no feedforward)
static void send_mit_pos(uint8_t motor_id, float p_des_rad,
                          float kp = 80.0f, float kd = 1.5f) {
  send_mit_cmd(motor_id, p_des_rad, 0.0f, kp, kd, 0.0f);
}

// Convenience: zero-torque / compliant (motor goes limp)
static void send_mit_stop(uint8_t motor_id) {
  send_mit_cmd(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// ============================================================
//  Degree ↔ Radian helpers  (MIT protocol uses radians)
// ============================================================
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

// ============================================================
//  RX — parse MIT feedback frame
//
//  MIT feedback uses standard frame, ID = 0x00 | motor_id
//  Layout per manual section 5.3 "Send Data Definition":
//    DATA[0]       : Driver ID
//    DATA[1..2]    : Position (16-bit)
//    DATA[3][7:4]  : Speed    high 8 bits   → combined with DATA[4][7:4] = 12-bit
//    DATA[4][3:0]  : Current  high 4 bits   → combined with DATA[5]      = 12-bit
//    DATA[6]       : Temperature (raw; subtract 40 → °C)
//    DATA[7]       : Error flags
// ============================================================
struct MotorFeedback {
  bool    valid       = false;
  uint8_t id          = 0;
  float   pos_rad     = 0.0f;
  float   vel_rad_s   = 0.0f;
  float   torque_Nm   = 0.0f;
  int8_t  temp_C      = 0;
  uint8_t error_flags = 0;
  uint32_t count      = 0;
  uint32_t last_ms    = 0;
};

static MotorFeedback fb_roll, fb_hip, fb_knee;

static void parse_mit_rx(const CAN_message_t &rx) {
  if (rx.len < 8) return;

  uint8_t src_id = rx.buf[0];

  int p_int = (rx.buf[1] << 8) | rx.buf[2];
  int v_int = (rx.buf[3] << 4) | (rx.buf[4] >> 4);
  int i_int = ((rx.buf[4] & 0xF) << 8) | rx.buf[5];

  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, T_MIN, T_MAX, 12);
  int8_t temp   = (int8_t)rx.buf[6] - 40;  // per manual: raw - 40 = °C
  uint8_t error = rx.buf[7];

  auto fill = [&](MotorFeedback &fb) {
    fb.valid       = true;
    fb.id          = src_id;
    fb.pos_rad     = p;
    fb.vel_rad_s   = v;
    fb.torque_Nm   = t;
    fb.temp_C      = temp;
    fb.error_flags = error;
    fb.count++;
    fb.last_ms     = millis();
  };

  if (src_id == MOTOR_ID_HIP_ROLL)   fill(fb_roll);
  if (src_id == MOTOR_ID_HIP_PITCH)  fill(fb_hip);
  if (src_id == MOTOR_ID_KNEE_PITCH) fill(fb_knee);
}

// ============================================================
//  Debug flags
// ============================================================
static bool DEBUG_RAW_RX    = false;
static bool DEBUG_RX_STATUS = true;
static uint32_t last_status_print_ms = 0;

static void print_rx_raw(const CAN_message_t &rx) {
  Serial.print("RX ");
  Serial.print(rx.flags.extended ? "EXT " : "STD ");
  Serial.print("ID=0x"); Serial.print(rx.id, HEX);
  Serial.print(" DLC="); Serial.print(rx.len);
  Serial.print(" DATA=");
  for (int i = 0; i < rx.len; i++) {
    if (rx.buf[i] < 16) Serial.print("0");
    Serial.print(rx.buf[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

static void drain_rx() {
  CAN_message_t rx;
  while (Can1.read(rx)) {
    if (DEBUG_RAW_RX) print_rx_raw(rx);
    // MIT feedback = standard frame
    if (!rx.flags.extended) {
      parse_mit_rx(rx);
    }
  }
}

static void print_rx_status() {
  auto pr = [](const char *name, MotorFeedback &fb) {
    Serial.print(name); Serial.print(":");
    if (fb.count > 0) {
      Serial.print("OK("); Serial.print(fb.count); Serial.print(") ");
      Serial.print("p="); Serial.print(rad2deg(fb.pos_rad), 1); Serial.print("deg ");
      Serial.print("t="); Serial.print(fb.torque_Nm, 2); Serial.print("Nm ");
      Serial.print(fb.temp_C); Serial.print("C ");
      if (fb.error_flags) { Serial.print("ERR=0x"); Serial.print(fb.error_flags, HEX); Serial.print(" "); }
    } else {
      Serial.print("NO REPLY  ");
    }
    fb.count = 0;
  };
  Serial.print("[RX] ");
  pr("ROLL", fb_roll);
  pr("HIP",  fb_hip);
  pr("KNEE", fb_knee);
  Serial.println();
}

// ============================================================
//  State machine
// ============================================================
enum class Mode : uint8_t {
  MENU, WIGGLE_ROLL, WIGGLE_HIP, WIGGLE_KNEE,
  JOG_NEUTRAL, WALK_CYCLE, ESTOP
};

static Mode     mode    = Mode::MENU;
static uint32_t mode_t0 = 0;

// Current position targets (radians — MIT protocol uses radians)
static float cmd_roll = 0.0f;  // rad
static float cmd_hip  = 0.0f;  // rad
static float cmd_knee = 0.0f;  // rad

// Default PD gains — TUNE THESE ON THE BENCH
// Start with low Kp and verify motor moves correctly before increasing.
static float kp_pos = 50.0f;   // N·m/rad  — moderate stiffness for leg joints
static float kd_pos =  1.5f;   // N·m·s/rad — damping

// Wiggle
static float    wiggle_amp_deg = 5.0f;   // amplitude in degrees (converted to rad below)
static float    wiggle_hz      = 0.8f;
static uint32_t wiggle_ms      = 3000;

// Jog step
static float jog_step_deg = 5.0f;

// Walk parameters (all in radians at runtime)
static float roll_offset = 0.0f, hip_offset = 0.0f, knee_offset = 0.0f;
static float roll_amp_deg = 2.0f, hip_amp_deg = 15.0f, knee_amp_deg = 30.0f;
static float walk_hz = 0.5f;
static constexpr float KNEE_PHASE = (float)M_PI / 2.0f;

static void enter_mode(Mode m) { mode = m; mode_t0 = millis(); }

static void stop_all() {
  // Zero-torque (compliant) stop — does NOT brake hard
  send_mit_stop(MOTOR_ID_HIP_ROLL);
  send_mit_stop(MOTOR_ID_HIP_PITCH);
  send_mit_stop(MOTOR_ID_KNEE_PITCH);
}

static void estop_now() {
  stop_all();
  enter_mode(Mode::ESTOP);
  Serial.println("!!! E-STOP !!!");
}

static void show_menu() {
  Serial.println();
  Serial.println("=== 3-DOF Leg Test — MIT Control Mode ===");
  Serial.print  ("  ROLL=0x"); Serial.print(MOTOR_ID_HIP_ROLL, HEX);
  Serial.print  ("  HIP=0x");  Serial.print(MOTOR_ID_HIP_PITCH, HEX);
  Serial.print  ("  KNEE=0x"); Serial.println(MOTOR_ID_KNEE_PITCH, HEX);
  Serial.println();
  Serial.println("  FIRST: 'e' to send Enter-MIT-Mode to all motors");
  Serial.println("  'E'  -> Exit MIT mode (motor won't respond to commands)");
  Serial.println("  'z'  -> Set current position as zero (all motors)");
  Serial.println("  'j'  -> Jog mode");
  Serial.println("  '1'  -> Wiggle ROLL    '2'  -> Wiggle HIP    '3'  -> Wiggle KNEE");
  Serial.println("  'w'  -> Walk cycle     's'  -> Stop (compliant)");
  Serial.println("  'm'  -> Menu           'p'  -> Toggle raw RX  'o' -> Toggle status");
  Serial.println("  'x'  -> E-STOP");
  Serial.println();
  Serial.println("JOG: q/a=roll  e/d=hip  r/f=knee");
  Serial.println("     ]/[=Kp    }/{=Kd   b=back");
  Serial.println();
  Serial.print("Kp="); Serial.print(kp_pos);
  Serial.print("  Kd="); Serial.println(kd_pos);
  Serial.println();
}

// ============================================================
//  Serial handler
// ============================================================
static void handle_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 'x') { estop_now(); return; }
    if (c == 'p') { DEBUG_RAW_RX    = !DEBUG_RAW_RX;    Serial.println(DEBUG_RAW_RX    ? "RAW RX: ON" : "RAW RX: off"); continue; }
    if (c == 'o') { DEBUG_RX_STATUS = !DEBUG_RX_STATUS; Serial.println(DEBUG_RX_STATUS ? "STATUS: ON" : "STATUS: off"); continue; }

    if (mode == Mode::ESTOP) { if (c == 'm') { enter_mode(Mode::MENU); show_menu(); } continue; }

    switch (c) {
      case 'm': show_menu(); break;

      case 'e':  // Enter MIT mode on all motors (MUST do before sending any motion commands)
        send_enter_mit(MOTOR_ID_HIP_ROLL);
        delay(10);
        send_enter_mit(MOTOR_ID_HIP_PITCH);
        delay(10);
        send_enter_mit(MOTOR_ID_KNEE_PITCH);
        Serial.println("Sent Enter-MIT-Mode to all motors. Wait for LED confirmation.");
        break;

      case 'E':  // Exit MIT mode (motors go offline)
        send_exit_mit(MOTOR_ID_HIP_ROLL);
        delay(10);
        send_exit_mit(MOTOR_ID_HIP_PITCH);
        delay(10);
        send_exit_mit(MOTOR_ID_KNEE_PITCH);
        Serial.println("Sent Exit-MIT-Mode. Motors offline.");
        break;

      case 'z':  // Set zero at current position (temporary)
        send_set_zero_mit(MOTOR_ID_HIP_ROLL);
        delay(10);
        send_set_zero_mit(MOTOR_ID_HIP_PITCH);
        delay(10);
        send_set_zero_mit(MOTOR_ID_KNEE_PITCH);
        cmd_roll = cmd_hip = cmd_knee = 0.0f;
        Serial.println("Zero set (temporary). Re-enter MIT mode after zeroing if motor restarted.");
        break;

      case '1': enter_mode(Mode::WIGGLE_ROLL); Serial.println("Wiggling ROLL..."); break;
      case '2': enter_mode(Mode::WIGGLE_HIP);  Serial.println("Wiggling HIP...");  break;
      case '3': enter_mode(Mode::WIGGLE_KNEE); Serial.println("Wiggling KNEE..."); break;
      case 'w': enter_mode(Mode::WALK_CYCLE);  Serial.println("Walking. 's' to stop."); break;
      case 'j': enter_mode(Mode::JOG_NEUTRAL); Serial.println("JOG. q/a=roll e/d=hip r/f=knee ]/[=Kp }/{=Kd b=back"); break;

      case 's':
        stop_all();
        enter_mode(Mode::MENU);
        Serial.println("Stopped (compliant).");
        break;

      default:
        if (mode == Mode::JOG_NEUTRAL) {
          bool changed = true;
          float step = deg2rad(jog_step_deg);
          switch (c) {
            case 'q': cmd_roll  += step;  break;
            case 'a': cmd_roll  -= step;  break;
            case 'e': cmd_hip   += step;  break;
            case 'd': cmd_hip   -= step;  break;
            case 'r': cmd_knee  += step;  break;
            case 'f': cmd_knee  -= step;  break;
            case ']': kp_pos += 10.0f; if (kp_pos > KP_MAX) kp_pos = KP_MAX; break;
            case '[': kp_pos -= 10.0f; if (kp_pos < 0)      kp_pos = 0;      break;
            case '}': kd_pos += 0.25f; if (kd_pos > KD_MAX) kd_pos = KD_MAX; break;
            case '{': kd_pos -= 0.25f; if (kd_pos < 0)      kd_pos = 0;      break;
            case 'b': enter_mode(Mode::MENU); Serial.println("Menu."); changed = false; break;
            default:  changed = false; break;
          }
          if (changed) {
            Serial.print("roll="); Serial.print(rad2deg(cmd_roll),  1); Serial.print("deg ");
            Serial.print("hip=");  Serial.print(rad2deg(cmd_hip),   1); Serial.print("deg ");
            Serial.print("knee="); Serial.print(rad2deg(cmd_knee),  1); Serial.print("deg ");
            Serial.print("Kp=");   Serial.print(kp_pos);
            Serial.print(" Kd=");  Serial.println(kd_pos);
          }
        }
        break;
    }
  }
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  delay(100);

  Serial.println("CAN1 @ 1Mbps — MIT Control Mode");
  Serial.println();
  Serial.println("!!! IMPORTANT — READ BEFORE OPERATING !!!");
  Serial.println("1. Motors must be put into MIT mode via R-Link BEFORE running.");
  Serial.println("   (Mode Switch tab -> Enter MIT Mode, saved to flash)");
  Serial.println("2. Press 'e' to send the Enter-MIT-Mode CAN command to all motors.");
  Serial.println("   Without this, motion commands are silently ignored.");
  Serial.println("3. MIT mode uses STANDARD (11-bit) CAN frames, not extended.");
  Serial.println("4. Positions are in RADIANS. 1 rad ≈ 57.3 degrees.");
  Serial.println("5. Start with low Kp (e.g. 10-20) on the bench before full gains.");
  Serial.println();

  show_menu();
}

void loop() {
  drain_rx();
  handle_serial();

  // RX status every 2 seconds
  if (DEBUG_RX_STATUS) {
    uint32_t now = millis();
    if (now - last_status_print_ms >= 2000) {
      print_rx_status();
      last_status_print_ms = now;
    }
  }

  // 100 Hz command loop
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < 10) return;
  last_ms = now;

  const float t_s  = (float)(now - mode_t0) * 0.001f;
  const uint32_t t_ms = now - mode_t0;

  switch (mode) {

    case Mode::MENU:
      // No commands sent — motors hold position via their own internal state
      // while in MIT mode.  If you want active hold, uncomment:
      // send_mit_pos(MOTOR_ID_HIP_ROLL,  cmd_roll, kp_pos, kd_pos);
      // send_mit_pos(MOTOR_ID_HIP_PITCH, cmd_hip,  kp_pos, kd_pos);
      // send_mit_pos(MOTOR_ID_KNEE_PITCH,cmd_knee, kp_pos, kd_pos);
      break;

    case Mode::WIGGLE_ROLL: {
      float off = deg2rad(wiggle_amp_deg) * sinf(2.0f * (float)M_PI * wiggle_hz * t_s);
      send_mit_pos(MOTOR_ID_HIP_ROLL,   cmd_roll + off, kp_pos, kd_pos);
      send_mit_stop(MOTOR_ID_HIP_PITCH);
      send_mit_stop(MOTOR_ID_KNEE_PITCH);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle ROLL done."); }
    } break;

    case Mode::WIGGLE_HIP: {
      float off = deg2rad(wiggle_amp_deg) * sinf(2.0f * (float)M_PI * wiggle_hz * t_s);
      send_mit_stop(MOTOR_ID_HIP_ROLL);
      send_mit_pos(MOTOR_ID_HIP_PITCH,  cmd_hip  + off, kp_pos, kd_pos);
      send_mit_stop(MOTOR_ID_KNEE_PITCH);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle HIP done."); }
    } break;

    case Mode::WIGGLE_KNEE: {
      float off = deg2rad(wiggle_amp_deg) * sinf(2.0f * (float)M_PI * wiggle_hz * t_s);
      send_mit_stop(MOTOR_ID_HIP_ROLL);
      send_mit_stop(MOTOR_ID_HIP_PITCH);
      send_mit_pos(MOTOR_ID_KNEE_PITCH, cmd_knee + off, kp_pos, kd_pos);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle KNEE done."); }
    } break;

    case Mode::JOG_NEUTRAL:
      send_mit_pos(MOTOR_ID_HIP_ROLL,   cmd_roll, kp_pos, kd_pos);
      send_mit_pos(MOTOR_ID_HIP_PITCH,  cmd_hip,  kp_pos, kd_pos);
      send_mit_pos(MOTOR_ID_KNEE_PITCH, cmd_knee, kp_pos, kd_pos);
      break;

    case Mode::WALK_CYCLE: {
      float w = 2.0f * (float)M_PI * walk_hz;
      cmd_roll  = roll_offset  + deg2rad(roll_amp_deg)  * sinf(w * t_s);
      cmd_hip   = hip_offset   + deg2rad(hip_amp_deg)   * sinf(w * t_s);
      cmd_knee  = knee_offset  + deg2rad(knee_amp_deg)  * sinf(w * t_s + KNEE_PHASE);
      send_mit_pos(MOTOR_ID_HIP_ROLL,   cmd_roll, kp_pos, kd_pos);
      send_mit_pos(MOTOR_ID_HIP_PITCH,  cmd_hip,  kp_pos, kd_pos);
      send_mit_pos(MOTOR_ID_KNEE_PITCH, cmd_knee, kp_pos, kd_pos);
    } break;

    case Mode::ESTOP:
    default:
      break;
  }
}
