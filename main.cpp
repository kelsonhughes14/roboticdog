#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ============================================================
//  CubeMars AK45-36 KV80 — MIT Control Mode, 8-DOF Quad Leg Test
//  Teensy 4.1, CAN1 (front legs) + CAN3 (back legs), 1 Mbps
//
//  Layout:
//    Front Left  (FL): Hip=0x75, Knee=0x78  — CAN1
//    Front Right (FR): Hip=0x79, Knee=0x7A  — CAN1
//    Back  Left  (BL): Hip=0x76, Knee=0x77  — CAN3
//    Back  Right (BR): Hip=0x73, Knee=0x74  — CAN3
//
//  STATIC STAND TEST:
//    1. Press 'e' — send Enter-MIT-Mode to all motors on both buses.
//    2. Press 'z' — set current position as zero on all motors.
//    3. Press 'j' — enter Jog mode. All motors will actively hold
//       their current (zero) position. Do NOT press movement keys;
//       just let the PD controller lock the joints. Monitor the
//       current readout (printed every 2 s) to watch load.
//    4. Press 's' or 'x' to stop / e-stop when done.
//
//  CURRENT ESTIMATION:
//    The AK45-36 does not report phase current directly in MIT
//    feedback. Current is estimated from reported torque using the
//    motor's torque constant (Kt). For AK45-36 KV80:
//      KV  = 80 RPM/V  →  Ke = 1/(KV * 2π/60) ≈ 0.1194 V·s/rad
//      Kt  ≈ Ke = 0.1194 N·m/A   (SI units, line-to-line back-EMF)
//      Gear ratio = 36:1
//      Output Kt = Kt_motor * gear_ratio ≈ 4.30 N·m/A (at output shaft)
//    So: I_estimated (A) = torque_Nm / KT_OUTPUT
//    This is a rough estimate — actual phase current is higher due
//    to efficiency losses (~85–90%) and electrical harmonics.
//    For a more accurate reading, add an inline current sensor on
//    the motor power rail.
//
//  RLINK PREPARATION (once per motor):
//    Connect via R-Link → Mode Switch → Enter MIT Mode.
//    LED changes colour. Mode is saved to flash.
// ============================================================

// ============================================================
//  CAN buses
// ============================================================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // Front legs
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;  // Back legs

// ============================================================
//  Motor CAN IDs
//  Format: MOTOR_ID_<LEG>_<JOINT>
//    FL = Front Left,  FR = Front Right
//    BL = Back  Left,  BR = Back  Right
// ============================================================
static constexpr uint8_t MOTOR_ID_FL_HIP  = 0x75;  // CAN1
static constexpr uint8_t MOTOR_ID_FL_KNEE = 0x78;  // CAN1
static constexpr uint8_t MOTOR_ID_FR_HIP  = 0x79;  // CAN1
static constexpr uint8_t MOTOR_ID_FR_KNEE = 0x7A;  // CAN1

static constexpr uint8_t MOTOR_ID_BL_HIP  = 0x76;  // CAN3
static constexpr uint8_t MOTOR_ID_BL_KNEE = 0x77;  // CAN3
static constexpr uint8_t MOTOR_ID_BR_HIP  = 0x73;  // CAN3
static constexpr uint8_t MOTOR_ID_BR_KNEE = 0x74;  // CAN3

// ============================================================
//  Torque constant for current estimation
//  AK45-36 KV80: Kt_motor ≈ 0.1194 N·m/A, gear ratio = 36
//  Output shaft: KT_OUTPUT ≈ 4.30 N·m/A
//  Adjust if you have a measured/calibrated value.
// ============================================================
static constexpr float KT_OUTPUT = 4.30f;  // N·m/A at output shaft

// ============================================================
//  MIT Parameter Limits  (same as 2-DOF version)
// ============================================================
static constexpr float P_MIN  = -12.5f;
static constexpr float P_MAX  =  12.5f;
static constexpr float V_MIN  = -10.0f;
static constexpr float V_MAX  =  10.0f;
static constexpr float T_MIN  = -7.0f;
static constexpr float T_MAX  =  7.0f;
static constexpr float KP_MIN =   0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN =   0.0f;
static constexpr float KD_MAX =   5.0f;

// ============================================================
//  MIT special command payloads
// ============================================================
static const uint8_t CMD_ENTER_MIT[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_EXIT_MIT[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_SET_ZERO[8]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

// ============================================================
//  Fixed-point conversion helpers
// ============================================================
static inline int float_to_uint(float x, float x_min, float x_max, int bits) {
  x = constrain(x, x_min, x_max);
  return (int)((x - x_min) * (float)((1 << bits) - 1) / (x_max - x_min));
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  return (float)x_int * (x_max - x_min) / (float)((1 << bits) - 1) + x_min;
}

// ============================================================
//  CAN bus tag — lets one send helper cover both buses
// ============================================================
enum class Bus : uint8_t { CAN1_FRONT, CAN3_BACK };

static void send_std_frame(Bus bus, uint8_t motor_id, const uint8_t data[8]) {
  CAN_message_t tx;
  tx.len            = 8;
  tx.flags.extended = 0;
  tx.flags.remote   = 0;
  tx.id             = motor_id;
  memcpy(tx.buf, data, 8);
  if (bus == Bus::CAN1_FRONT) Can1.write(tx);
  else                        Can3.write(tx);
}

// ============================================================
//  Per-motor descriptor — associates ID with its CAN bus
// ============================================================
struct MotorDesc {
  uint8_t id;
  Bus     bus;
  const char *name;
};

// Table of all 8 motors — order defines print order
static const MotorDesc MOTORS[8] = {
  { MOTOR_ID_FL_HIP,  Bus::CAN1_FRONT, "FL_HIP"  },
  { MOTOR_ID_FL_KNEE, Bus::CAN1_FRONT, "FL_KNEE" },
  { MOTOR_ID_FR_HIP,  Bus::CAN1_FRONT, "FR_HIP"  },
  { MOTOR_ID_FR_KNEE, Bus::CAN1_FRONT, "FR_KNEE" },
  { MOTOR_ID_BL_HIP,  Bus::CAN3_BACK,  "BL_HIP"  },
  { MOTOR_ID_BL_KNEE, Bus::CAN3_BACK,  "BL_KNEE" },
  { MOTOR_ID_BR_HIP,  Bus::CAN3_BACK,  "BR_HIP"  },
  { MOTOR_ID_BR_KNEE, Bus::CAN3_BACK,  "BR_KNEE" },
};
static constexpr int N_MOTORS = 8;

// ============================================================
//  MIT special commands — generic helpers
// ============================================================
static void send_enter_mit(Bus bus, uint8_t id)   { send_std_frame(bus, id, CMD_ENTER_MIT); }
static void send_exit_mit(Bus bus, uint8_t id)    { send_std_frame(bus, id, CMD_EXIT_MIT);  }
static void send_set_zero_mit(Bus bus, uint8_t id){ send_std_frame(bus, id, CMD_SET_ZERO);  }

static void send_all(const uint8_t cmd[8]) {
  for (int i = 0; i < N_MOTORS; i++) {
    send_std_frame(MOTORS[i].bus, MOTORS[i].id, cmd);
    delay(5);
  }
}

// ============================================================
//  Core MIT motion command
// ============================================================
static void send_mit_cmd(Bus bus, uint8_t motor_id,
                          float p_des, float v_des,
                          float kp,    float kd,
                          float t_ff) {
  p_des = constrain(p_des, P_MIN, P_MAX);
  v_des = constrain(v_des, V_MIN, V_MAX);
  kp    = constrain(kp,    KP_MIN, KP_MAX);
  kd    = constrain(kd,    KD_MIN, KD_MAX);
  t_ff  = constrain(t_ff,  T_MIN,  T_MAX);

  int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
  int t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

  uint8_t buf[8];
  buf[0] = (uint8_t)(p_int  >> 8);
  buf[1] = (uint8_t)(p_int  & 0xFF);
  buf[2] = (uint8_t)(v_int  >> 4);
  buf[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8));
  buf[4] = (uint8_t)(kp_int & 0xFF);
  buf[5] = (uint8_t)(kd_int >> 4);
  buf[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));
  buf[7] = (uint8_t)(t_int  & 0xFF);

  send_std_frame(bus, motor_id, buf);
}

static void send_mit_pos(Bus bus, uint8_t id, float p_rad,
                          float kp = 80.0f, float kd = 1.5f) {
  send_mit_cmd(bus, id, p_rad, 0.0f, kp, kd, 0.0f);
}

static void send_mit_stop(Bus bus, uint8_t id) {
  send_mit_cmd(bus, id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// ============================================================
//  Degree / Radian helpers
// ============================================================
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

// ============================================================
//  Motor feedback — one struct per motor, indexed to MOTORS[]
// ============================================================
struct MotorFeedback {
  bool     valid       = false;
  uint8_t  id          = 0;
  float    pos_rad     = 0.0f;
  float    vel_rad_s   = 0.0f;
  float    torque_Nm   = 0.0f;
  float    current_A   = 0.0f;   // estimated: torque_Nm / KT_OUTPUT
  int8_t   temp_C      = 0;
  uint8_t  error_flags = 0;
  uint32_t count       = 0;
  uint32_t last_ms     = 0;
};

static MotorFeedback fb[N_MOTORS];

// Map a source ID + bus to a feedback slot
static int find_motor_index(uint8_t src_id, Bus bus) {
  for (int i = 0; i < N_MOTORS; i++) {
    if (MOTORS[i].id == src_id && MOTORS[i].bus == bus) return i;
  }
  return -1;
}

static void parse_mit_rx(const CAN_message_t &rx, Bus bus) {
  if (rx.len < 8) return;

  // Match on the CAN frame ID (rx.id), not rx.buf[0].
  // buf[0] is the motor's self-reported driver ID which can differ from
  // its actual CAN ID if the motor's internal ID register is misconfigured.
  // rx.id is always authoritative — it is what the motor transmits on the bus.
  int idx = find_motor_index((uint8_t)rx.id, bus);
  if (idx < 0) return;

  int p_int = (rx.buf[1] << 8) | rx.buf[2];
  int v_int = (rx.buf[3] << 4) | (rx.buf[4] >> 4);
  int i_int = ((rx.buf[4] & 0xF) << 8) | rx.buf[5];

  fb[idx].valid       = true;
  fb[idx].id          = (uint8_t)rx.id;  // use frame ID, matches MOTORS[] table
  fb[idx].pos_rad     = uint_to_float(p_int, P_MIN, P_MAX, 16);
  fb[idx].vel_rad_s   = uint_to_float(v_int, V_MIN, V_MAX, 12);
  fb[idx].torque_Nm   = uint_to_float(i_int, T_MIN, T_MAX, 12);
  fb[idx].current_A   = fabsf(fb[idx].torque_Nm) / KT_OUTPUT;  // magnitude
  fb[idx].temp_C      = (int8_t)rx.buf[6] - 40;
  fb[idx].error_flags = rx.buf[7];
  fb[idx].count++;
  fb[idx].last_ms     = millis();
}

// ============================================================
//  Debug flags
// ============================================================
static bool     DEBUG_RAW_RX         = false;
static bool     DEBUG_RX_STATUS      = true;
static uint32_t last_status_print_ms = 0;

static void print_rx_raw(const CAN_message_t &rx, Bus bus) {
  Serial.print("RX ");
  Serial.print(bus == Bus::CAN1_FRONT ? "CAN1 " : "CAN3 ");
  Serial.print(rx.flags.extended ? "EXT " : "STD ");
  Serial.print("ID=0x"); Serial.print(rx.id, HEX);
  // Flag any mismatch between frame ID and the motor's self-reported ID in buf[0]
  if (rx.len >= 1 && rx.buf[0] != (uint8_t)rx.id) {
    Serial.print(" [ID MISMATCH buf[0]=0x"); Serial.print(rx.buf[0], HEX); Serial.print("]");
  }
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
    if (DEBUG_RAW_RX) print_rx_raw(rx, Bus::CAN1_FRONT);
    if (!rx.flags.extended) parse_mit_rx(rx, Bus::CAN1_FRONT);
  }
  while (Can3.read(rx)) {
    if (DEBUG_RAW_RX) print_rx_raw(rx, Bus::CAN3_BACK);
    if (!rx.flags.extended) parse_mit_rx(rx, Bus::CAN3_BACK);
  }
}

static void print_rx_status() {
  float total_current = 0.0f;

  Serial.println("---------------------------------------------------");
  Serial.println("  Motor       | Replies | Pos(deg) | Torq(Nm) | I_est(A) | Temp | Err");

  for (int i = 0; i < N_MOTORS; i++) {
    // Print separator between legs for readability
    if (i == 2 || i == 4 || i == 6) Serial.println("  ------------|---------|----------|----------|----------|------|----");

    Serial.print("  ");
    // Pad name to 11 chars
    int nlen = strlen(MOTORS[i].name);
    Serial.print(MOTORS[i].name);
    for (int s = nlen; s < 11; s++) Serial.print(' ');
    Serial.print(" | ");

    if (fb[i].count > 0) {
      // Replies (3 digits)
      if (fb[i].count < 100) Serial.print(' ');
      if (fb[i].count < 10)  Serial.print(' ');
      Serial.print(fb[i].count);
      Serial.print("     | ");

      float pos_d = rad2deg(fb[i].pos_rad);
      if (pos_d >= 0 && pos_d < 100) Serial.print(' ');
      if (fabsf(pos_d) < 100)        Serial.print(' ');
      Serial.print(pos_d, 1);
      Serial.print("    | ");

      float trq = fb[i].torque_Nm;
      if (trq >= 0 && trq < 10) Serial.print(' ');
      if (fabsf(trq) < 10)      Serial.print(' ');
      Serial.print(trq, 2);
      Serial.print("    | ");

      Serial.print(fb[i].current_A, 3);
      Serial.print("    | ");

      Serial.print(fb[i].temp_C);
      Serial.print("°C  | ");

      if (fb[i].error_flags) {
        Serial.print("0x"); Serial.print(fb[i].error_flags, HEX);
      } else {
        Serial.print("OK");
      }
      Serial.println();

      total_current += fb[i].current_A;
      fb[i].count = 0;
    } else {
      Serial.println("NO REPLY |          |          |          |      |");
    }
  }

  Serial.println("---------------------------------------------------");
  Serial.print  ("  Total estimated current: ");
  Serial.print  (total_current, 3);
  Serial.println(" A");
  Serial.println("  (I_est = |torque| / KT_OUTPUT; KT_OUTPUT = 4.30 N·m/A)");
  Serial.println("---------------------------------------------------");
  Serial.println();
}

// ============================================================
//  Per-leg position targets and jog state
// ============================================================
// Indexed: 0=FL_HIP, 1=FL_KNEE, 2=FR_HIP, 3=FR_KNEE,
//          4=BL_HIP, 5=BL_KNEE, 6=BR_HIP, 7=BR_KNEE
static float cmd[N_MOTORS] = {0};  // rad

static float kp_pos = 50.0f;
static float kd_pos =  1.5f;

// Jog: which leg is selected (0=FL, 1=FR, 2=BL, 3=BR, 4=ALL)
static int   jog_leg      = 4;   // default: ALL
static float jog_step_deg = 5.0f;

// ============================================================
//  Walk parameters (identical physics to 2-DOF, applied per leg)
//  Front and back legs are 180° out of phase for a trot-like gait.
// ============================================================
static float hip_offset    = 0.0f,  knee_offset   = 0.0f;
static float hip_amp_deg   = 15.0f, knee_amp_deg  = 30.0f;
static float walk_hz       = 0.5f;
static constexpr float KNEE_PHASE = (float)M_PI / 2.0f;
static constexpr float BACK_PHASE = (float)M_PI;       // back legs antiphase

// Wiggle
static float    wiggle_amp_deg = 5.0f;
static float    wiggle_hz      = 0.8f;
static uint32_t wiggle_ms      = 3000;

// ============================================================
//  State machine
// ============================================================
enum class Mode : uint8_t {
  MENU,
  WIGGLE_HIP, WIGGLE_KNEE,
  JOG_NEUTRAL,
  WALK_CYCLE,
  ESTOP
};

static Mode     mode    = Mode::MENU;
static uint32_t mode_t0 = 0;
static void enter_mode(Mode m) { mode = m; mode_t0 = millis(); }

// ============================================================
//  Helpers: send commands to a group of motors
// ============================================================

// Send MIT position command to all 8 motors using their current cmd[] targets
static void send_all_pos() {
  for (int i = 0; i < N_MOTORS; i++) {
    send_mit_pos(MOTORS[i].bus, MOTORS[i].id, cmd[i], kp_pos, kd_pos);
  }
}

// Send compliant stop to all 8 motors
static void stop_all() {
  for (int i = 0; i < N_MOTORS; i++) {
    send_mit_stop(MOTORS[i].bus, MOTORS[i].id);
  }
}

static void estop_now() {
  stop_all();
  enter_mode(Mode::ESTOP);
  Serial.println("!!! E-STOP !!!");
}

// ============================================================
//  Menu
// ============================================================
static void show_menu() {
  Serial.println();
  Serial.println("====== 8-DOF Quad Leg Test — MIT Control Mode ======");
  Serial.println("  CAN1 (front): FL_HIP=0x75  FL_KNEE=0x78");
  Serial.println("                FR_HIP=0x79  FR_KNEE=0x7A");
  Serial.println("  CAN3 (back):  BL_HIP=0x76  BL_KNEE=0x77");
  Serial.println("                BR_HIP=0x73  BR_KNEE=0x74");
  Serial.println();
  Serial.println("  FIRST: 'e' -> Enter-MIT-Mode (all 8 motors)");
  Serial.println("  'E'   -> Exit MIT mode");
  Serial.println("  'z'   -> Set zero (all motors, current pos)");
  Serial.println();
  Serial.println("  '2'   -> Wiggle HIP  (all legs)");
  Serial.println("  '3'   -> Wiggle KNEE (all legs)");
  Serial.println("  'w'   -> Walk cycle (trot gait, front/back antiphase)");
  Serial.println("  'j'   -> Jog / Static Stand mode");
  Serial.println("  's'   -> Stop (compliant, all motors)");
  Serial.println("  'x'   -> E-STOP");
  Serial.println("  'm'   -> Show this menu");
  Serial.println("  'p'   -> Toggle raw RX print");
  Serial.println("  'o'   -> Toggle status print");
  Serial.println();
  Serial.println("  JOG LEG SELECT: 1=FL  2=FR  3=BL  4=BR  0=ALL");
  Serial.println("  JOG KEYS:  u/n = hip+/-    i/k = knee+/-");
  Serial.println("             ]/[  = Kp+/-    }/{ = Kd+/-");
  Serial.println("             b    = back to menu");
  Serial.println();
  Serial.println("  STATIC STAND: Press 'j' after zeroing. Motors will");
  Serial.println("  hold zero position. Watch current readout (every 2s).");
  Serial.println();
  Serial.print  ("  Kp="); Serial.print(kp_pos);
  Serial.print  ("  Kd="); Serial.println(kd_pos);
  Serial.println("=====================================================");
}

// ============================================================
//  Serial command handler
// ============================================================
static void handle_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // Global overrides (work in any mode)
    if (c == 'x') { estop_now(); return; }
    if (c == 'p') {
      DEBUG_RAW_RX = !DEBUG_RAW_RX;
      Serial.println(DEBUG_RAW_RX ? "RAW RX: ON" : "RAW RX: off");
      continue;
    }
    if (c == 'o') {
      DEBUG_RX_STATUS = !DEBUG_RX_STATUS;
      Serial.println(DEBUG_RX_STATUS ? "STATUS: ON" : "STATUS: off");
      continue;
    }

    // ESTOP: only 'm' to recover
    if (mode == Mode::ESTOP) {
      if (c == 'm') { enter_mode(Mode::MENU); show_menu(); }
      continue;
    }

    switch (c) {

      // ── Menu ──────────────────────────────────────────────
      case 'm':
        show_menu();
        break;

      // ── Enter MIT mode ────────────────────────────────────
      case 'e':
        Serial.println("Sending Enter-MIT-Mode to all 8 motors...");
        send_all(CMD_ENTER_MIT);
        Serial.println("Done. Wait for LED confirmation on each motor.");
        break;

      // ── Exit MIT mode ─────────────────────────────────────
      case 'E':
        send_all(CMD_EXIT_MIT);
        Serial.println("Sent Exit-MIT-Mode. Motors offline.");
        break;

      // ── Set zero ──────────────────────────────────────────
      case 'z':
        send_all(CMD_SET_ZERO);
        for (int i = 0; i < N_MOTORS; i++) cmd[i] = 0.0f;
        Serial.println("Zero set (temporary). cmd[] cleared to 0.");
        break;

      // ── Wiggle modes ──────────────────────────────────────
      case '2':
        enter_mode(Mode::WIGGLE_HIP);
        Serial.println("Wiggling HIP on all legs...");
        break;
      case '3':
        enter_mode(Mode::WIGGLE_KNEE);
        Serial.println("Wiggling KNEE on all legs...");
        break;

      // ── Walk ──────────────────────────────────────────────
      case 'w':
        enter_mode(Mode::WALK_CYCLE);
        Serial.println("Walk cycle. Front/back antiphase. 's' to stop.");
        break;

      // ── Jog / Static stand ────────────────────────────────
      case 'j':
        enter_mode(Mode::JOG_NEUTRAL);
        Serial.print("JOG mode. Leg select: 0=ALL(current=");
        Serial.print(jog_leg == 4 ? "ALL" :
                     jog_leg == 0 ? "FL"  :
                     jog_leg == 1 ? "FR"  :
                     jog_leg == 2 ? "BL"  : "BR");
        Serial.println("). u/n=hip i/k=knee ]/[=Kp }/{=Kd b=back");
        break;

      // ── Compliant stop ────────────────────────────────────
      case 's':
        stop_all();
        enter_mode(Mode::MENU);
        Serial.println("Stopped (compliant).");
        break;

      // ── Jog sub-commands ──────────────────────────────────
      default:
        if (mode == Mode::JOG_NEUTRAL) {
          bool changed = true;
          float step = deg2rad(jog_step_deg);

          // Leg selection: '0'=ALL, '1'=FL, '2'=FR, '3'=BL, '4'=BR
          if (c >= '0' && c <= '4') {
            // '0' maps to jog_leg=4 (ALL), '1'→0, '2'→1, '3'→2, '4'→3
            jog_leg = (c == '0') ? 4 : (c - '1');
            const char *names[] = {"FL","FR","BL","BR","ALL"};
            Serial.print("Jog leg: "); Serial.println(names[jog_leg]);
            changed = false;
            break;
          }

          // Determine which motor indices to jog
          // Motor index layout: FL=0,1  FR=2,3  BL=4,5  BR=6,7
          int hip_idx  = -1, knee_idx = -1;
          if (jog_leg == 4) {
            // ALL legs — move all hips or all knees together
            switch (c) {
              case 'u': for (int i=0;i<N_MOTORS;i+=2) cmd[i]   += step; break;
              case 'n': for (int i=0;i<N_MOTORS;i+=2) cmd[i]   -= step; break;
              case 'i': for (int i=1;i<N_MOTORS;i+=2) cmd[i]   += step; break;
              case 'k': for (int i=1;i<N_MOTORS;i+=2) cmd[i]   -= step; break;
              case ']': kp_pos += 10.0f; if (kp_pos > KP_MAX) kp_pos = KP_MAX; break;
              case '[': kp_pos -= 10.0f; if (kp_pos < 0)      kp_pos = 0;      break;
              case '}': kd_pos += 0.25f; if (kd_pos > KD_MAX) kd_pos = KD_MAX; break;
              case '{': kd_pos -= 0.25f; if (kd_pos < 0)      kd_pos = 0;      break;
              case 'b': enter_mode(Mode::MENU); Serial.println("Menu."); changed = false; break;
              default:  changed = false; break;
            }
          } else {
            // Single leg: base index = jog_leg * 2
            int base = jog_leg * 2;
            switch (c) {
              case 'u': cmd[base]   += step; break;
              case 'n': cmd[base]   -= step; break;
              case 'i': cmd[base+1] += step; break;
              case 'k': cmd[base+1] -= step; break;
              case ']': kp_pos += 10.0f; if (kp_pos > KP_MAX) kp_pos = KP_MAX; break;
              case '[': kp_pos -= 10.0f; if (kp_pos < 0)      kp_pos = 0;      break;
              case '}': kd_pos += 0.25f; if (kd_pos > KD_MAX) kd_pos = KD_MAX; break;
              case '{': kd_pos -= 0.25f; if (kd_pos < 0)      kd_pos = 0;      break;
              case 'b': enter_mode(Mode::MENU); Serial.println("Menu."); changed = false; break;
              default:  changed = false; break;
            }
          }

          if (changed) {
            // Print all leg positions
            const char *leg_names[] = {"FL","FR","BL","BR"};
            for (int l = 0; l < 4; l++) {
              int b = l * 2;
              Serial.print(leg_names[l]);
              Serial.print(" hip=");  Serial.print(rad2deg(cmd[b]),   1); Serial.print("deg ");
              Serial.print("knee="); Serial.print(rad2deg(cmd[b+1]), 1); Serial.print("deg  ");
            }
            Serial.print("Kp="); Serial.print(kp_pos);
            Serial.print(" Kd="); Serial.println(kd_pos);
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

  // Init both CAN buses
  Can1.begin(); Can1.setBaudRate(1000000); Can1.setMaxMB(16); Can1.enableFIFO();
  Can3.begin(); Can3.setBaudRate(1000000); Can3.setMaxMB(16); Can3.enableFIFO();
  delay(100);

  Serial.println("CAN1 @ 1Mbps (front)  CAN3 @ 1Mbps (back)  — MIT 8-DOF");
  Serial.println();
  Serial.println("!!! IMPORTANT !!!");
  Serial.println("1. All 8 motors must be in MIT mode (R-Link, once per motor).");
  Serial.println("2. Press 'e' to send Enter-MIT-Mode on both CAN buses.");
  Serial.println("3. MIT mode uses STANDARD (11-bit) CAN frames.");
  Serial.println("4. For static stand: 'e' -> 'z' -> 'j' -> observe current readout.");
  Serial.println("5. Current is estimated: I = |torque| / 4.30 N·m/A (approximate).");
  Serial.println();

  show_menu();
}

void loop() {
  drain_rx();
  handle_serial();

  // Status print every 2 seconds
  if (DEBUG_RX_STATUS) {
    uint32_t now_ms = millis();
    if (now_ms - last_status_print_ms >= 2000) {
      print_rx_status();
      last_status_print_ms = now_ms;
    }
  }

  // 100 Hz command loop (10 ms tick)
  static uint32_t last_cmd_ms = 0;
  uint32_t now = millis();
  if (now - last_cmd_ms < 10) return;
  last_cmd_ms = now;

  const float    t_s  = (float)(now - mode_t0) * 0.001f;
  const uint32_t t_ms = now - mode_t0;

  switch (mode) {

    case Mode::MENU:
      // No active commands — motors hold via their own state in MIT mode.
      // Uncomment below to actively hold while in menu:
      // send_all_pos();
      break;

    // ── Wiggle HIP on all legs ─────────────────────────────
    case Mode::WIGGLE_HIP: {
      float off = deg2rad(wiggle_amp_deg) * sinf(2.0f * (float)M_PI * wiggle_hz * t_s);
      for (int l = 0; l < 4; l++) {
        int base = l * 2;
        send_mit_pos(MOTORS[base].bus,   MOTORS[base].id,   cmd[base]   + off, kp_pos, kd_pos);
        send_mit_stop(MOTORS[base+1].bus, MOTORS[base+1].id);
      }
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle HIP done."); }
    } break;

    // ── Wiggle KNEE on all legs ────────────────────────────
    case Mode::WIGGLE_KNEE: {
      float off = deg2rad(wiggle_amp_deg) * sinf(2.0f * (float)M_PI * wiggle_hz * t_s);
      for (int l = 0; l < 4; l++) {
        int base = l * 2;
        send_mit_stop(MOTORS[base].bus,   MOTORS[base].id);
        send_mit_pos(MOTORS[base+1].bus, MOTORS[base+1].id, cmd[base+1] + off, kp_pos, kd_pos);
      }
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle KNEE done."); }
    } break;

    // ── Jog / Static Stand ────────────────────────────────
    //  All motors actively hold cmd[] targets. At zero after 'z',
    //  this is your static standing test — no keys needed.
    case Mode::JOG_NEUTRAL:
      send_all_pos();
      break;

    // ── Walk cycle — trot gait ────────────────────────────
    //  FL+BR in phase, FR+BL antiphase (diagonal pairs = trot).
    case Mode::WALK_CYCLE: {
      float w = 2.0f * (float)M_PI * walk_hz;
      // FL (index 0,1) — phase 0
      cmd[0] = hip_offset  + deg2rad(hip_amp_deg)  * sinf(w * t_s);
      cmd[1] = knee_offset + deg2rad(knee_amp_deg) * sinf(w * t_s + KNEE_PHASE);
      // FR (index 2,3) — antiphase (trot diagonal pair with BL)
      cmd[2] = hip_offset  + deg2rad(hip_amp_deg)  * sinf(w * t_s + BACK_PHASE);
      cmd[3] = knee_offset + deg2rad(knee_amp_deg) * sinf(w * t_s + BACK_PHASE + KNEE_PHASE);
      // BL (index 4,5) — antiphase with FL (trot diagonal pair with FR)
      cmd[4] = hip_offset  + deg2rad(hip_amp_deg)  * sinf(w * t_s + BACK_PHASE);
      cmd[5] = knee_offset + deg2rad(knee_amp_deg) * sinf(w * t_s + BACK_PHASE + KNEE_PHASE);
      // BR (index 6,7) — same phase as FL
      cmd[6] = hip_offset  + deg2rad(hip_amp_deg)  * sinf(w * t_s);
      cmd[7] = knee_offset + deg2rad(knee_amp_deg) * sinf(w * t_s + KNEE_PHASE);

      send_all_pos();
    } break;

    case Mode::ESTOP:
    default:
      break;
  }
}
