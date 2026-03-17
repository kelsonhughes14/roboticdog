#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ============================================================
//  CubeMars AK45-36 KV80 — Servo Mode, 3-DOF Leg Test
//  Teensy 4.1, CAN1, 1 Mbps
//
//  All send functions are clones of your working send_set_rpm:
//    tx.flags.extended = 1
//    tx.id = motor_id | (packet_type << 8)
//    payload = big-endian int32(s)
// ============================================================

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

static constexpr uint8_t MOTOR_ID_HIP_ROLL   = 0x78;
static constexpr uint8_t MOTOR_ID_HIP_PITCH  = 0x70;
static constexpr uint8_t MOTOR_ID_KNEE_PITCH = 0x71;

// Packet types
static constexpr uint8_t CAN_PACKET_SET_RPM           = 3;  // Speed Mode
static constexpr uint8_t CAN_PACKET_SET_POS           = 4;  // Position Mode
static constexpr uint8_t CAN_PACKET_SET_ORIGIN_HERE   = 5;  // Set Origin Mode
static constexpr uint8_t CAN_PACKET_SET_POS_SPD       = 6;  // Position-Speed Loop Mode

// ============================================================
//  Helpers — identical to your working code
// ============================================================
static inline void append_int32_be(uint8_t *buf, int32_t val) {
  buf[0] = (uint8_t)(val >> 24);
  buf[1] = (uint8_t)(val >> 16);
  buf[2] = (uint8_t)(val >> 8);
  buf[3] = (uint8_t)(val);
}

// ============================================================
//  Send functions — all clones of send_set_rpm
// ============================================================

// Velocity control (eRPM) — identical to your working code
static void send_set_rpm(uint8_t motor_id, int32_t eRPM) {
  CAN_message_t tx;
  tx.len = 4;
  tx.flags.extended = 1;
  tx.flags.remote   = 0;
  tx.id = (uint32_t)motor_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
  append_int32_be(tx.buf, eRPM);
  Can1.write(tx);
}

// Position only — moves at whatever speed the motor's internal speed limit is set to
// This is the simplest position command, most likely to work across firmware versions
static void send_set_pos(uint8_t motor_id, float pos_deg) {
  CAN_message_t tx;
  tx.len = 4;
  tx.flags.extended = 1;
  tx.flags.remote   = 0;
  tx.id = (uint32_t)motor_id | ((uint32_t)CAN_PACKET_SET_POS << 8);
  append_int32_be(tx.buf, (int32_t)(pos_deg * 10000.0f));
  Can1.write(tx);
}

// Position + Speed + Acceleration (8 bytes)
// pos_deg: degrees, scaled * 10000 as int32
// spd_erpm: electrical RPM, sent as int16 scaled /10 (so 50000 eRPM -> send 5000)
// accel: eRPM/s, sent as int16 scaled /10 (so 200000 eRPM/s -> send 20000)
static void send_pos_vel(uint8_t motor_id, float pos_deg,
                          int16_t spd_erpm = 5000,
                          int16_t accel = 2000) {
  CAN_message_t tx;
  tx.len = 8;
  tx.flags.extended = 1;
  tx.flags.remote   = 0;
  tx.id = (uint32_t)motor_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8);
  append_int32_be(tx.buf + 0, (int32_t)(pos_deg * 10000.0f));
  // speed and accel are int16, scaled /10 per manual
  int16_t spd_send  = (int16_t)(spd_erpm  / 10);
  int16_t accel_send = (int16_t)(accel / 10);
  tx.buf[4] = (uint8_t)(spd_send  >> 8);
  tx.buf[5] = (uint8_t)(spd_send  & 0xFF);
  tx.buf[6] = (uint8_t)(accel_send >> 8);
  tx.buf[7] = (uint8_t)(accel_send & 0xFF);
  Can1.write(tx);
}

// Set current position as zero
// permanent=0: temporary (lost on power cycle)
// permanent=1: saved to flash
static void send_set_origin(uint8_t motor_id, uint8_t permanent = 0) {
  CAN_message_t tx;
  tx.len = 1;                   // must be 1, not 0
  tx.flags.extended = 1;
  tx.flags.remote   = 0;
  tx.id = (uint32_t)motor_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8);
  tx.buf[0] = permanent;        // 0=temporary, 1=permanent
  Can1.write(tx);
}

// Stop motor
static void send_stop(uint8_t motor_id) {
  send_set_rpm(motor_id, 0);
}

// ============================================================
//  RX print — identical to your working code
// ============================================================
static void print_rx(const CAN_message_t &rx) {
  Serial.print("RX ");
  Serial.print(rx.flags.extended ? "EXT " : "STD ");
  Serial.print("ID=0x");
  Serial.print(rx.id, HEX);
  Serial.print(" DLC=");
  Serial.print(rx.len);
  Serial.print(" DATA=");
  for (int i = 0; i < rx.len; i++) {
    if (rx.buf[i] < 16) Serial.print("0");
    Serial.print(rx.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// ============================================================
//  RX status tracking
// ============================================================
struct MotorStatus {
  bool     valid   = false;
  uint32_t count   = 0;
  uint32_t last_ms = 0;
};
static MotorStatus st_roll, st_hip, st_knee;
static uint32_t    last_status_print_ms = 0;

static bool DEBUG_RAW_RX    = false;
static bool DEBUG_RX_STATUS = true;

static void drain_rx() {
  CAN_message_t rx;
  while (Can1.read(rx)) {
    if (DEBUG_RAW_RX) print_rx(rx);

    if (rx.flags.extended) {
      uint8_t src = (uint8_t)(rx.id & 0xFF);
      if (src == MOTOR_ID_HIP_ROLL)   { st_roll.valid=true; st_roll.count++;  st_roll.last_ms=millis(); }
      if (src == MOTOR_ID_HIP_PITCH)  { st_hip.valid=true;  st_hip.count++;   st_hip.last_ms=millis();  }
      if (src == MOTOR_ID_KNEE_PITCH) { st_knee.valid=true; st_knee.count++;  st_knee.last_ms=millis(); }
    }
  }
}

static void print_rx_status() {
  auto pr = [](const char* name, MotorStatus& st) {
    Serial.print(name); Serial.print(":");
    if (st.count > 0) {
      Serial.print("OK("); Serial.print(st.count); Serial.print(") ");
    } else {
      Serial.print("NO REPLY  ");
    }
    st.count = 0;
  };
  Serial.print("[RX] ");
  pr("ROLL", st_roll);
  pr("HIP",  st_hip);
  pr("KNEE", st_knee);
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

// Current position targets (degrees)
static float cmd_roll = 0.0f;
static float cmd_hip  = 0.0f;
static float cmd_knee = 0.0f;

// Motion params
static int32_t move_vel   = 5000;   // eRPM
static int32_t move_accel = 20000;  // eRPM/s

// Wiggle
static float    wiggle_amp_deg = 5.0f;
static float    wiggle_hz      = 0.8f;
static uint32_t wiggle_ms      = 3000;

// Jog step
static float jog_step_deg = 5.0f;

// Walk
static float roll_offset=0, hip_offset=0, knee_offset=0;
static float roll_amp=2.0f, hip_amp=15.0f, knee_amp=160.0f;
static float walk_hz = 0.5f;
static constexpr float KNEE_PHASE = 1.5708f; // pi/2

static void enter_mode(Mode m) { mode = m; mode_t0 = millis(); }

static void stop_all() {
  send_stop(MOTOR_ID_HIP_ROLL);
  send_stop(MOTOR_ID_HIP_PITCH);
  send_stop(MOTOR_ID_KNEE_PITCH);
}

static void estop_now() {
  stop_all();
  enter_mode(Mode::ESTOP);
  Serial.println("!!! E-STOP !!!");
}

static void show_menu() {
  Serial.println();
  Serial.println("=== 3-DOF Leg Test (Servo Mode) ===");
  Serial.println("ROLL=0x76  HIP=0x73  KNEE=0x78");
  Serial.println();
  Serial.println("  z  -> Temporary zero (lost on power cycle)");
  Serial.println("  Z  -> Permanent zero (saved to flash)");
  Serial.println("  j  -> Jog mode");
  Serial.println("  1  -> Wiggle ROLL    2  -> Wiggle HIP    3  -> Wiggle KNEE");
  Serial.println("  w  -> Walk cycle     s  -> Stop          m  -> Menu");
  Serial.println("  p  -> Toggle raw RX  o  -> Toggle status x  -> E-STOP");
  Serial.println();
  Serial.println("JOG: q/a=roll  e/d=hip  r/f=knee  ]/[=vel  }/{=accel  b=back");
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

      case '1': enter_mode(Mode::WIGGLE_ROLL); Serial.println("Wiggling ROLL..."); break;
      case '2': enter_mode(Mode::WIGGLE_HIP);  Serial.println("Wiggling HIP...");  break;
      case '3': enter_mode(Mode::WIGGLE_KNEE); Serial.println("Wiggling KNEE..."); break;
      case 'w': enter_mode(Mode::WALK_CYCLE);  Serial.println("Walking. 's' to stop."); break;
      case 'j': enter_mode(Mode::JOG_NEUTRAL); Serial.println("JOG. q/a=roll e/d=hip r/f=knee ]/[=vel }/{=accel b=back"); break;

      case 's':
        stop_all();
        enter_mode(Mode::MENU);
        Serial.println("Stopped.");
        break;

      case 'z':
        send_set_origin(MOTOR_ID_HIP_ROLL,   0);
        send_set_origin(MOTOR_ID_HIP_PITCH,  0);
        send_set_origin(MOTOR_ID_KNEE_PITCH, 0);
        cmd_roll = cmd_hip = cmd_knee = 0.0f;
        Serial.println("Temporary zero set.");
        break;

      case 'Z':
        send_set_origin(MOTOR_ID_HIP_ROLL,   1);
        send_set_origin(MOTOR_ID_HIP_PITCH,  1);
        send_set_origin(MOTOR_ID_KNEE_PITCH, 1);
        cmd_roll = cmd_hip = cmd_knee = 0.0f;
        Serial.println("Permanent zero set.");
        break;

      default:
        if (mode == Mode::JOG_NEUTRAL) {
          bool changed = true;
          switch (c) {
            case 'q': cmd_roll  += jog_step_deg; break;
            case 'a': cmd_roll  -= jog_step_deg; break;
            case 'e': cmd_hip   += jog_step_deg; break;
            case 'd': cmd_hip   -= jog_step_deg; break;
            case 'r': cmd_knee  += jog_step_deg; break;
            case 'f': cmd_knee  -= jog_step_deg; break;
            case ']': move_vel   += 1000; if (move_vel   > 50000) move_vel   = 50000; break;
            case '[': move_vel   -= 1000; if (move_vel   <   500) move_vel   =    500; break;
            case '}': move_accel += 5000; if (move_accel > 100000) move_accel = 100000; break;
            case '{': move_accel -= 5000; if (move_accel <   5000) move_accel =   5000; break;
            case 'b': enter_mode(Mode::MENU); Serial.println("Menu."); changed = false; break;
            default:  changed = false; break;
          }
          if (changed) {
            Serial.print("roll="); Serial.print(cmd_roll,  1);
            Serial.print(" hip="); Serial.print(cmd_hip,   1);
            Serial.print(" knee=");Serial.print(cmd_knee,  1);
            Serial.print(" vel="); Serial.print(move_vel);
            Serial.print(" accel=");Serial.println(move_accel);
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

  Serial.println("CAN1 @ 1Mbps — Servo Mode");
  stop_all();

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
      // No commands — motor holds on its own in servo mode
      break;

    case Mode::WIGGLE_ROLL: {
      float off = wiggle_amp_deg * sinf(2.0f * M_PI * wiggle_hz * t_s);
      send_pos_vel(MOTOR_ID_HIP_ROLL,  cmd_roll + off, move_vel, move_accel);
      send_stop(MOTOR_ID_HIP_PITCH);
      send_stop(MOTOR_ID_KNEE_PITCH);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle ROLL done."); }
    } break;

    case Mode::WIGGLE_HIP: {
      float off = wiggle_amp_deg * sinf(2.0f * M_PI * wiggle_hz * t_s);
      send_stop(MOTOR_ID_HIP_ROLL);
      send_pos_vel(MOTOR_ID_HIP_PITCH, cmd_hip  + off, move_vel, move_accel);
      send_stop(MOTOR_ID_KNEE_PITCH);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle HIP done."); }
    } break;

    case Mode::WIGGLE_KNEE: {
      float off = wiggle_amp_deg * sinf(2.0f * M_PI * wiggle_hz * t_s);
      send_stop(MOTOR_ID_HIP_ROLL);
      send_stop(MOTOR_ID_HIP_PITCH);
      send_pos_vel(MOTOR_ID_KNEE_PITCH, cmd_knee + off, move_vel, move_accel);
      if (t_ms > wiggle_ms) { enter_mode(Mode::MENU); Serial.println("Wiggle KNEE done."); }
    } break;

    case Mode::JOG_NEUTRAL: {
      static float last_sent_hip = -99999.0f;
      if (cmd_hip != last_sent_hip) {
        Serial.print(">> send_set_pos HIP pos="); Serial.print(cmd_hip, 2);
        Serial.print("deg raw="); Serial.println((int32_t)(cmd_hip * 10000.0f));
        last_sent_hip = cmd_hip;
      }
      send_set_pos(MOTOR_ID_HIP_ROLL,   cmd_roll);
      send_set_pos(MOTOR_ID_HIP_PITCH,  cmd_hip);
      send_set_pos(MOTOR_ID_KNEE_PITCH, cmd_knee);
    } break;

    case Mode::WALK_CYCLE: {
      float w = 2.0f * M_PI * walk_hz;
      cmd_roll  = roll_offset  + roll_amp  * sinf(w * t_s);
      cmd_hip   = hip_offset   + hip_amp   * sinf(w * t_s);
      cmd_knee  = knee_offset  + knee_amp  * sinf(w * t_s + KNEE_PHASE);
      send_pos_vel(MOTOR_ID_HIP_ROLL,  cmd_roll, move_vel, move_accel);
      send_pos_vel(MOTOR_ID_HIP_PITCH, cmd_hip,  move_vel, move_accel);
      send_pos_vel(MOTOR_ID_KNEE_PITCH,cmd_knee, move_vel, move_accel);
    } break;

    case Mode::ESTOP:
    default:
      break;
  }
}
