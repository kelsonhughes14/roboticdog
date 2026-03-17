/*
 * spotmicro_teensy — main.cpp
 * --------------------
 * micro-ROS firmware for Teensy 4.1
 *
 * Hardware connected to Teensy:
 *   PCA9685 #0 (0x40)  — FL + RL legs  — I2C Wire  (pins 18/19)
 *   PCA9685 #1 (0x41)  — FR + RR legs  — I2C Wire  (pins 18/19)
 *   MPU-6050   (0x68)  — IMU            — I2C Wire1 (pins 16/17)
 *
 * micro-ROS transport: USB serial (native Teensy USB)
 *
 * ROS topics
 * ----------
 * Subscribed:
 *   /servo_angles  (std_msgs/Float32MultiArray)
 *     12 floats [deg]: FR_hip, FR_shoulder, FR_knee,
 *                      FL_hip, FL_shoulder, FL_knee,
 *                      RR_hip, RR_shoulder, RR_knee,
 *                      RL_hip, RL_shoulder, RL_knee
 * Published:
 *   /imu/data   (sensor_msgs/Imu)        — 50 Hz
 *   /imu/euler  (geometry_msgs/Vector3)  — 50 Hz
 *
 * LED indicator
 * -------------
 *   Slow blink (500 ms)  — waiting for micro-ROS agent
 *   Solid ON             — agent connected
 */

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─────────────────────────────────────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────────────────────────────────────

#define PCA_ADDR_0     0x40
#define PCA_ADDR_1     0x41
#define PCA_FREQ_HZ    50
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
#define SERVO_MIN_DEG  0.0f
#define SERVO_MAX_DEG  180.0f

#define MPU6050_ADDR   0x68
#define IMU_RATE_HZ    50

static const uint8_t SERVO_BOARD[12]   = {1, 1, 1,  0, 0, 0,  1, 1, 1,  0, 0, 0};
static const uint8_t SERVO_CHANNEL[12] = {2, 1, 0,  2, 1, 0, 13,14,15, 13,14,15};

// ─────────────────────────────────────────────────────────────────────────────
// MPU-6050 registers
// ─────────────────────────────────────────────────────────────────────────────
#define MPU_PWR_MGMT_1    0x6B
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG_REG    0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_GYRO_XOUT_H   0x43

#define ACCEL_SCALE  16384.0f   // LSB/g  for ±2g
#define GYRO_SCALE   131.0f     // LSB/(°/s) for ±250°/s
#define G_MS2        9.80665f

// ─────────────────────────────────────────────────────────────────────────────
// Hardware objects
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pca0(PCA_ADDR_0);
Adafruit_PWMServoDriver pca1(PCA_ADDR_1);

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS handles
// ─────────────────────────────────────────────────────────────────────────────
rcl_node_t         node;
rclc_support_t     support;
rcl_allocator_t    allocator;
rclc_executor_t    executor;

rcl_subscription_t servo_sub;
rcl_publisher_t    imu_pub;
rcl_publisher_t    euler_pub;
rcl_timer_t        imu_timer;

std_msgs__msg__Float32MultiArray servo_msg;
sensor_msgs__msg__Imu            imu_msg;
geometry_msgs__msg__Vector3      euler_msg;

float servo_data_buf[12];

// ─────────────────────────────────────────────────────────────────────────────
// Complementary filter state
// ─────────────────────────────────────────────────────────────────────────────
static float cf_roll    = 0.0f;
static float cf_pitch   = 0.0f;
static unsigned long cf_last_us = 0;
static const float CF_ALPHA = 0.98f;

// ─────────────────────────────────────────────────────────────────────────────
// Agent connection state machine
// ─────────────────────────────────────────────────────────────────────────────
enum AgentState { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agent_state = WAITING_AGENT;

// ─────────────────────────────────────────────────────────────────────────────
// Servo helpers
// ─────────────────────────────────────────────────────────────────────────────
void set_servo_angle(uint8_t board_idx, uint8_t channel, float angle_deg) {
    angle_deg = constrain(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    uint16_t pulse_us = (uint16_t)(
        SERVO_MIN_US + (angle_deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US)
    );
    if (board_idx == 0) pca0.writeMicroseconds(channel, pulse_us);
    else                pca1.writeMicroseconds(channel, pulse_us);
}

void send_angles(const float *angles) {
    for (int i = 0; i < 12; i++)
        set_servo_angle(SERVO_BOARD[i], SERVO_CHANNEL[i], angles[i]);
}

// ─────────────────────────────────────────────────────────────────────────────
// MPU-6050 helpers
// ─────────────────────────────────────────────────────────────────────────────
void mpu_init() {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU_PWR_MGMT_1);  Wire1.write(0x00);
    Wire1.endTransmission();

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU_SMPLRT_DIV);  Wire1.write(0x07);
    Wire1.endTransmission();

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU_CONFIG_REG);  Wire1.write(0x00);
    Wire1.endTransmission();

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU_GYRO_CONFIG);  Wire1.write(0x00);
    Wire1.endTransmission();

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU_ACCEL_CONFIG);  Wire1.write(0x00);
    Wire1.endTransmission();
}

int16_t mpu_read_word(uint8_t reg) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    Wire1.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, true);
    return (Wire1.read() << 8) | Wire1.read();
}

void mpu_read(float &ax, float &ay, float &az,
              float &gx, float &gy, float &gz) {
    ax = mpu_read_word(MPU_ACCEL_XOUT_H)     / ACCEL_SCALE * G_MS2;
    ay = mpu_read_word(MPU_ACCEL_XOUT_H + 2) / ACCEL_SCALE * G_MS2;
    az = mpu_read_word(MPU_ACCEL_XOUT_H + 4) / ACCEL_SCALE * G_MS2;

    gx = radians(mpu_read_word(MPU_GYRO_XOUT_H)     / GYRO_SCALE);
    gy = radians(mpu_read_word(MPU_GYRO_XOUT_H + 2) / GYRO_SCALE);
    gz = radians(mpu_read_word(MPU_GYRO_XOUT_H + 4) / GYRO_SCALE);
}

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS callbacks
// ─────────────────────────────────────────────────────────────────────────────
void servo_callback(const void *msg_in) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size == 12)
        send_angles(msg->data.data);
}

void imu_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/) {
    if (timer == NULL) return;

    float ax, ay, az, gx, gy, gz;
    mpu_read(ax, ay, az, gx, gy, gz);

    unsigned long now_us = micros();
    float dt = (cf_last_us == 0) ? 0.02f : (now_us - cf_last_us) * 1e-6f;
    cf_last_us = now_us;

    float accel_roll  = degrees(atan2f(ay, az));
    float accel_pitch = degrees(atan2f(-ax, sqrtf(ay*ay + az*az)));

    cf_roll  = CF_ALPHA * (cf_roll  + degrees(gx) * dt) + (1.0f - CF_ALPHA) * accel_roll;
    cf_pitch = CF_ALPHA * (cf_pitch + degrees(gy) * dt) + (1.0f - CF_ALPHA) * accel_pitch;

    int64_t stamp_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec     = (int32_t)(stamp_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

    imu_msg.angular_velocity.x    = gx;
    imu_msg.angular_velocity.y    = gy;
    imu_msg.angular_velocity.z    = gz;
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    rcl_publish(&imu_pub, &imu_msg, NULL);

    euler_msg.x = cf_roll;
    euler_msg.y = cf_pitch;
    euler_msg.z = 0.0;

    rcl_publish(&euler_pub, &euler_msg, NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS entity lifecycle
// ─────────────────────────────────────────────────────────────────────────────
bool create_entities() {
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&node, "teensy_node", "", &support) != RCL_RET_OK)
        return false;

    if (rclc_subscription_init_default(
            &servo_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "servo_angles") != RCL_RET_OK)
        return false;

    if (rclc_publisher_init_default(
            &imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu/data") != RCL_RET_OK)
        return false;

    if (rclc_publisher_init_default(
            &euler_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "imu/euler") != RCL_RET_OK)
        return false;

    if (rclc_timer_init_default(
            &imu_timer, &support,
            RCL_MS_TO_NS(1000 / IMU_RATE_HZ),
            imu_timer_callback) != RCL_RET_OK)
        return false;

    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK)
        return false;

    rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg,
                                   &servo_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &imu_timer);

    rmw_uros_sync_session(1000);
    return true;
}

void destroy_entities() {
    // Set destroy timeout to 0 so fini calls don't block waiting for the agent
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&imu_pub,   &node);
    rcl_publisher_fini(&euler_pub, &node);
    rcl_subscription_fini(&servo_sub, &node);
    rcl_timer_fini(&imu_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // ── Hardware init ──────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000);

    Wire1.begin();
    Wire1.setClock(400000);

    mpu_init();
    delay(100);

    pca0.begin();
    pca0.setOscillatorFrequency(27000000);
    pca0.setPWMFreq(PCA_FREQ_HZ);

    pca1.begin();
    pca1.setOscillatorFrequency(27000000);
    pca1.setPWMFreq(PCA_FREQ_HZ);

    // Boot at safe center — ROS2 commands the actual pose after connecting
    static const float SAFE_ANGLES[12] = {
        90.0f, 90.0f, 90.0f,
        90.0f, 90.0f, 90.0f,
        90.0f, 90.0f, 90.0f,
        90.0f, 90.0f, 90.0f,
    };
    send_angles(SAFE_ANGLES);

    // ── One-time message field init (no micro-ROS needed) ──────────
    servo_msg.data.data     = servo_data_buf;
    servo_msg.data.size     = 0;
    servo_msg.data.capacity = 12;

    static char frame_id_str[] = "imu_link";
    imu_msg.header.frame_id.data     = frame_id_str;
    imu_msg.header.frame_id.size     = strlen(frame_id_str);
    imu_msg.header.frame_id.capacity = sizeof(frame_id_str);

    for (int i = 0; i < 9; i++) {
        imu_msg.angular_velocity_covariance[i]    = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
        imu_msg.orientation_covariance[i]         = 0.0;
    }
    imu_msg.angular_velocity_covariance[0]    = 0.01;
    imu_msg.angular_velocity_covariance[4]    = 0.01;
    imu_msg.angular_velocity_covariance[8]    = 0.01;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    imu_msg.orientation_covariance[0]         = -1.0;
}

void loop() {
    static unsigned long last_blink_ms = 0;
    static unsigned long last_ping_ms  = 0;

    switch (agent_state) {

        case WAITING_AGENT:
            // Slow blink while waiting
            if (millis() - last_blink_ms > 500) {
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                last_blink_ms = millis();
            }
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (create_entities()) {
                    agent_state  = AGENT_CONNECTED;
                    last_ping_ms = millis();
                    digitalWrite(LED_BUILTIN, HIGH);
                }
            }
            break;

        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            // Only ping every 500 ms — not every loop iteration
            if (millis() - last_ping_ms > 500) {
                last_ping_ms = millis();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    destroy_entities();
                    agent_state = AGENT_DISCONNECTED;
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
            break;

        case AGENT_DISCONNECTED:
            // Return servos to safe position, then wait to reconnect
            {
                static const float SAFE[12] = {
                    90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f,
                    90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f,
                };
                send_angles(SAFE);
            }
            agent_state = WAITING_AGENT;
            break;
    }
}
