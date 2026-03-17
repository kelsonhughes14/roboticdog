"""
servo_node.py
-------------
ROS2 node that drives the two PCA9685 PWM boards and the 12 MG996R servos.

Hardware drivers (tried in order):
  1. adafruit-blinka + adafruit-circuitpython-pca9685  (if installed)
  2. smbus2 direct I2C driver                          (built-in fallback)

Subscribed topics:
  /servo_angles  (std_msgs/Float32MultiArray)
    12-element array of servo angles in degrees [0-180].
    Order: FR_hip, FR_upper, FR_lower,
           FL_hip, FL_upper, FL_lower,
           RR_hip, RR_upper, RR_lower,
           RL_hip, RL_upper, RL_lower

Published topics:
  /servo_status  (std_msgs/String)
    JSON string with current angles and any error messages.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from spotmicro.robot_config import (
    I2C_BUS,
    NEUTRAL_ANGLES,
    PCA9685_ADDRESS_0, PCA9685_ADDRESS_1, PCA9685_FREQUENCY,
    SERVO_ANGLE_MAX, SERVO_ANGLE_MIN,
    SERVO_CHANNEL_MAP,
    SERVO_MIN_PULSE, SERVO_MAX_PULSE,
)

# ── Hardware back-end detection ───────────────────────────────────────────────
# 'adafruit' → adafruit-blinka stack is available
# 'smbus2'   → fall back to our built-in raw I2C driver
# None       → no I2C library found at all
_HW_MODE = None

try:
    import board                               # adafruit-blinka
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as adafruit_servo
    _HW_MODE = 'adafruit'
except ImportError:
    pass

if _HW_MODE is None:
    try:
        import smbus2 as _smbus2_mod
        _HW_MODE = 'smbus2'
    except ImportError:
        pass


# ── Built-in smbus2 PCA9685 driver ───────────────────────────────────────────

class _PCA9685Direct:
    """
    Minimal PCA9685 PWM driver using smbus2.
    Works with any generic PCA9685 board over I2C.
    """

    _MODE1     = 0x00
    _PRESCALE  = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, bus, address: int, frequency: int = 50,
                 min_pulse_us: int = 500, max_pulse_us: int = 2500):
        self._bus        = bus
        self._addr       = address
        self._min_pulse  = min_pulse_us
        self._max_pulse  = max_pulse_us
        self._period_us  = 1_000_000 // frequency   # 20 000 µs at 50 Hz

        # Full reset: write 0x00 to MODE1 to wake chip and clear all flags.
        # Clone PCA9685 boards often power on with SLEEP=1 (bit 4 set), which
        # prevents any PWM output until explicitly cleared.
        self._bus.write_byte_data(self._addr, self._MODE1, 0x00)
        time.sleep(0.01)   # wait for internal oscillator to stabilise

        self._set_pwm_frequency(frequency)

    def _set_pwm_frequency(self, freq_hz: int) -> None:
        # PCA9685 datasheet: prescale = floor(osc / (4096 * freq) - 0.5)
        prescale = int(math.floor(25_000_000.0 / (4096.0 * freq_hz) - 0.5))
        prescale = max(3, min(255, prescale))

        # Must be in SLEEP mode to change prescaler (datasheet requirement)
        self._bus.write_byte_data(self._addr, self._MODE1, 0x10)  # SLEEP=1
        self._bus.write_byte_data(self._addr, self._PRESCALE, prescale)

        # Wake up: AI (auto-increment, bit 5) on, SLEEP (bit 4) off
        self._bus.write_byte_data(self._addr, self._MODE1, 0x20)
        time.sleep(0.005)   # oscillator needs ≥500 µs after wake

        # RESTART (bit 7): resets all PWM counters to a clean state
        self._bus.write_byte_data(self._addr, self._MODE1, 0xA0)

    def set_angle(self, channel: int, angle_deg: float) -> None:
        """Drive channel to angle_deg (0–180°)."""
        pulse_us = self._min_pulse + (angle_deg / 180.0) * (
            self._max_pulse - self._min_pulse
        )
        tick = int(pulse_us * 4096 / self._period_us)
        tick = max(0, min(4095, tick))
        reg = self._LED0_ON_L + 4 * channel
        # Write ON=0, OFF=tick in one 4-byte block
        self._bus.write_i2c_block_data(self._addr, reg, [
            0x00, 0x00,
            tick & 0xFF, (tick >> 8) & 0x0F,
        ])


class _SmbusSevo:
    """
    Drop-in servo wrapper for _PCA9685Direct.
    Exposes the same  .angle  attribute as adafruit_motor.servo.Servo
    so _send_angles() works identically regardless of back-end.
    """

    def __init__(self, pca: _PCA9685Direct, channel: int):
        self._pca    = pca
        self._ch     = channel
        self._angle  = 90.0

    @property
    def angle(self) -> float:
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        self._angle = float(value)
        self._pca.set_angle(self._ch, self._angle)


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('dry_run', _HW_MODE is None)
        self.dry_run = self.get_parameter('dry_run').value

        if self.dry_run:
            self.get_logger().warn(
                'No I2C library available or dry_run=True — '
                'running in simulation mode (no hardware output).\n'
                '  To enable hardware: pip3 install smbus2'
            )

        # ── Hardware init ────────────────────────────────────────────
        self.servos: list = []
        self._current_angles = list(NEUTRAL_ANGLES)

        if not self.dry_run:
            self._init_hardware()

        # ── ROS interfaces ───────────────────────────────────────────
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'servo_angles',
            self._servo_callback,
            10,
        )
        self.status_pub   = self.create_publisher(String, 'servo_status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Servo node ready  [driver={_HW_MODE}  dry_run={self.dry_run}]'
        )

        # Move to neutral on startup
        self._send_angles(NEUTRAL_ANGLES)

    # ────────────────────────────────────────────────────────────────
    def _init_hardware(self):
        try:
            if _HW_MODE == 'adafruit':
                self._init_adafruit()
            elif _HW_MODE == 'smbus2':
                self._init_smbus2()
            else:
                raise RuntimeError('No supported I2C library found.')
        except Exception as e:
            self.get_logger().error(f'Hardware init failed: {e}')
            self.dry_run = True

    def _init_adafruit(self):
        i2c  = busio.I2C(board.SCL, board.SDA)
        pca0 = PCA9685(i2c, address=PCA9685_ADDRESS_0)
        pca0.frequency = PCA9685_FREQUENCY
        pca1 = PCA9685(i2c, address=PCA9685_ADDRESS_1)
        pca1.frequency = PCA9685_FREQUENCY
        self._pca_boards = [pca0, pca1]

        self.servos = [None] * 12
        for (leg, joint), (board_idx, channel) in SERVO_CHANNEL_MAP.items():
            pca = self._pca_boards[board_idx]
            self.servos[leg * 3 + joint] = adafruit_servo.Servo(
                pca.channels[channel],
                min_pulse=SERVO_MIN_PULSE,
                max_pulse=SERVO_MAX_PULSE,
            )
        self.get_logger().info(
            f'[adafruit] PCA9685 at 0x{PCA9685_ADDRESS_0:02X} '
            f'and 0x{PCA9685_ADDRESS_1:02X} ready'
        )

    def _init_smbus2(self):
        bus  = _smbus2_mod.SMBus(I2C_BUS)
        pca0 = _PCA9685Direct(bus, PCA9685_ADDRESS_0,
                              frequency=PCA9685_FREQUENCY,
                              min_pulse_us=SERVO_MIN_PULSE,
                              max_pulse_us=SERVO_MAX_PULSE)
        pca1 = _PCA9685Direct(bus, PCA9685_ADDRESS_1,
                              frequency=PCA9685_FREQUENCY,
                              min_pulse_us=SERVO_MIN_PULSE,
                              max_pulse_us=SERVO_MAX_PULSE)
        self._pca_boards = [pca0, pca1]

        self.servos = [None] * 12
        for (leg, joint), (board_idx, channel) in SERVO_CHANNEL_MAP.items():
            pca = self._pca_boards[board_idx]
            self.servos[leg * 3 + joint] = _SmbusSevo(pca, channel)

        self.get_logger().info(
            f'[smbus2] PCA9685 at 0x{PCA9685_ADDRESS_0:02X} '
            f'and 0x{PCA9685_ADDRESS_1:02X} ready  (I2C bus {I2C_BUS})'
        )

    # ────────────────────────────────────────────────────────────────
    def _servo_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(
                f'Expected 12 servo angles, got {len(msg.data)}'
            )
            return
        self._send_angles(list(msg.data))

    def _send_angles(self, angles: list):
        for i, angle in enumerate(angles):
            clamped = max(SERVO_ANGLE_MIN, min(SERVO_ANGLE_MAX, float(angle)))
            self._current_angles[i] = clamped
            if not self.dry_run and self.servos and self.servos[i] is not None:
                try:
                    self.servos[i].angle = clamped
                except Exception as e:
                    self.get_logger().error(
                        f'Servo {i} → {clamped:.1f}° failed: {e}'
                    )

    # ────────────────────────────────────────────────────────────────
    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'angles':  [round(a, 2) for a in self._current_angles],
            'driver':  _HW_MODE,
            'dry_run': self.dry_run,
        })
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Returning servos to neutral…')
        self._send_angles(NEUTRAL_ANGLES)
        time.sleep(0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
