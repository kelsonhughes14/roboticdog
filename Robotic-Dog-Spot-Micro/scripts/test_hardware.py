#!/usr/bin/env python3
"""
test_hardware.py
----------------
Direct hardware test — no ROS needed.
Run this inside the container to confirm PCA9685 + servos work:

  python3 /ros2_ws/src/spotmicro/scripts/test_hardware.py

Tests each servo channel one at a time: sweeps 90° → 70° → 110° → 90°.
"""

import math
import sys
import time

# ── Config (matches robot_config.py) ─────────────────────────────────────────
I2C_BUS       = 1
ADDR_0        = 0x40   # Left  side
ADDR_1        = 0x41   # Right side
FREQUENCY     = 50
MIN_PULSE_US  = 500
MAX_PULSE_US  = 2500
PERIOD_US     = 1_000_000 // FREQUENCY   # 20 000 µs

# (board_index, channel) in servo order [FR-hip … RL-knee]
SERVO_SEQUENCE = [
    (1,  2, 'FR Hip'),      (1,  1, 'FR Shoulder'), (1,  0, 'FR Knee'),
    (0,  2, 'FL Hip'),      (0,  1, 'FL Shoulder'), (0,  0, 'FL Knee'),
    (1, 13, 'RR Hip'),      (1, 14, 'RR Shoulder'), (1, 15, 'RR Knee'),
    (0, 13, 'RL Hip'),      (0, 14, 'RL Shoulder'), (0, 15, 'RL Knee'),
]

# ── PCA9685 helpers ───────────────────────────────────────────────────────────

def pca_init(bus, addr: int) -> None:
    """Wake chip, set 50 Hz, enable auto-increment."""
    bus.write_byte_data(addr, 0x00, 0x00)   # full reset
    time.sleep(0.01)

    prescale = int(math.floor(25_000_000.0 / (4096.0 * FREQUENCY) - 0.5))
    bus.write_byte_data(addr, 0x00, 0x10)   # SLEEP=1 to change prescaler
    bus.write_byte_data(addr, 0xFE, prescale)
    bus.write_byte_data(addr, 0x00, 0x20)   # wake: AI=1, SLEEP=0
    time.sleep(0.005)
    bus.write_byte_data(addr, 0x00, 0xA0)   # RESTART + AI


def pca_set_angle(bus, addr: int, channel: int, angle_deg: float) -> None:
    pulse_us = MIN_PULSE_US + (angle_deg / 180.0) * (MAX_PULSE_US - MIN_PULSE_US)
    tick = int(pulse_us * 4096 / PERIOD_US)
    tick = max(0, min(4095, tick))
    reg = 0x06 + 4 * channel
    bus.write_i2c_block_data(addr, reg, [
        0x00, 0x00,
        tick & 0xFF, (tick >> 8) & 0x0F,
    ])


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    try:
        import smbus2
    except ImportError:
        print('ERROR: smbus2 not installed.  Run: pip3 install smbus2')
        sys.exit(1)

    bus = smbus2.SMBus(I2C_BUS)

    # Initialise both boards
    print(f'Initialising PCA9685 at 0x{ADDR_0:02X} and 0x{ADDR_1:02X}…')
    boards = {}
    for addr in (ADDR_0, ADDR_1):
        try:
            pca_init(bus, addr)
            boards[addr] = True
            print(f'  0x{addr:02X}  OK')
        except Exception as e:
            boards[addr] = False
            print(f'  0x{addr:02X}  FAILED: {e}')

    addr_map = {0: ADDR_0, 1: ADDR_1}

    print()
    print('─' * 52)
    print('  Testing each servo (90° → 70° → 110° → 90°)')
    print('  Press Enter after each wiggle, or "s" to skip,')
    print('  "q" to quit.')
    print('─' * 52)

    # Centre all servos first
    print('\nCentring all servos at 90°…')
    for board_idx, ch, _ in SERVO_SEQUENCE:
        addr = addr_map[board_idx]
        if boards[addr]:
            pca_set_angle(bus, addr, ch, 90.0)
    time.sleep(1.0)

    for i, (board_idx, ch, name) in enumerate(SERVO_SEQUENCE):
        addr = addr_map[board_idx]
        print(f'\n[{i + 1:2d}/12]  {name:<15}  (Board 0x{addr:02X}, Ch {ch})')

        if not boards[addr]:
            print('       Board not available — skipping.')
            continue

        print('       Wiggling…', end='', flush=True)
        for angle in (70.0, 110.0, 90.0):
            pca_set_angle(bus, addr, ch, angle)
            time.sleep(0.5)
        print(' done.')

        resp = input('       Correct joint moved? [Enter / s=skip / q=quit]: '
                     ).strip().lower()
        if resp == 'q':
            break

    # Return all to centre
    print('\nReturning all servos to 90°…')
    for board_idx, ch, _ in SERVO_SEQUENCE:
        addr = addr_map[board_idx]
        if boards[addr]:
            pca_set_angle(bus, addr, ch, 90.0)
    time.sleep(0.5)

    bus.close()
    print('Done.')


if __name__ == '__main__':
    main()
