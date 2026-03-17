# spotmicro

Full ROS2 Humble control package for the **Spot Micro** quadruped robot.

## Hardware

| Component | Qty | Notes |
|---|---|---|
| MG996R Servo | 12 | 3 per leg |
| Raspberry Pi 5 | 1 | Running ROS2 in Docker |
| MPU-6050 IMU | 1 | I2C address 0x68 |
| PCA9685 PWM Board | 2 | Addresses 0x40 and 0x41 |
| 7.4V LiPo Battery | 2 | One for servos, one for Pi |
| Xbox One Controller | 1 | Wired or wireless |

---

## Package Structure

```
spotmicro/
├── spotmicro/
│   ├── robot_config.py      # All physical constants and pin mappings
│   ├── kinematics.py        # Inverse kinematics solver
│   ├── gait_generator.py    # Crawl and trot gait trajectories
│   ├── servo_node.py        # PCA9685 driver node
│   ├── imu_node.py          # MPU-6050 reader node
│   ├── controller_node.py   # Xbox controller watchdog
│   ├── state_manager.py     # Robot state machine
│   └── gait_node.py         # Gait + IK → servo angles
├── config/
│   ├── xbox_controller.yaml # joy_node parameters
│   └── robot_params.yaml    # Runtime-tunable robot parameters
├── launch/
│   ├── spotmicro_launch.py  # Full system launch
│   └── calibration_launch.py
└── test/
    ├── test_kinematics.py
    └── test_gait.py
```

---

## Node Graph

```
Xbox Controller (/dev/input/js0)
        │
   [joy_node] ──/joy_raw──▶ [controller_node] ──/joy──▶ [state_manager]
                                                                │
                                               /gait_command   │  /body_pose
                                                        ▼      ▼
                                                   [gait_node]
                                                        │
                                                /servo_angles
                                                        ▼
                                                  [servo_node]
                                                        │
                                               PCA9685 x2 (I2C)
                                                        │
                                                  12x MG996R

[imu_node] ──/imu/data──▶ [state_manager]   (fall detection)
            ──/imu/euler─▶ [gait_node]      (future: balance)
```

---

## Installation

### 1. Enable I2C on the host Pi

```bash
sudo raspi-config   # Interface Options → I2C → Enable
sudo i2cdetect -y 1
# Expected: 0x40, 0x41, 0x68
```

### 2. Run Docker with hardware access

```bash
docker run -it \
  --privileged \
  --device /dev/i2c-1 \
  --device /dev/input/js0 \
  -v /dev:/dev \
  --network host \
  --name spotmicro \
  ros:humble-ros-base
```

### 3. Install Python dependencies (inside container)

```bash
pip install --break-system-packages \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-blinka \
    smbus2
```

### 4. Install ROS2 dependencies

```bash
apt update && apt install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy
```

### 5. Build the package

```bash
mkdir -p ~/ros2_ws/src
cp -r spotmicro ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running

### Full system

```bash
ros2 launch spotmicro spotmicro_launch.py
```

### Without hardware (simulation / dry run)

```bash
ros2 launch spotmicro spotmicro_launch.py dry_run:=true
```

### Servo calibration only

```bash
ros2 launch spotmicro calibration_launch.py
# Then from another terminal:
ros2 topic pub --once /servo_angles std_msgs/msg/Float32MultiArray \
  "data: [90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0]"
```

---

## Xbox One Controller Reference

| Control | Action |
|---|---|
| **LB** (hold) | Enable movement (deadman switch) |
| **A** | Stand up / Sit down (toggle) |
| **X** | Switch Gaits (Crawl, Trot, Walk) (toggle) |
| **Left Stick Y** | Walk forward / backward |
| **Left Stick X** | Strafe left / right |
| **Right Stick X** | Turn / yaw |
| **Right Stick Y** | Body pitch |
| **LT / RT** | Body roll left / right |
| **RB** | Turbo speed (2×) |
| **BACK** | **Emergency stop** |
| **START** | Clear E-stop, return to sit |

---

## Calibration

After building, tune the `SERVO_OFFSETS` in `robot_config.py`.

1. Launch calibration: `ros2 launch spotmicro calibration_launch.py`
2. Send all servos to 90°
3. For each servo, adjust its offset until the joint is mechanically neutral
4. Update `SERVO_OFFSETS` in `robot_config.py`
5. Also verify `SERVO_DIRECTION` — left-side servos may need to be `-1`

---

## Running Tests

```bash
cd ~/ros2_ws
pytest src/spotmicro/test/ -v
```

---

## Tuning Tips

- **STAND_HEIGHT** in `robot_config.py` — raise if legs are too bent, lower if overextended
- **STEP_HEIGHT** — increase if feet scuff the ground during gait
- **STEP_LENGTH** — controls stride length, reduce if the robot tips
- Start with `GaitType.CRAWL` (one foot off the ground) before enabling `TROT`
- The **complementary filter** alpha in `imu_node.py` can be lowered (e.g. 0.95) if the IMU drifts

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `i2cdetect` shows nothing | I2C not enabled or wiring issue | Run `raspi-config`, check wires |
| Servos jitter on startup | Power issue | Use separate power rail for servos, check buck converter output |
| `ImportError: board` | Adafruit Blinka not installed | `pip install adafruit-blinka` |
| Controller not detected | USB not passed to Docker | Add `--device /dev/input/js0` |
| Robot falls immediately | Wrong servo directions | Check `SERVO_DIRECTION` in `robot_config.py` |
| IK errors in logs | Leg segment lengths wrong | Measure your build and update `HIP_LENGTH`, `UPPER_LENGTH`, `LOWER_LENGTH` |
