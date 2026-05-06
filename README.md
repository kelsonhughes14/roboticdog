# Dog — Quadruped Robot ROS2 Package

Full control stack for a large-scale 8-DOF quadruped robot. Runs on a Jetson Orin Nano (ROS2 Humble) with motor control delegated to a Teensy 4.1 over USB (micro-ROS). Includes teleoperation, autonomous navigation (Nav2 + SLAM), and a Gazebo Fortress simulation.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Software Architecture](#software-architecture)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Teensy Firmware](#teensy-firmware)
- [First-Time Motor Calibration](#first-time-motor-calibration)
- [Running the Robot](#running-the-robot)
  - [Hardware Mode (normal operation)](#hardware-mode-normal-operation)
  - [Simulation Mode](#simulation-mode)
  - [Autonomous Navigation Mode](#autonomous-navigation-mode)
- [Startup Procedure (every session)](#startup-procedure-every-session)
- [Controls Reference](#controls-reference)
  - [Xbox Controller](#xbox-controller)
  - [PS4 Controller](#ps4-controller)
  - [Keyboard](#keyboard)
- [Configuration Files](#configuration-files)
- [Node Reference](#node-reference)
- [Troubleshooting](#troubleshooting)

---

## Hardware Overview

| Component | Part | Notes |
|-----------|------|-------|
| Compute | NVIDIA Jetson Orin Nano | Runs ROS2 Humble |
| MCU | PJRC Teensy 4.1 | micro-ROS firmware, CAN bus driver |
| Actuators | CubeMars AK45-36 (×8) | MIT mini-cheetah CAN protocol, 24 V, 36:1 gear ratio |
| Power | mjbots power_dist r4.5b | 24 V motor bus, 5 V Teensy VIN |
| IMU | Bosch BNO085 | Connected to Teensy (I2C, SDA=18 SCL=19, addr 0x4A) |
| GPS | u-blox SAM-M10Q | Connected to Teensy (Serial1, RX=0 TX=1, 9600 baud) |
| Lidar | Hokuyo UTM-30LX | Autonomous mode only; USB serial |
| Controller | Xbox One S or PS4 DualShock 4 | Bluetooth via USB dongle or wired |

### Robot Geometry

- **Legs:** 4 legs × 2 joints each = 8 DOF. Hip joints are physically present as static 3D-printed dummies (original hip motors failed).
- **Leg segments:** Hip link = 95 mm, Femur (upper) = 240 mm, Tibia (lower) = 230 mm
- **Body:** 720 mm front-to-rear, 480 mm side-to-side
- **Default stand height:** 350 mm (body above ground)

### CAN Bus Wiring

```
Teensy CAN1  (TX=22, RX=23)  ──► SN65HVD230 transceiver ──► Front legs (FR + FL)
Teensy CAN3  (TX=31, RX=30)  ──► SN65HVD230 transceiver ──► Rear  legs (RR + RL)
```

Each bus requires 120 Ω termination resistors at both physical ends (between CANH and CANL).

### Motor IDs

Set with the CubeMars R-Link USB adapter and CubeMars debugging software:

| Leg | Joint | CAN bus | Motor ID (hex) | Motor ID (dec) |
|-----|-------|---------|----------------|----------------|
| FR | Shoulder | CAN1 | 0x79 | 121 |
| FR | Knee | CAN1 | 0x7A | 122 |
| FL | Shoulder | CAN1 | 0x75 | 117 |
| FL | Knee | CAN1 | 0x78 | 120 |
| RR | Shoulder | CAN3 | 0x73 | 115 |
| RR | Knee | CAN3 | 0x74 | 116 |
| RL | Shoulder | CAN3 | 0x76 | 118 |
| RL | Knee | CAN3 | 0x77 | 119 |

---

## Software Architecture

```
Joystick / Nav2 cmd_vel
       │
       ▼
 state_manager          ← /joy, /estop, /joint_states
       │  publishes /gait_command, /robot_state, /joint_angles (direct), /can_enable
       ▼
  gait_node             ← /gait_command, /robot_state, /gait_type, /body_pose
       │  runs gait generator + IK at 100 Hz
       │  publishes /joint_angles (8 motor rads), /joint_gains
       ▼
micro_ros_agent ←→ Teensy 4.1
  (USB serial)          drives CAN motors, publishes /joint_states, /imu/euler
       │
       ▼
  AK45-36 Motors (×8)
```

In **simulation mode**, `sim_bridge_node` replaces `micro_ros_agent` + Teensy by translating `/joint_angles` → Gazebo position controllers and forwarding Gazebo IMU + odometry back to ROS2.

In **autonomous mode**, `autonomous_bridge_node` sits between Nav2's `/cmd_vel` and `/gait_command`, and slam_toolbox + Nav2 handle path planning.

### Belt-Drive Coupling

The knee motor sits at the body and drives the tibia via a timing belt along the femur (MIT Mini Cheetah layout). Its encoder encodes knee angle relative to the **body**, not the femur. Consequently:

- **Command:** `knee_motor_cmd = shoulder_ik + knee_ik`
- **Gazebo decode:** `geometric_knee = motor_knee − motor_shoulder`

This is handled automatically inside `kinematics.py` and `sim_bridge_node.py`. You do not need to manually account for it.

---

## Prerequisites

### On the Jetson Orin Nano

- **OS:** Ubuntu 22.04 (JetPack 6.x)
- **ROS2:** Humble (`ros-humble-desktop`)
- **micro-ROS agent:**

  ```bash
  sudo snap install micro-ros-agent
  # or build from source:
  # https://micro.ros.org/docs/tutorials/core/first_application_linux/
  ```

- **ROS2 packages** (install via apt):

  ```bash
  sudo apt install \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-rviz2 \
    ros-humble-tf2-ros
  ```

- **urg_node2** (Hokuyo lidar driver — must be built from source for Humble):

  ```bash
  cd ~/ros2_ws/src
  git clone https://github.com/ros-drivers/urg_node2.git
  cd .. && colcon build --packages-select urg_node2
  ```

- **Gazebo Fortress** (simulation only):

  ```bash
  sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
    ros-humble-position-controllers ros-humble-joint-state-broadcaster
  ```

### Environment Variables

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7        # must match Teensy firmware (hardcoded to 7)
```

---

## Installation & Build

```bash
cd ~/ros2_ws/src
git clone <repo-url> dog        # or it's already here at src/dog
cd ~/ros2_ws
colcon build --packages-select dog
source install/setup.bash
```

Rebuild whenever you change Python source files or config files.

---

## Teensy Firmware

The firmware lives in `teensy_firmware/PlatformIO/Projects/micro_ros_teensy41/`.

### Build & Flash

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. Open the project folder in PlatformIO.
3. Connect the Teensy 4.1 via USB.
4. Build and upload:

   ```bash
   cd teensy_firmware/PlatformIO/Projects/micro_ros_teensy41
   pio run --target upload
   ```

### Key Firmware Constants

These are hardcoded in `src/main.cpp` and must match the rest of the system:

| Constant | Value | Note |
|----------|-------|------|
| `ROS_DOMAIN_ID` | 7 | Must match Jetson `$ROS_DOMAIN_ID` |
| CAN baud | 1 Mbps | AK45-36 factory default |
| Watchdog timeout | 2000 ms | Motors exit MIT mode if no `/joint_angles` arrives |
| Thermal cutoff | 75 °C | Per motor |
| Instantaneous current | 2.5 A | Per motor |
| Sustained stall torque | 14.0 N·m | Per motor, 2-second window |

### Teensy LED Status

| Pattern | Meaning |
|---------|---------|
| Slow blink (500 ms) | Waiting for micro-ROS agent connection |
| Solid ON | Agent connected, ready to receive commands |

---

## First-Time Motor Calibration

This sets each motor's electrical zero to its mechanical zero position. **You only need to do this once per motor, or after replacing a motor.**

### Mechanical Zero Definitions

- **Shoulder:** Upper leg (femur) pointing straight **down** toward the floor (vertical, no forward lean).
- **Knee:** Lower leg (tibia) fully **extended**, inline with the upper leg (leg appears straight).

### Calibration Procedure

1. Support the robot in the air (hang it or hold it) so all legs can swing freely.

2. Start the calibration launch (bridges Teensy only, no gait):

   ```bash
   ros2 launch dog calibration_launch.py
   ```

3. In a second terminal, run the interactive calibration tool:

   ```bash
   ros2 run dog calibration_node
   ```

4. **Phase 1 — Motor ID verification:** Each motor nudges ±0.2 rad. Confirm the correct physical joint moved before continuing.

5. **Phase 2 — Zero position:** For each joint, type degree values to drive the motor toward mechanical zero, then type `z` to save. After setting zero on any motor, the Teensy automatically re-enters motor mode on all motors.

6. **Phase 3 — Verification:** All motors command 0.0 rad. Check that shoulders point straight down and knees are fully extended. If any joint is off, adjust `JOINT_OFFSETS` in `dog/robot_config.py` and rebuild.

---

## Running the Robot

### Hardware Mode (normal operation)

```bash
# Xbox controller (default)
ros2 launch dog dog_launch.py

# PS4 controller
ros2 launch dog dog_launch.py ctrl:=ps4

# Keyboard teleoperation (no controller needed)
ros2 launch dog dog_launch.py ctrl:=keyboard

# Override Teensy USB port (check with: ls /dev/ttyACM*)
ros2 launch dog dog_launch.py serial_port:=/dev/ttyACM0
```

### Simulation Mode

Runs the full control stack against Gazebo Fortress. No Teensy or motors required.

```bash
# Basic simulation
ros2 launch dog dog_launch.py sim:=true

# Simulation with keyboard control
ros2 launch dog dog_launch.py sim:=true ctrl:=keyboard

# Simulation + autonomous navigation
ros2 launch dog dog_launch.py sim:=true autonomous:=true
```

The robot spawns in Gazebo at z=0.65 m. Gazebo provides the IMU and lidar data; `sim_bridge_node` translates the control stack's motor commands into Gazebo position controller commands.

### Autonomous Navigation Mode

Runs Nav2 + SLAM Toolbox on top of the normal hardware stack. Requires the Hokuyo UTM-30LX lidar.

```bash
# Hardware + autonomous
ros2 launch dog dog_launch.py autonomous:=true

# Override lidar port (check with: ls /dev/ttyACM*)
ros2 launch dog dog_launch.py autonomous:=true lidar_port:=/dev/ttyACM0

# If you have an external odometry/localization source publishing odom→base_link:
ros2 launch dog dog_launch.py autonomous:=true fallback_odom_tf:=false
```

**Nav2 startup delay:** Nav2 starts 15 seconds after launch to give SLAM time to produce its first map→odom TF. This is normal.

**Setting a navigation goal:** Open RViz (launches automatically), select the "Nav2 Goal" tool in the toolbar, and click on the map.

---

## Startup Procedure (every session)

Follow this sequence every time you power on the robot for normal hardware operation:

1. **Power on** the mjbots power_dist board (motors + Teensy). Wait for Teensy LED to begin blinking slowly.

2. **Start the launch file** on the Jetson:

   ```bash
   ros2 launch dog dog_launch.py
   ```

   Wait until the terminal shows `micro_ros_agent` connecting and `State manager ready`. The Teensy LED should go solid ON.

3. **Connect the controller.** Verify with `ros2 topic echo /joy` that axis/button data is arriving.

4. **Press Start** (Xbox) or **Options** (PS4). The robot enters **POSITIONING** state — motors become torque-free and you can move legs by hand.

5. **Choose your startup path:**

   - **Option A — Auto-stand (motors were already calibrated):**
     Press Start again immediately. The robot ramps to calibrated neutral angles and stands up automatically over ~2.5 seconds.

   - **Option B — Manual capture (first session or after recalibration):**
     Move each leg to its neutral standing position by hand. Press A/B/X/Y to lock each leg:
     - **A** → lock Front-Right (FR)
     - **B** → lock Front-Left (FL)
     - **X** → lock Rear-Right (RR)
     - **Y** → lock Rear-Left (RL)

     Then press Start to confirm. The robot stands using the captured angles as its home position.

6. **Set the robot on the ground.** Hold LB + move the left stick to begin walking.

7. **To stop:** Release the stick or LB → robot returns to STANDING. Press Back (View) for emergency stop.

---

## Controls Reference

### Xbox Controller

| Button / Axis | Action |
|---------------|--------|
| **Start** (Menu) | ESTOP clear → POSITIONING; or POSITIONING confirm → STANDING |
| **Back** (View) | Emergency stop (ESTOP) |
| **A** | Standup recovery animation (press in STANDING or WALKING) |
| **B** | Jump forward |
| **RB + B** | Backflip |
| **X** | Cycle gait (SHUFFLE → CRAWL → WALK → DIAG_TROT → TROT → GALLOP → STEP → TURTLE) |
| **Y** | Toggle autonomous mode on/off |
| **LB** (hold) | Deadman — enables locomotion while held |
| **RB** (hold) | Turbo speed (2× velocity) |
| **Left stick Y** | Forward / backward |
| **Left stick X** | Strafe left / right |
| **Right stick X** | Turn left / right |
| **Right stick Y** | Body pitch (nose up/down) |
| **LT / RT** (analog) | Body roll (left/right tilt) |

During **POSITIONING** only:

| Button | Action |
|--------|--------|
| **A** | Lock Front-Right (FR) leg at current position |
| **B** | Lock Front-Left (FL) leg |
| **X** | Lock Rear-Right (RR) leg |
| **Y** | Lock Rear-Left (RL) leg |
| **Start** | Confirm all locked legs and stand up |

### PS4 Controller

Same layout as Xbox with these mappings:

| PS4 | Xbox equivalent |
|-----|----------------|
| Square (□) | A |
| Cross (×) | B (jump / backflip with R1) |
| Circle (○) | X |
| Triangle (△) | Y |
| L1 | LB |
| R1 | RB |
| Share | Back (ESTOP) |
| Options | Start |

**Note:** DS4 driver axis layout differs from xpadneo. If sticks feel wrong, check `AXIS_RIGHT_X/Y` in `robot_config.py` against `ros2 topic echo /joy`.

### Keyboard

Launch with `ctrl:=keyboard`. Run in a dedicated terminal:

```bash
ros2 run dog keyboard_node
```

| Key | Action |
|-----|--------|
| **Enter** | ESTOP clear → POSITIONING; or POSITIONING confirm → stand |
| **Backspace** | Emergency stop (ESTOP) |
| **w / s** | Forward / backward |
| **a / d** | Strafe left / right |
| **q / e** | Turn left / right |
| **W/A/S/D/Q/E** | Same, at turbo speed |
| **Space** | Standup recovery animation |
| **g** | Cycle gait |
| **h** | Jump back to SHUFFLE gait immediately |
| **y** | Toggle autonomous mode |
| **j** | Jump forward |
| **b** | Backflip |
| **r** | Reset simulation (sim mode only) |
| **Ctrl+C** | Quit |

During **POSITIONING** only:

| Key | Action |
|-----|--------|
| **1** | Lock FR leg |
| **2** | Lock FL leg |
| **3** | Lock RR leg |
| **4** | Lock RL leg |
| **Enter** | Confirm and stand |

---

## Configuration Files

All config files are in `config/`. Changes take effect on next `colcon build` + relaunch.

| File | Controls |
|------|----------|
| `robot_params.yaml` | Gait rate (Hz), start angles, shuffle trim, joystick scaling |
| `xbox_controller.yaml` | joy_node deadzone, autorepeat rate, device ID |
| `ps4_controller.yaml` | Same for PS4 |
| `nav2_params.yaml` | Nav2 planner, controller, costmap settings |
| `slam_params.yaml` | SLAM Toolbox resolution, loop closure, scan matching |
| `ros2_controllers.yaml` | Gazebo ros2_control joint controller config |
| `rviz_nav.rviz` | RViz layout for autonomous navigation |

### Key Parameters in `robot_params.yaml`

```yaml
gait_node:
  control_rate_hz: 50          # gait loop rate (must match Teensy watchdog expectations)
  upper_start_angle_deg: 45.0  # forward lean of femur at stand (affects nominal stand_z)
  lower_start_angle_deg: 45.0  # rearward lean of tibia at stand
  shuffle_lift_upper_trim_deg: 0.0   # shoulder trim during SHUFFLE lift
  shuffle_lift_lower_trim_deg: 20.0  # knee trim during SHUFFLE lift (+value = more tuck)
```

### Key Values in `robot_config.py`

Edit this file to match your physical build:

- `HIP_LENGTH`, `UPPER_LENGTH`, `LOWER_LENGTH` — physical link lengths in mm
- `STAND_HEIGHT` — target body height above ground in mm
- `NEUTRAL_ANGLES` — pre-computed IK angles for the default standing pose (radians, motor frame)
- `JOINT_OFFSETS` — fine-tuning after calibration if any joint is slightly off zero
- `JOINT_DIRECTION` — set to -1 if a motor rotates opposite to the IK convention

---

## Node Reference

| Node | Launch file includes it? | Purpose |
|------|--------------------------|---------|
| `state_manager` | Always | Robot state machine; reads joystick, publishes motor enable/disable and gait commands |
| `gait_node` | Always | Gait generator + IK at 100 Hz; publishes `/joint_angles` |
| `controller_node` | Always | Joystick watchdog; re-publishes `/joy_raw` as `/joy` with 1 s timeout |
| `imu_leveling_node` | Always | Smooths BNO085 roll/pitch and publishes `/level_correction` for body leveling |
| `torque_monitor_node` | Hardware only | Logs per-motor telemetry to CSV; forwards motor faults to `/estop` |
| `sim_bridge_node` | Simulation only | Translates `/joint_angles` → Gazebo controllers; Gazebo IMU → `/imu/euler` |
| `autonomous_bridge_node` | Autonomous mode | Routes Nav2 `/cmd_vel` → `/gait_command` when in AUTONOMOUS state; obstacle hold |
| `calibration_node` | `calibration_launch.py` | Interactive CAN motor calibration |
| `keyboard_node` | `ctrl:=keyboard` | Keyboard teleoperation (curses UI) |

### ROS Topics (key)

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/joy` | `sensor_msgs/Joy` | `controller_node` | `state_manager`, `keyboard_node` |
| `/robot_state` | `std_msgs/String` | `state_manager` | `gait_node`, `autonomous_bridge_node`, `keyboard_node` |
| `/gait_command` | `geometry_msgs/Twist` | `state_manager` / `autonomous_bridge_node` | `gait_node` |
| `/gait_type` | `std_msgs/String` | `state_manager` / `autonomous_bridge_node` | `gait_node` |
| `/joint_angles` | `std_msgs/Float32MultiArray` [8] | `gait_node` / `state_manager` | Teensy (via `micro_ros_agent`) / `sim_bridge_node` |
| `/joint_gains` | `std_msgs/Float32MultiArray` [2] | `gait_node` / `state_manager` | Teensy |
| `/joint_states` | `std_msgs/Float32MultiArray` [32] | Teensy | `gait_node`, `state_manager`, `torque_monitor_node` |
| `/can_enable` | `std_msgs/Bool` | `state_manager` | Teensy |
| `/imu/euler` | `geometry_msgs/Vector3` | Teensy / `sim_bridge_node` | `imu_leveling_node` |
| `/motor_fault` | `std_msgs/UInt8` | Teensy | `torque_monitor_node` → `/estop` |
| `/estop` | `std_msgs/Bool` | External / `torque_monitor_node` | `state_manager` |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | `autonomous_bridge_node` |
| `/scan` | `sensor_msgs/LaserScan` | `hokuyo` / Gazebo bridge | SLAM, Nav2, `autonomous_bridge_node` |

---

## Troubleshooting

### Teensy LED blinks but no motor response
- Check that `micro_ros_agent` is running: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1`
- Verify `ROS_DOMAIN_ID=7` is exported on the Jetson shell.
- Check USB serial port: `ls /dev/ttyACM*`. Pass the correct port with `serial_port:=`.

### Motors jerk violently on startup
- The robot entered POSITIONING before you set it down. Make sure to enter POSITIONING while the robot is on the ground (or supported), then set it down **after** confirming.
- If it happens mid-session, press Back (ESTOP) immediately. Check for motor ID conflicts with the calibration tool.

### Motors go to wrong positions (wrong joint moves)
- Motor IDs are incorrectly assigned. Re-run Phase 1 of the calibration tool to verify each joint.
- Update `CAN_ID_MAP` in `robot_config.py` to match the actual motor IDs.

### Robot falls sideways / leans heavily to one side
- `JOINT_DIRECTION` may be wrong for one or more motors. See the direction verification instructions at the top of `JOINT_DIRECTION` in `robot_config.py`.
- Run the calibration tool Phase 2 and re-zero any motor that is off.

### Joystick not detected
- Check `/dev/input/jsX`: `ls /dev/input/js*`
- `device_id` in `xbox_controller.yaml` or `ps4_controller.yaml` defaults to `0`. If another device claims js0, change it.
- Verify with: `ros2 topic echo /joy`

### Motor fault / thermal shutdown
- Check `/motor_fault` bitmask: `ros2 topic echo /motor_fault`
- Check temperatures in `/motor_temps`: `ros2 topic echo /motor_temps`
- Allow motors to cool (thermal cutoff = 75 °C). The robot must be re-enabled via Start → POSITIONING after a fault.
- If faults occur frequently at low load, check that `MOTOR_KT_EFF` in `robot_config.py` matches the actual motor.

### SLAM or Nav2 fails to start / "extrapolation into past"
- This is usually a TF timing issue at launch. Nav2 intentionally starts 15 s after the launch file to give SLAM time to emit its first map→odom TF. Wait for it.
- Ensure `/scan` is arriving: `ros2 topic hz /scan`. If not, check the lidar port (`lidar_port:=`) and that `urg_node2` is built.
- If you have an odometry source, set `fallback_odom_tf:=false` and ensure it publishes the `odom→base_link` TF.

### Gazebo simulation: robot falls or spins on spawn
- The spawn happens 3 s after Gazebo starts. If Gazebo is slow (first launch, no GPU), increase the `period=3.0` delay in `dog_launch.py` for the spawn `TimerAction`.
- Controllers load 4 s after Gazebo starts. If "controller_manager not ready" loops appear, this is normal on the first launch — wait.

### "No /joint_angles received" watchdog fires mid-session
- The Teensy exits motor mode if no `/joint_angles` arrives for 2 seconds. This can happen if the Jetson is overloaded or the ROS2 DDS stack drops messages.
- Check CPU load. The next `/joint_angles` automatically re-enables the motors (watchdog auto-recovery).

---

## Telemetry Logs

`torque_monitor_node` writes a CSV to `~/dog/Logs/` with one row per motor per sample:

```
timestamp_s, motor, position_rad, velocity_rad_s, torque_nm, current_a, temp_c
```

Use these logs to diagnose thermal behaviour, motor loading, and calibration drift over time.

---

## Other Launch Files

| Launch file | Purpose |
|-------------|---------|
| `dog_launch.py` | Main — hardware, simulation, autonomous (all modes) |
| `calibration_launch.py` | Motor calibration only (micro_ros_agent + no gait) |
| `stand_launch.py` | Minimal standing test (no gait, no autonomous) |
| `leg_test_launch.py` | Single motor test |
| `autonomous_launch.py` | Nav2 + SLAM only (assumes control stack already running) |
