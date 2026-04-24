# Trash Terminator

An intelligent trash sorting robot built on Raspberry Pi and Arduino Mega. Uses MobileNetV2 for classification and ultrasonic PID stepping for precise navigation.

## System Architecture

```
Raspberry Pi (Host)                        Arduino Mega (Controller)
+------------------------+                +------------------------+
|  Pi Camera (CSI)       |                |  4x Ultrasonic sensors |
|  MobileNetV2 Classifier|  <--- UART --->|  4x Mecanum wheel motors|
|  Background Subtraction|   115200bps    |  Arm + Gripper servos    |
|  Serial Comm + Control |                |  PID stepping           |
|  Voice (espeak-ng)     |                |  OLED status display    |
+------------------------+                +------------------------+
```

## Sorting Sequence

1. **Position** -- PID ultrasonic step to 25cm from wall + wall alignment
2. **Recognize** -- AI captures image, classifies trash (metal/plastic), auto-centers
3. **Grab** -- Open gripper -> step to 6cm -> close gripper
4. **Transport** -- Step back -> rotate 180 deg -> step to 5cm from drop wall
5. **Drop** -- Strafe to type-specific zone -> release
6. **Return** -- Rotate -> align -> return to start position

## Project Structure

```
trash-terminator/
├── firmware/
│   └── trash_robot_firmware.ino       # Arduino firmware
├── src/
│   ├── robot_serial.py                # Serial communication library
│   ├── manual_test.py                 # Manual motion test (no AI)
│   ├── auto_sort.py                   # AI auto sorting main program
│   ├── model/
│   │   ├── classifier_mobile.pt       # MobileNetV2 classification model
│   │   └── bg_empty.jpg               # Background reference image
│   └── config/
│       ├── calibration.json           # Motion calibration params
│       └── grip_angles.json           # Gripper angle config
├── requirements.txt
├── .gitignore
└── README.md
```

## Hardware

| Component | Description |
|-----------|-------------|
| Arduino Mega 2560 | Motor/sensor controller |
| Raspberry Pi 4/5 | AI host (camera + serial) |
| Pi Camera (CSI) | Image capture (libcamera) |
| 4x HC-SR04 | Ultrasonic distance sensors |
| 4x Mecanum wheels + motors | Omnidirectional chassis |
| 2x Servos | Arm + gripper |
| SSD1306 OLED 128x32 | Status display |

### Pin Mapping (Arduino)

```
Ultrasonic:
  Front-Left:  TRIG=48, ECHO=47     Front-Right: TRIG=40, ECHO=41
  Side-Left:   TRIG=33, ECHO=32     Side-Right:  TRIG=30, ECHO=29

Motors (PWM + DIR):
  A: PWM=12, DIR1=34, DIR2=35      B: PWM=8,  DIR1=37, DIR2=36
  C: PWM=6,  DIR1=43, DIR2=42      D: PWM=5,  DIR1=A4, DIR2=A5

Servos:
  Arm: PIN=44     Gripper: PIN=45

OLED: I2C (SDA/SCL), address 0x3C
```

## Quick Start

### 1. Flash Firmware

```bash
# Open in Arduino IDE, install deps: Adafruit GFX, Adafruit SSD1306
# Board: Arduino Mega 2560 -> Compile & Upload
```

### 2. Pi Setup

```bash
sudo apt update
sudo apt install -y python3-pip espeak-ng python3-picamera2
pip install -r requirements.txt
```

### 3. Calibrate

Edit `src/config/calibration.json` to match your hardware:

- `TIME_FWD_10CM` / `TIME_STRAFE_10CM` / `TIME_ROTATE_180` -- motion timing (ms)
- `FWD_LEFT_ADJ` / `BWD_LEFT_ADJ` -- wheel drift compensation
- `WALL_TOL` / `FRONT_DIST_TOL` -- PID tolerance thresholds

### 4. Run

```bash
cd src

# Manual test (no AI, verify motion)
python3 manual_test.py

# AI auto sorting
python3 auto_sort.py
```

## Serial Protocol

### Motion

| Command | Format | Description |
|---------|--------|-------------|
| Forward | `FWD:pwm:ms` | Timed forward |
| Backward | `BWD:pwm:ms` | Timed backward |
| Strafe L | `SL:pwm:ms` | Timed strafe left |
| Strafe R | `SR:pwm:ms` | Timed strafe right |
| Rotate | `ROT:angle` | Timed rotation (degrees) |
| Stop | `STOP` | Emergency stop |

### PID Control

| Command | Format | Description |
|---------|--------|-------------|
| Front dist | `FRONTPID:target` | PID step to target distance (cm) |
| Wall align | `WALIGN` | PID align to wall (left vs right sensor) |

### Arm

| Command | Description |
|---------|-------------|
| `ARMHOME` | Return arm to home position |
| `GRAB` | Close gripper |
| `RELEASE` | Open gripper |

### Sequences

| Command | Description |
|---------|-------------|
| `SEQ:METAL` | Execute metal sorting path |
| `SEQ:PLASTIC` | Execute plastic sorting path |
| `SEQ:AUTO` | Auto mode (waits for Pi classification) |

### Config

| Command | Format | Description |
|---------|--------|-------------|
| Set | `SET:param:value` | Runtime parameter update |
| Get | `GET:param` | Query current parameter |
| Dist | `DIST` | Read all 4 ultrasonic distances |

## AI Details

- **Model**: MobileNetV2 (ImageNet pretrained, fine-tuned)
- **Classes**: metal, plastic
- **Detection**: Background subtraction (OpenCV) for object localization
- **Centering**: Compute pixel offset -> strfe robot to align
- **Confidence threshold**: 0.5 (configurable via `CONF_THRESH`)
- <img width="1280" height="1280" alt="846ef0b3a4f2560510eb6900d5174c5" src="https://github.com/user-attachments/assets/62948326-fc7a-4fdf-9173-ebabaf56b51a" />


## License

MIT
