# CELERINO · High-Speed Line Follower Robot

**Line Follower Robot with PID Control, 16-Channel IR Mux, Suction Turbine and ESP32-C3**  
Competitive Robotics / Embedded Software Project — Instituto Tecnológico de Chihuahua (ITCH) · 2026

## Description

**CELERINO** is a high-performance line follower robot designed for speed robotics competitions. Its architecture combines an ESP32-C3 SuperMini microcontroller, a multiplexed data acquisition system with 16 analog infrared sensors, motor speed control through high-current BTN9960 H-bridges with thermal and overcurrent protection, and an aerodynamic suction system using an ESC-driven turbine to maximize grip in curves.

The firmware is written in C on the **ESP-IDF** framework, using a finite state machine driven by the interrupts of an infrared receiver for start/stop, position calculation through a local centroid (weighted average), and real-time closed-loop PID control.

---

## Main Features

* **Processing Control:** ESP32-C3 SuperMini running native ESP-IDF firmware with hardware timing (`esp_timer`) and 12-bit ADC reading.
* **Modular Sensor Bar:** 16 IR phototransistors/LEDs switched by a 74HC4067 (16:1) analog multiplexer, sampled in microseconds and processed using local centroid estimation.
* **Power Stage:** 4 BTN9960 half H-bridges forming full H-bridges for both motors, with overvoltage support, linear PWM response (5 kHz) and overcurrent dissipation into the ground plane.
* **Suction System (Ground Effect):** Turbine driven by a brushless motor and controlled via ESC (50 Hz PWM) with an automatic acceleration ramp in accordance with competition rules.
* **Infrared Remote Control:** Real-time decoding of the NEC protocol through GPIO interrupts for start, calibration and emergency stop commands.
* **Custom Mechanical Transmission:** 5:1 reduction gears 3D-printed in PETG for high durability and interchangeability.

---

## System Architecture

```
                      ┌───────────────────────────┐
                      │   IR Remote Control (NEC) │
                      └─────────────┬─────────────┘
                                    │ GPIO0 Interrupt
                                    ▼
┌──────────────────┐  16:1 MUX   ┌───────────────────┐  PWM (5 kHz)  ┌──────────────────┐
│ 16-Sensor        │────────────>│ ESP32-C3          │──────────────>│ BTN9960          │───> DC Motors
│ IR Bar           │  ADC GPIO4  │ SuperMini         │               │ H-Bridges        │     (Traction)
└──────────────────┘             │                   │               └──────────────────┘
                                 │ (ESP-IDF / C)     │  PWM (50 Hz)  ┌──────────────────┐
                                 │ 2 ms PID loop     │──────────────>│ ESC / Turbine    │───> Suction
                                 └───────────────────┘               │ (Fan)            │     (Ground Effect)
                                                                     └──────────────────┘
```

---

## Hardware and Components

| Subsystem | Component | Notes |
| --- | --- | --- |
| **Controller** | ESP32-C3 SuperMini | 32-bit RISC-V, high-speed ADC |
| **Line Sensors** | 16 IR Sensor Array + 74HC4067 MUX | Multiplexed 16-channel sampling at 12 bits |
| **Motor Drivers** | 4x BTN9960 (Half H-Bridges) | Bidirectional control, thermal and overcurrent protection |
| **Turbine / Suction** | Brushless Motor + ESC | Generates negative pressure for grip in curves |
| **IR Receiver** | 38 kHz Infrared Receiver (GPIO 1) | Reads ASCII commands via NEC protocol |
| **Transmission** | PETG Gears (5:1 Reduction) | 3D printed, interchangeable gear ratio |
| **Chassis** | SolidWorks Design / Modular PCB | Separate main board and sensor bar |

---

## Pin Mapping (ESP32-C3 SuperMini)

| Peripheral | GPIO Pin | Function |
| --- | --- | --- |
| **MUX S0** | GPIO 3 | Select bit 0 of the 74HC4067 multiplexer |
| **MUX S1** | GPIO 2 | Select bit 1 of the 74HC4067 multiplexer |
| **MUX S2** | GPIO 10 | Select bit 2 of the 74HC4067 multiplexer |
| **MUX S3** | GPIO 7 | Select bit 3 of the 74HC4067 multiplexer |
| **ADC MUX** | GPIO 4 (ADC1_CH4) | Analog output of the multiplexer |
| **Left Motor IN1 / IN2** | GPIO 21 / GPIO 20 | LEDC PWM Channel 0 / 1 (5 kHz) |
| **Right Motor IN1 / IN2** | GPIO 5 / GPIO 6 | LEDC PWM Channel 2 / 3 (5 kHz) |
| **Turbine ESC** | GPIO 8 | LEDC PWM Channel 4 (50 Hz / Servo Protocol) |
| **Indicator LED** | GPIO 9 | Digital Output (White indicator) |
| **IR Receiver** | GPIO 1 | Digital Input with Negedge Interrupt |

---

## Firmware and Control Algorithm

The software runs in a periodic **2 ms** control loop structured as a finite state machine:

```
                  ┌──────────────┐
                  │  STATE_IDLE  │
                  └──────┬───────┘
                         │
        ┌────────────────┴────────────────┐
        ▼                                 ▼
┌───────────────────────┐       ┌───────────────────┐
│   STATE_CALIBRATING   │       │   STATE_RUNNING   │
│ 5 s auto-calibration  │       │ PID Loop + Motors │
└───────────────────────┘       └─────────┬─────────┘
                                          │ IR_CMD_STOP
                                          ▼
                                ┌───────────────────┐
                                │   STATE_STOPPED   │
                                └───────────────────┘
```

### Position Estimator Algorithm
1. **Per-Sensor Normalization:** Individual scaling of each sensor ($0 \dots 1000$) using the minimum and maximum values captured during the calibration phase.
2. **Noise Filtering:** Subtraction of a noise floor (`NOISE_FLOOR`) to eliminate stray readings away from the line.
3. **Local Centroid (Blobbing):** Identification of the peak sensor and inclusion of only its contiguous active neighbors ($\pm \text{BLOB\_HALF}$), preventing jumps caused by reflections on the track.
4. **IIR Filter and Jump Limit:** Position smoothing to avoid abrupt discontinuities.

---

## Infrared Commands (Remote Control)

| Command | Hex Code | Action |
| --- | --- | --- |
| **CALIBRATE** | `0x45` | Enters calibration mode (5 seconds of min/max sampling per step) |
| **START** | `0x46` | Starts safety sequence (5 s delay + turbine ramp + PID loop start) |
| **STOP** | `0x47` | Emergency stop of motors and turbine |

---

## Repository Structure

```
Celerino-LineFollower/
├── main/
│   ├── main.c              # Main code in C (ESP-IDF)
│   └── CMakeLists.txt      # Build configuration of the main component
├── CMakeLists.txt          # CMake project
└── README.md               # General repository documentation
```

---

## Software and Build Requirements

| Tool | Recommended Version |
| --- | --- |
| **ESP-IDF** | v5.x |
| **RISC-V Toolchain** | Included in ESP-IDF |
| **EasyEDA** | For PCB design / Gerber |
| **SolidWorks** | For viewing and modifying 3D parts |

### Build and Upload via ESP-IDF

```bash
# Set the target to the ESP32-C3
idf.py set-target esp32c3

# Build the project
idf.py build

# Flash the microcontroller and open serial monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Development Team

**Firmware, Electronics and Mechanical Development:**

* Ana Paola Cardona Valenzuela
* Emiliano Pérez Dyck
* Luis Adrián Anchondo Carreón

**Institution:** Instituto Tecnológico de Chihuahua (TecNM / ITCH)  
**Department:** Metal-Mechanics / Electrical and Electronic Engineering  
**Specialty:** Embedded Systems and Competitive Robotics  
**Year:** 2026

---

<p align="center">
  Instituto Tecnológico de Chihuahua · Chihuahua, Mexico · 2026
</p>
