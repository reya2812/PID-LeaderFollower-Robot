# RealTime-LeaderFollower-Control

PID-based leader–follower robotic control system using encoder feedback, ultrasonic sensing, and real-time closed-loop control. This project explores motion tracking, positional error analysis, multi-robot coordination, and collision avoidance using multiple control architectures developed across progressive milestones.

---

# Project Overview

This repository contains the complete implementation of a System Dynamics & Control mini-project focused on practical robotics and feedback control engineering.

The project progresses through multiple milestones:
- Hardware verification and basic motion control
- Cascaded PID-based positional control
- Leader–follower robotic coordination
- Final tuning, validation, and performance analysis

The system consists of:
- A leader robot
- A follower robot
- Encoder-based odometry
- Ultrasonic distance sensing
- PID-based feedback control
- Real-time drift correction and tracking

---

# Repository Structure

```text
sdc_mini_project/
│
├── bangbang-control-baseline.ino
│
├── cascaded-pid-control/
│   ├── Task2_v1/
│   │   └── Task2_v1.ino
│   └── Task2_v2/
│       └── Task2_v2.ino
│
├── leader-follower-control.ino
│
├── system-tuning-and-validation/
│   ├── README.md
│   ├── test_cases.txt
│   │
│   ├── robot_follower_final/
│   │   ├── robot_follower_final.ino
│   │   ├── analyze.py
│   │   ├── data/
│   │   ├── plots/
│   │   └── README.md
│   │
│   └── archive/
│       ├── firmware_versions/
│       ├── robot_follower_older_version/
│       ├── empty_top_level_data/
│       └── empty_top_level_plots/
│
└── README.md
```

---

# Milestone Breakdown

---

## 1. Bang-Bang Control Baseline

### File
```text
bangbang-control-baseline.ino
```

### Objectives
- Verify motors, encoders, and motor drivers
- Validate encoder interrupt handling
- Implement basic closed-loop motion control
- Analyze stopping accuracy and drift

### Features
- PWM motor control
- Encoder-based motion estimation
- Bang-bang control logic
- Differential PWM drift correction

### Key Results
- Encoder accuracy within ±7 ticks
- Approximate stopping accuracy of 10 mm

---

## 2. Cascaded PID Control

### Files
```text
cascaded-pid-control/Task2_v1/Task2_v1.ino
cascaded-pid-control/Task2_v2/Task2_v2.ino
```

### Objectives
- Implement closed-loop positional control
- Reduce positional error
- Improve motion stability and tracking

### Features
- Cascaded PID architecture
- Outer position control loop
- Inner velocity PID loops
- Alignment correction
- Low-pass velocity filtering

### Challenges Observed
- Encoder quantization noise
- Oscillatory response at low speeds
- PWM chattering
- Battery voltage sensitivity

---

## 3. Leader–Follower Coordination

### File
```text
leader-follower-control.ino
```

### Objectives
- Maintain constant distance between robots
- Implement real-time feedback coordination
- Improve tracking smoothness and robustness

### Features
- Single-loop PID control
- Ultrasonic distance sensing
- Exponential Moving Average (EMA) filtering
- Drift correction using encoder feedback
- Anti-windup protection
- PWM deadband compensation

### Key Results
- Stable leader–follower tracking
- Reduced oscillations
- Improved robustness and responsiveness

---

## 4. System Tuning and Validation

### Directory
```text
system-tuning-and-validation/
```

This section contains the final tuned implementation, experimental datasets, plots, archived firmware versions, and analysis scripts used for performance evaluation.

---

### Final System

```text
system-tuning-and-validation/robot_follower_final/
```

Contains:
- Final tuned robot firmware
- CSV logging
- Plot generation scripts
- Experimental tracking data
- Final performance plots

### Included Files
- `robot_follower_final.ino`
- `analyze.py`
- CSV logs
- Tracking plots

---

### Archive

```text
system-tuning-and-validation/archive/
```

Contains:
- Older firmware versions
- Debug versions
- Pre-final tuning iterations
- Older tracking datasets
- Experimental comparison results

---

# Experimental Metrics

The system was evaluated using:
- Positional error
- RMS tracking error
- Integral Absolute Error (IAE)
- Overshoot
- Settling time
- Tracking lag

---

# Final Experimental Results

| Metric | Result |
|---|---|
| Encoder Accuracy | ±7 ticks |
| Stopping Accuracy | ~10 mm |
| Maximum Tracking Lag | ~8 cm |
| Minimum Separation Distance | ~20 cm |
| Recovery Time | ~0.7 s |

The final implementation demonstrated:
- Stable leader–follower coordination
- Smooth recovery after disturbances
- Reliable collision avoidance
- Improved robustness under dynamic operating conditions

---

# Hardware Used

## Microcontroller
- Raspberry Pi Pico

## Sensors
- Hall-effect wheel encoders
- HC-SR04 ultrasonic sensor

## Actuation
- DC motors
- Motor drivers

## Platform
- Differential drive mobile robot

---

# Control Techniques Used

- Bang-bang control
- Cascaded PID control
- Single-loop PID control
- Low-pass filtering
- EMA filtering
- Encoder drift correction
- Anti-windup protection
- PWM deadband compensation

---

# Data Analysis

Python scripts were used to:
- Analyze tracking performance
- Generate response plots
- Evaluate steady-state behaviour
- Compare controller performance

Plots and CSV logs are included in:

```text
system-tuning-and-validation/robot_follower_final/
```

---

# Future Improvements

- LiDAR or ToF sensing
- Sensor fusion
- Adaptive gain scheduling
- Velocity feedforward control
- ROS2 integration
- Wireless communication
- SLAM-based navigation
- Model Predictive Control (MPC)

---

# Author

**Aditya Jaitly, Reya Saigal, Rohan Goyal**  
Plaksha University

---

# License

This project is intended for academic and educational purposes.
