<div align="center">

<!-- Animated Header Wave -->
<img width="100%" src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=6,17,30&height=220&section=header&text=Autonomous%20Surface%20Vehicle&fontSize=60&fontColor=fff&animation=twinkling&fontAlignY=32&desc=🌊%20Closed-Loop%20Navigation%20•%20EKF%20Sensor%20Fusion%20•%20APF%20Obstacle%20Avoidance%20🤖&descAlignY=52&descAlign=50&descSize=18"/>

<!-- Animated Tagline -->
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=600&size=20&duration=3000&pause=1000&color=00D9FF&center=true&vCenter=true&width=900&lines=Real-Time+GPS+Navigation+with+Kalman+Filtering+📡;Extended+Kalman+Filter+for+Drift-Free+Positioning+🎯;Artificial+Potential+Field+Obstacle+Avoidance+🧲;ESP32-Powered+Autonomous+Marine+Navigation+🚤;Differential+Drive+System+with+IMU+Fusion+🧭" alt="Typing SVG" />
</p>

<!-- Hero Badges -->
<p align="center">
  <img src="https://img.shields.io/badge/🌊-Marine_Robotics-00D9FF?style=for-the-badge&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/🤖-Autonomous_Navigation-10B981?style=for-the-badge&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/📡-GPS_&_IMU_Fusion-6366F1?style=for-the-badge&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/⚡-ESP32_Powered-FF6B35?style=for-the-badge&labelColor=1a1b27"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Extended_Kalman_Filter-Sensor_Fusion-FFD93D?style=for-the-badge&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/APF_Algorithm-Obstacle_Avoidance-A855F7?style=for-the-badge&labelColor=1a1b27"/>
</p>

<!-- Project Stats -->
<p align="center">
  <img src="https://img.shields.io/badge/License-Apache_2.0-4ECDC4?style=flat-square&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/Platform-ESP32-00D9FF?style=flat-square&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/Status-Active_Development-10B981?style=flat-square&labelColor=1a1b27"/>
  <img src="https://img.shields.io/badge/PRs-Welcome-FF6B35?style=flat-square&labelColor=1a1b27"/>
</p>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

</div>

<br>

## <img src="https://media.giphy.com/media/WUlplcMpOCEmTGBtBW/giphy.gif" width="35"> Project Overview

<img align="right" width="420" src="https://media.giphy.com/media/l0HlDHQEiIdY3kxlm/giphy.gif">

This project implements a **real-time, closed-loop navigation system** for an **Autonomous Surface Vehicle (ASV)** using cutting-edge algorithms and sensor fusion techniques.

### 🎯 **Core Technologies**

- 🧮 **Extended Kalman Filter (EKF)** — Drift-free sensor fusion
- 🧲 **Artificial Potential Field (APF)** — Dynamic obstacle avoidance
- 📡 **GPS + IMU Integration** — Precise positioning
- 🤖 **Differential Drive Control** — Autonomous navigation
- ⚡ **Real-Time Processing** — Sub-second decision making

### 🌊 **Navigation Philosophy**

Combining sensor fusion with intelligent path planning to create a fully autonomous marine vehicle capable of navigating complex environments with obstacles, GPS dropouts, and dynamic conditions.

<br clear="right">

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/j2pOGeGYKe2xCCKwfi/giphy.gif" width="35"> Key Features

<div align="center">

<table>
<tr>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/3oKIPnAiaMCws8nOsE/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/📡-GPS_Tracking-00D9FF?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Quectel L89-S90**
<br><br>
✓ Real-time location
<br>
✓ Lat/Lon to XY conversion
<br>
✓ Meter-level accuracy
<br>
✓ NMEA parsing
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/26tn33aiTi1jkl6H6/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/🧭-IMU_Fusion-6366F1?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**BNO055 9-DOF**
<br><br>
✓ Heading estimation
<br>
✓ Acceleration data
<br>
✓ Gyroscope fusion
<br>
✓ Magnetometer calibration
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/qgQUggAC3Pfv687qPC/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/🎯-Kalman_Filtering-10B981?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Extended Kalman Filter**
<br><br>
✓ Sensor fusion
<br>
✓ Drift correction
<br>
✓ Velocity estimation
<br>
✓ Position smoothing
</td>
</tr>
<tr>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/l0HlBO7eyXzSZkJri/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/🧲-APF_Navigation-A855F7?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Potential Field Planning**
<br><br>
✓ Attractive force to goal
<br>
✓ Repulsive from obstacles
<br>
✓ Closed-loop control
<br>
✓ Real-time adjustment
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/3oKIPnAiaMCws8nOsE/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/📏-Ultrasonic_Sensing-FFD93D?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**HC-SR04 Sensors**
<br><br>
✓ Distance measurement
<br>
✓ Obstacle detection
<br>
✓ Multi-sensor array
<br>
✓ Real-time feedback
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/l0HlDHQEiIdY3kxlm/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/🤖-Motor_Control-FF6B35?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Differential Drive**
<br><br>
✓ L298N motor driver
<br>
✓ 2 rear DC motors
<br>
✓ 1 passive caster wheel
<br>
✓ Smooth maneuvering
</td>
</tr>
</table>

</div>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/iY8CRBdQXODJSCERIr/giphy.gif" width="35"> Hardware Architecture

<div align="center">

### 🔧 **Component Breakdown**

</div>

<table>
<tr>
<td width="50%" valign="top">

### 🎛️ **Core Components**

<br>

**⚡ ESP32 DevKit V1**
<br>
<img src="https://img.shields.io/badge/Microcontroller-Main_Brain-00D9FF?style=flat-square&labelColor=1a1b27"/>
<br>
Dual-core 240MHz processor, WiFi/BT ready, real-time processing capability for sensor fusion and motor control.

<br><br>

**📡 Quectel L89-S90 GPS**
<br>
<img src="https://img.shields.io/badge/Navigation-Position_Tracking-4ECDC4?style=flat-square&labelColor=1a1b27"/>
<br>
High-precision GPS module with NMEA output. Connected via UART (RX2/TX2). Provides lat/lon coordinates converted to local XY meters.

<br><br>

**🧭 Adafruit BNO055 IMU**
<br>
<img src="https://img.shields.io/badge/Orientation-9--DOF_Sensor-A855F7?style=flat-square&labelColor=1a1b27"/>
<br>
9-axis absolute orientation sensor with built-in sensor fusion. I2C interface (SDA=21, SCL=22). Provides heading, acceleration, gyroscope data.

<br><br>

**📏 HC-SR04 Ultrasonic Sensors**
<br>
<img src="https://img.shields.io/badge/Proximity-Distance_Sensing-10B981?style=flat-square&labelColor=1a1b27"/>
<br>
1-3 sensors for obstacle detection. Range: 2cm to 400cm. Real-time distance feedback for APF algorithm.

</td>
<td width="50%" valign="top">

### ⚙️ **Actuation & Power**

<br>

**🚗 L298N Motor Driver**
<br>
<img src="https://img.shields.io/badge/Driver-Dual_H--Bridge-F59E0B?style=flat-square&labelColor=1a1b27"/>
<br>
Dual H-bridge motor driver. Controls 2 rear DC motors independently for differential drive. PWM speed control.

<br><br>

**⚙️ DC Motors (2x Rear)**
<br>
<img src="https://img.shields.io/badge/Propulsion-Differential_Drive-FFD93D?style=flat-square&labelColor=1a1b27"/>
<br>
Two rear-mounted DC motors for movement. Single passive front caster wheel. Enables forward motion and turning.

<br><br>

**🔋 Power System**
<br>
<img src="https://img.shields.io/badge/Battery-7.4V_/_12V-EF4444?style=flat-square&labelColor=1a1b27"/>
<br>
7.4V or 12V battery pack for motor power. Separate 5V regulation for ESP32 and sensors. Proper power isolation.

<br><br>

**🔌 Wiring Configuration**
<br>
<img src="https://img.shields.io/badge/Connections-I2C_/_UART_/_GPIO-6366F1?style=flat-square&labelColor=1a1b27"/>
<br>
- GPS: UART (GPIO16/17)
- IMU: I2C (GPIO21/22)
- Motors: PWM + Digital pins
- Ultrasonic: Trigger/Echo pins

</td>
</tr>
</table>

<div align="center">

<br>

### 🔋 **Power Distribution**

```
Battery (7.4V-12V)
    │
    ├──> L298N Motor Driver ──> DC Motors (×2)
    │
    └──> 5V Regulator
            │
            ├──> ESP32 DevKit
            ├──> GPS Module
            ├──> BNO055 IMU
            └──> Ultrasonic Sensors
```

</div>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/fwbZnTftCXVocKzfxR/giphy.gif" width="35"> Navigation Stack

<div align="center">

### 🧠 **Three-Layer Intelligence System**

</div>

<table>
<tr>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/26tn33aiTi1jkl6H6/giphy.gif" width="120"/>
<br><br>
<img src="https://img.shields.io/badge/LAYER_1-Sensor_Fusion-00D9FF?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**GPS + IMU + EKF**
<br><br>
📡 Raw GPS coordinates
<br>
🧭 IMU heading & acceleration
<br>
🔄 Kalman filter fusion
<br>
📍 Drift-free position estimate
<br><br>
**Output:** Accurate XY position & velocity
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/l0HlBO7eyXzSZkJri/giphy.gif" width="120"/>
<br><br>
<img src="https://img.shields.io/badge/LAYER_2-Path_Planning-10B981?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Artificial Potential Field**
<br><br>
🎯 Attractive force → Goal
<br>
🧲 Repulsive force ← Obstacles
<br>
📏 Ultrasonic distance data
<br>
⚖️ Force vector summation
<br><br>
**Output:** Desired heading & velocity
</td>
<td width="33%" align="center">
<img src="https://media.giphy.com/media/qgQUggAC3Pfv687qPC/giphy.gif" width="120"/>
<br><br>
<img src="https://img.shields.io/badge/LAYER_3-Motor_Control-FF6B35?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Differential Drive**
<br><br>
🔄 Heading error calculation
<br>
⚡ Left/right motor PWM
<br>
🎮 Forward, turn, stop commands
<br>
🔁 Closed-loop adjustment
<br><br>
**Output:** Motor commands (L/R speed)
</td>
</tr>
</table>

<br>

### 🔄 **Complete Navigation Pipeline**

```mermaid
graph LR
    A[GPS Data] --> D[EKF Fusion]
    B[IMU Data] --> D
    C[Ultrasonic] --> E[APF Algorithm]
    D --> E
    E --> F[Motor Controller]
    F --> G[Vehicle Motion]
    G --> A
```

<div align="center">

**Closed-loop system with continuous feedback and correction**

</div>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/WUlplcMpOCEmTGBtBW/giphy.gif" width="35"> Algorithm Deep Dive

### 🎯 **Extended Kalman Filter (EKF)**

<img align="right" width="380" src="https://media.giphy.com/media/3oKIPnAiaMCws8nOsE/giphy.gif">

**Purpose:** Fuse GPS and IMU data to get drift-free position and velocity estimates.

**Why EKF?**
- GPS has **low frequency** but **absolute position**
- IMU has **high frequency** but **drifts over time**
- EKF combines both for **optimal estimation**

**Process:**
1. **Prediction Step:** Use IMU acceleration to predict next state
2. **Update Step:** Correct prediction using GPS measurements
3. **Covariance Update:** Track uncertainty in estimates
4. **Output:** Smooth, accurate position & velocity

**Key Parameters:**
- Process noise: Models IMU drift
- Measurement noise: Models GPS accuracy
- State vector: [x, y, vx, vy]

<br clear="right">

<br>

### 🧲 **Artificial Potential Field (APF)**

<img align="right" width="380" src="https://media.giphy.com/media/l0HlBO7eyXzSZkJri/giphy.gif">

**Purpose:** Generate navigation commands by simulating attractive and repulsive forces.

**How It Works:**

**Attractive Force (Goal)**
```
F_attractive = K_att * (Goal - Current_Position)
```
Pulls the vehicle toward the target destination.

**Repulsive Force (Obstacles)**
```
F_repulsive = K_rep * (1/distance - 1/threshold) * direction
```
Pushes the vehicle away from detected obstacles.

**Total Force**
```
F_total = F_attractive + Σ F_repulsive
```

**Advantages:**
- ✅ Real-time obstacle avoidance
- ✅ Smooth trajectory generation
- ✅ No pre-computed path needed
- ✅ Adapts to dynamic environments

<br clear="right">

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/LnQjpWaON8nhr21vNW/giphy.gif" width="35"> Installation & Setup

### 📋 **Prerequisites**

<p>
<img src="https://img.shields.io/badge/Arduino_IDE-1.8.19+-00979D?style=for-the-badge&logo=arduino&logoColor=white&labelColor=1a1b27"/>
<img src="https://img.shields.io/badge/PlatformIO-Optional-FF7F00?style=for-the-badge&logo=platformio&logoColor=white&labelColor=1a1b27"/>
<img src="https://img.shields.io/badge/ESP32_Board_Package-2.0+-000000?style=for-the-badge&logo=espressif&logoColor=white&labelColor=1a1b27"/>
</p>

### 🔧 **Required Libraries**

```cpp
// Install via Arduino Library Manager
#include <Adafruit_BNO055.h>      // IMU sensor library
#include <TinyGPS++.h>              // GPS parsing library
#include <Wire.h>                   // I2C communication
```

### 📥 **Installation Steps**

<table>
<tr>
<td width="33%" align="center">
<img src="https://img.shields.io/badge/STEP_1-Clone_Repository-FF6B35?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
```bash
git clone https://github.com/
yourusername/asv-navigation.git
cd asv-navigation
```
<br>
Clone the project repository
</td>
<td width="33%" align="center">
<img src="https://img.shields.io/badge/STEP_2-Install_Libraries-00D9FF?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
Open Arduino IDE
<br>
Install required libraries
<br>
Configure ESP32 board
<br>
Set COM port
</td>
<td width="33%" align="center">
<img src="https://img.shields.io/badge/STEP_3-Upload_Code-10B981?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
Connect ESP32 via USB
<br>
Open main .ino file
<br>
Click Upload
<br>
Monitor serial output
</td>
</tr>
</table>

### 🔌 **Hardware Connections**

<div align="center">

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| **GPS RX** | GPIO 17 (TX2) | UART communication |
| **GPS TX** | GPIO 16 (RX2) | UART communication |
| **IMU SDA** | GPIO 21 | I2C data line |
| **IMU SCL** | GPIO 22 | I2C clock line |
| **Motor IN1** | GPIO 25 | Left motor control |
| **Motor IN2** | GPIO 26 | Left motor control |
| **Motor IN3** | GPIO 27 | Right motor control |
| **Motor IN4** | GPIO 14 | Right motor control |
| **Ultrasonic Trig** | GPIO 5 | Trigger pulse |
| **Ultrasonic Echo** | GPIO 18 | Echo response |

</div>

### ⚙️ **Configuration**

```cpp
// Adjust these parameters in config.h
#define GOAL_LAT 37.7749  // Target latitude
#define GOAL_LON -122.4194 // Target longitude
#define OBSTACLE_THRESHOLD 50 // cm
#define K_ATTRACTIVE 1.0  // APF attractive gain
#define K_REPULSIVE 2.0   // APF repulsive gain
```

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/j2pOGeGYKe2xCCKwfi/giphy.gif" width="35"> System Operation

### 🚀 **Startup Sequence**

```
1. Initialize serial communication (115200 baud)
2. Initialize I2C for BNO055 IMU
3. Initialize UART for GPS module
4. Calibrate IMU (wait for good calibration values)
5. Wait for GPS fix (valid NMEA data)
6. Initialize EKF with first GPS position
7. Enter main navigation loop
```

### 🔄 **Main Loop Execution**

<div align="center">

```mermaid
graph TD
    A[Read GPS] --> B[Read IMU]
    B --> C[EKF Prediction]
    C --> D[EKF Update]
    D --> E[Read Ultrasonic]
    E --> F[Compute APF Forces]
    F --> G[Calculate Desired Heading]
    G --> H[Motor Control Commands]
    H --> I[Apply PWM to Motors]
    I --> A
```

</div>

### 📊 **Real-Time Monitoring**

```cpp
// Serial output every loop iteration
Position: (12.34, 56.78) m
Velocity: (0.5, 0.2) m/s
Heading: 45.2°
Goal Distance: 15.3 m
Obstacle: 35 cm (FRONT)
Motor: L=180 R=150
Status: NAVIGATING
```

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/iY8CRBdQXODJSCERIr/giphy.gif" width="35"> Performance Characteristics

<div align="center">

<table>
<tr>
<td width="50%" align="center">
<img src="https://media.giphy.com/media/26tn33aiTi1jkl6H6/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/Position_Accuracy-±2_meters-10B981?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**GPS + EKF Fusion**
<br><br>
Typical accuracy with consumer GPS
<br>
Improved with IMU fusion
<br>
Better than GPS-only systems
</td>
<td width="50%" align="center">
<img src="https://media.giphy.com/media/qgQUggAC3Pfv687qPC/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/Update_Rate-10_Hz-00D9FF?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Real-Time Processing**
<br><br>
100ms loop time
<br>
Fast enough for marine navigation
<br>
Smooth motion control
</td>
</tr>
<tr>
<td width="50%" align="center">
<img src="https://media.giphy.com/media/3oKIPnAiaMCws8nOsE/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/Obstacle_Detection-2cm_to_4m-FFD93D?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**Ultrasonic Range**
<br><br>
HC-SR04 specifications
<br>
Adjustable avoidance threshold
<br>
Multi-sensor coverage
</td>
<td width="50%" align="center">
<img src="https://media.giphy.com/media/l0HlBO7eyXzSZkJri/giphy.gif" width="100"/>
<br><br>
<img src="https://img.shields.io/badge/Heading_Accuracy-±2_degrees-A855F7?style=for-the-badge&labelColor=1a1b27"/>
<br><br>
**IMU Precision**
<br><br>
BNO055 absolute orientation
<br>
Magnetometer calibration
<br>
Drift compensation
</td>
</tr>
</table>

</div>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/LnQjpWaON8nhr21vNW/giphy.gif" width="35"> Future Enhancements

<table>
<tr>
<td width="50%" valign="top">

### 🚀 **Immediate Roadmap**

<br>

**🎮 PID Heading Control**
<br>
<img src="https://img.shields.io/badge/Priority-High-EF4444?style=flat-square&labelColor=1a1b27"/>
<br>
Replace bang-bang control with PID for smoother trajectory tracking and reduced oscillations.

<br><br>

**🗺️ Dynamic Re-routing**
<br>
<img src="https://img.shields.io/badge/Priority-High-EF4444?style=flat-square&labelColor=1a1b27"/>
<br>
Implement path replanning when trapped in local minima or facing unexpected obstacles.

<br><br>

**📊 Data Logging**
<br>
<img src="https://img.shields.io/badge/Priority-Medium-F59E0B?style=flat-square&labelColor=1a1b27"/>
<br>
Add SD card logging for position, velocity, sensor data, and motor commands for post-analysis.

</td>
<td width="50%" valign="top">

### 🌟 **Advanced Features**

<br>

**📡 Telemetry System**
<br>
<img src="https://img.shields.io/badge/Priority-Medium-F59E0B?style=flat-square&labelColor=1a1b27"/>
<br>
Bluetooth/WiFi telemetry for real-time monitoring and remote control capability.

<br><br>

**🗺️ Marine Map Integration**
<br>
<img src="https://img.shields.io/badge/Priority-Low-3B82F6?style=flat-square&labelColor=1a1b27"/>
<br>
Integrate Navionics or NOAA marine charts for depth awareness and coastal navigation.

<br><br>

**🤖 Machine Learning**
<br>
<img src="https://img.shields.io/badge/Priority-Research-6366F1?style=flat-square&labelColor=1a1b27"/>
<br>
Explore RL algorithms for optimal navigation policy learning in various conditions.

</td>
</tr>
</table>

<!-- Animated Divider -->
<img src="https://user-images.githubusercontent.com/73097560/115834477-dbab4500-a447-11eb-908a-139a6edaec5c.gif">

<br>

## <img src="https://media.giphy.com/media/hvRJCLFzcasrR4ia7z/giphy.gif" width="35"> Media Gallery

<div align="center">

### 📷
