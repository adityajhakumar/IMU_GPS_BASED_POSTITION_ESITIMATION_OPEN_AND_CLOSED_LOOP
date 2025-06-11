# Autonomous Surface Vehicle (ASV) - Closed-Loop Navigation using EKF + APF

This project implements a real-time, closed-loop navigation system for an Autonomous Surface Vehicle (ASV) using an *Extended Kalman Filter (EKF)* for sensor fusion and a *closed-loop Artificial Potential Field (APF)* algorithm for obstacle avoidance and goal-seeking behavior. Built on an *ESP32, it integrates **GPS (Quectel L89-S90), **BNO055 IMU, **ultrasonic sensors, and a **differential drive system (L298N)*.

---

## 🚀 Features

- 📡 *GPS (Quectel L89-S90)* for real-time location tracking  
- 🧭 *BNO055 IMU* for heading and acceleration  
- 🔄 *Extended Kalman Filter* for sensor fusion and drift correction  
- 🧲 *Closed-loop Artificial Potential Field (APF)* for dynamic obstacle avoidance  
- 🤖 *Differential drive control* for a 3-wheeled vehicle (2 rear motors, 1 passive front caster)  
- 📏 *Ultrasonic sensors* for real-time distance feedback  
- 🔁 Fully autonomous decision-making in real-world environments

---

## 🔌 Hardware Used

- ESP32 DevKit V1  
- Quectel L89-S90 GPS Module  
- Adafruit BNO055 IMU  
- HC-SR04 Ultrasonic Sensors (1-3 depending on setup)  
- L298N Motor Driver  
- 2 Rear DC Motors (with 1 passive front wheel)  
- 7.4V or 12V battery pack  

---

## 📍 Navigation Stack Overview

1. *GPS + IMU + EKF Fusion*  
   - Converts lat/lon into local XY meters  
   - Uses Kalman Filter to estimate velocity and position drift-free

2. *Artificial Potential Field (APF)*  
   - Computes attractive force to the goal  
   - Computes repulsive force from obstacles  
   - Real-time closed-loop adjustment of heading and velocity

3. *Motor Commands*  
   - Adjusts movement using left/right motor control  
   - Forward, Turn-Left, Turn-Right decisions made based on corrected heading

---

## 🛠 Installation

1. Flash the code on ESP32 via Arduino IDE or PlatformIO  
2. Connect GPS to RX2 (GPIO16) and TX2 (GPIO17)  
3. Connect BNO055 via I2C (SDA=21, SCL=22)  
4. Connect ultrasonic sensors and L298N motor pins as per your wiring  
5. Power the motors with a suitable power source

---

## 🧠 Future Enhancements

- PID-based heading control for smoother trajectory  
- Dynamic obstacle re-routing  
- Logging and telemetry with SD card / Bluetooth  
- Integration with marine-specific map overlays (Navionics/NOAA)

---

## 📷 Media (Coming Soon)

- Real-time demo video  
- Navigation path plot  
- Sensor debug snapshots

---

## 📄 License

 Apache License Version 2.0, January 2004. Feel free to fork, use, and improve!

---

## 🤝 Contributions

PRs and ideas are welcome. Drop a star ⭐ if you like it or open an issue for suggestions!
