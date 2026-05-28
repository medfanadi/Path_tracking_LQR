# Path_tracking_LQR

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Python](https://img.shields.io/badge/python-3.8%2B-blue)

A robust path-tracking control framework designed specifically for **double-steering off-road mobile robots**. This repository implements a Linear Quadratic Regulator (LQR) control law integrated with a continuous-time Kalman-Bucy Filter (Observer) to handle state estimation and slip disturbances common in off-road environments.

---

## 📌 Features

* **Double-Steering Kinematics:** Supports independent front and rear steering control for enhanced maneuverability.
* **LQR Control Law:** Optimal control approach for precise trajectory tracking and minimized lateral error.
* **Kalman-Bucy Observer:** Continuous-time state estimation to filter sensor noise and account for off-road wheel slip/disturbances.
* **Off-Road Optimization:** Designed to handle non-holonomic constraints and low-traction terrains.

---

## 🛠️ Architecture Overview

The system operates in a closed loop where the Kalman-Bucy Observer estimates the robot's true states (including lateral deviation and heading error) from noisy sensor data. The LQR controller then computes the optimal front and rear steering angles to minimize tracking errors.
