# 🦾 Smart Warehouse Bot
### AI-Driven Adaptive Sorting System with KUKA IIWA
**Course:** Smart Manufacturing (IN317)

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python)
![PyBullet](https://img.shields.io/badge/Sim-PyBullet-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Operational-green?style=for-the-badge)

---

## 📖 Overview
The **Smart Warehouse Bot** is a simulation of a next-generation logistics robot. Unlike traditional "blind" automation, this system uses simulated computer vision logic to **identify** packages and **adapt** its physical behavior in real-time.

### 🎯 The Goal
To demonstrate how intelligent robotics can:
1.  **Prioritize Safety:** Automatically slow down for fragile items.
2.  **Maximize Efficiency:** Speed up for standard durable goods.
3.  **Visualize Data:** Provide real-time telemetry to operators.

---

## 🚀 Key Features

### 🧠 1. Adaptive Speed Control ("The Brain")
The robot changes its motor control parameters based on the detected package type:
| Package Type | Color | Handling Mode | Speed |
| :--- | :--- | :--- | :--- |
| **Fragile** | 🔴 Red | **Precision** | 🐢 Slow & Smooth |
| **Standard** | 🔵 Blue | **Efficiency** | 🐇 Fast / High-Throughput |
| **Standard** | 🟢 Green | **Efficiency** | 🐇 Fast / High-Throughput |

### 📊 2. Dual Operator Dashboards
We replaced standard console logs with a GUI Control Center:
* **Telemetry Screen:** 3D trajectory plotting & Joint Angle analysis.
* **Live Scoreboard:** Gamified progress bar tracking sorted items.

### 🎲 3. Randomized Placement Strategy
To prevent unstable piles without complex sensors, the system utilizes a **stochastic placement algorithm**. It randomizes the drop location within a specific radius (`+/- 6cm`) of the bin center, ensuring even distribution and preventing stack collapse.

---

## 💻 How to Run
1.  **Prerequisites:**
    Make sure you have Python installed. Then install the dependencies:
    ```bash
    pip install -r requirements.txt
    ```

2.  **Launch Simulation:**
    ```bash
    python main.py
    ```

3.  **Controls:**
    * **Rotate View:** `Ctrl` + `Left Click`
    * **Zoom:** `Scroll Wheel`
    * **Pan:** `Ctrl` + `Middle Click`

---

## 📂 Project Structure
```text
📦 robot_arm-main
 ┣ 📜 main.py          # The Core Simulation Engine
 ┣ 📜 README.md        # Documentation
 ┣ 📜 requirements.txt # Python Dependencies
 ┗ 📜 .gitignore       # Git Configuration