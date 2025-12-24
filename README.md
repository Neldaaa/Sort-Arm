# Smart Warehouse Robotic Arm
### AI-Driven Adaptive Sorting System with KUKA IIWA

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python)
![PyBullet](https://img.shields.io/badge/Sim-PyBullet-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Operational-green?style=for-the-badge)

</div>

---

## 📖 Overview

<p align="justify">
The <strong>Smart Warehouse Bot</strong> is a simulation of a next-generation logistics robot. Unlike traditional "blind" automation, this system uses simulated computer vision logic to <strong>identify</strong> packages and <strong>adapt</strong> its physical behavior in real-time.
</p>

### 🎯 The Goal
<p align="justify">
To demonstrate how intelligent robotics can:
</p>

1.  **Prioritize Safety:** Automatically slow down for fragile items.
2.  **Maximize Efficiency:** Speed up for standard durable goods.
3.  **Visualize Data:** Provide real-time telemetry to operators.

---

## 🚀 Key Features

### 🧠 1. Adaptive Speed Control ("The Brain")
<p align="justify">
The robot changes its motor control parameters based on the detected package type. It balances speed and precision dynamically:
</p>

| Package Type | Color | Handling Mode | Speed |
| :--- | :--- | :--- | :--- |
| **Fragile** | 🔴 Red | **Precision** | 🐢 Slow & Smooth |
| **Standard** | 🔵 Blue | **Efficiency** | 🐇 Fast / High-Throughput |
| **Standard** | 🟢 Green | **Efficiency** | 🐇 Fast / High-Throughput |

### 📊 2. Dual Operator Dashboards
<p align="justify">
We replaced standard console logs with a GUI Control Center to enhance monitoring capabilities:
</p>

* **Telemetry Screen:** 3D trajectory plotting & Joint Angle analysis.
* **Live Scoreboard:** Gamified progress bar tracking sorted items.

### 🎲 3. Randomized Placement Strategy
<p align="justify">
To prevent unstable piles without using complex sensors, the system utilizes a <strong>stochastic placement algorithm</strong>. It randomizes the drop location within a specific radius (<code>+/- 6cm</code>) of the bin center, ensuring even distribution and preventing stack collapse.
</p>

---

## 💻 How to Run

### 1. Prerequisites
<p align="justify">
Make sure you have Python installed. Then install the dependencies:
</p>

```bash
pip install -r requirements.txt
```

### 2. Launch Simulation
```bash
python main.py
```

### 3. Controls:
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
