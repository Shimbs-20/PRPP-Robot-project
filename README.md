# 4-Axis PRPP Robotic Arm with Computer Vision 🤖

This project implements a custom 4-DOF (Degrees of Freedom) robotic manipulator using a PRPP (Prismatic-Revolute-Prismatic-Prismatic) configuration. It features autonomous object sorting via Computer Vision and a custom Inverse Kinematics solver.

## 🚀 Features
* **Inverse Kinematics:** Custom Python solver to calculate joint angles for (x, y, z) coordinates.
* **Computer Vision:** OpenCV integration for real-time object detection and color sorting.
* **High Precision:** ESP32-based stepper motor control for sub-millimeter accuracy.
* **Wireless Control:** WiFi-based command interface between Python script and Robot firmware.

## 🛠️ Hardware Stack
* **Microcontroller:** ESP32 (Dual Core)
* **Actuators:** 4x NEMA 17 Stepper Motors
* **Drivers:** A4988 / TMC2208 Stepper Drivers
* **Power:** 12V 10A PSU

## 💻 Software & Libraries
* **Python 3.8+**
    * `opencv-python` (Vision)
    * `numpy` (Matrix Math/Kinematics)
    * `pyserial` (Communication)
* **C++ (Arduino IDE)**
    * `AccelStepper` library for smooth motion profiles.

## 📐 Kinematics
The robot uses a PRPP configuration. The Inverse Kinematics equation was derived using the Denavit-Hartenberg (DH) parameters:
> [You can paste a screenshot of your math formula here if you have one!]

## 🔧 How to Run
1.  Upload the firmware in `/firmware` to the ESP32.
2.  Install Python dependencies: `pip install opencv-python numpy`
3.  Run the controller: `python main.py`

## 👨‍💻 Author
**Youssef S.** - Mechatronics Engineer
