# Autonomous Mobile Service Robot (ATmega2560)
**Role:** Lead Systems Engineer & Firmware Developer | **Platform:** ATmega2560 (C++)

![Final System](media/final_system_deployment.jpg)  
*(Phase 3: Final deployed prototype with custom enclosure for campus environments)*

## 1. Project Overview
This project executed the full **Systems Engineering V-Model** to design, build, and validate an autonomous mobile robot. The system bridges low-level embedded control (interrupts, PWM) with high-level path planning logic to execute navigation tasks while maintaining active safety protocols.

## 2. System Architecture
The robot utilizes a **Differential Drive** kinematics model controlled by an **ATmega2560**.
* **Actuation:** Dual DC Motors via L298N H-Bridge Driver
* **Feedback:** Magnetic Hall-Effect Encoders (Interrupt-driven odometry)
* **Perception:** Ultrasonic Sensor (HC-SR04) for obstacle detection
* **Power:** 12V DC Independent Power Supply (8xAA Array) for mobile autonomy

![Hardware Integration](media/hardware_integration_breadboard.jpg)  
*(Phase 1: Electrical subsystem integration and signal validation)*

## 3. Technical Implementation
### A. Interrupt-Driven Odometry
To ensure precise position tracking without blocking the main execution loop, I implemented hardware interrupts for the encoders.
* **Mechanism:** `attachInterrupt()` captures rising/falling edges on Pins 2 & 3.
* **Data Integrity:** Implemented `noInterrupts()` atomic blocks within the main control loop to prevent 16-bit data race conditions when reading volatile encoder variables.

### B. Feedback Control Loop
Implemented a **Proportional (P) Controller** to correct heading drift in real-time.
* **Error Calculation:** `error = right_ticks - left_ticks`
* **Correction:** Dynamically adjusts PWM duty cycle (`analogWrite`) to synchronize wheel velocities.
* *See `driveForward()` function in `main_controller.ino`*

### C. Safety Architecture (Subsumption)
The system features a strict hardware-level safety override that preempts navigation commands.
* **Obstacle-Dependent Hysteresis:** Implemented a blocking 'Stop-and-Wait' subsumption architecture. Upon detecting an obstacle (<20cm), the system enters a holding state, pausing the P-controller until the path clears to prevent oscillation or collision.

## 4. Systems Engineering Process (V-Model)
1.  **Requirements Analysis:** Defined "No-Collision" zones and navigation accuracy metrics (<5% drift).
2.  **Subsystem Design:** Selected the ATmega2560 for its high interrupt count (6 hardware interrupts) to handle dual encoders.
3.  **Integration:** Breadboard prototyping to validate H-Bridge logic levels.
4.  **System Validation:** Verified kinematic fidelity via a multi-stage waypoint test (Forward $\to$ 90° Turn $\to$ Half-Distance $\to$ 180° Pivot). Performed stress testing on inclined surfaces to validate torque sufficiency and center-of-gravity stability.

## 5. Media
[Link to Demo Video](https://github.com/user-attachments/assets/c3757b37-e308-423b-8d45-2012a37088b3)
