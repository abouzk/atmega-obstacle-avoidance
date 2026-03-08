# Autonomous Mobile Service Robot (ATmega2560)
**Role:** Lead Systems Engineer & Firmware Developer | **Platform:** ATmega2560 (C++)

![Final System](media/final_system_deployment.jpg)  
*(Phase 3: Final deployed prototype with custom enclosure for campus environments)*

## 1. Project Overview
This project executed the full **Systems Engineering V-Model** to design, build, and validate an autonomous mobile robot. The system bridges low-level embedded actuation with an overarching, asynchronous safety architecture designed to prevent collisions in dynamic environments.


## 2. System Architecture & Sensor Fusion
The robot utilizes a **Differential Drive** kinematics model, prioritizing safe navigation via sensor fusion and event-driven state changes.
* **Actuation:** Dual DC Motors via L298N H-Bridge Driver
* **Feedback:** Magnetic Hall-Effect Encoders (Interrupt-driven odometry)
* **Perception:** Ultrasonic Sensor (HC-SR04) for obstacle detection
* **Power:** 12V DC Independent Power Supply (8xAA Array) for mobile autonomy

![Hardware Integration](media/hardware_integration_breadboard.jpg)  
*(Phase 1: Electrical subsystem integration and signal validation)*

## 3. Technical Implementation
### A. Asynchronous Odometry & Data Integrity
To ensure precise position tracking without blocking the main execution loop, I implemented hardware interrupts for the encoders.
* **Mechanism:** `attachInterrupt()` captures rising/falling edges on Pins 2 & 3, ensuring the path-planning logic never misses a spatial update.
* **Concurrency Safety:** Implemented `noInterrupts()` atomic blocks within the main control loop to prevent 16-bit data race conditions when reading volatile encoder variables on an 8-bit architecture.

### B. Heading Correction Control Loop
Implemented a **Proportional (P) Controller** to correct heading drift in real-time.
* **Error Calculation:** Dynamically calculates tracking error (`error = right_ticks - left_ticks`).
* **Correction:** Synchronizes the dual PWM H-bridge outputs (`analogWrite`) to maintain a zero-degree deviation vector.

### C. Active Safety Architecture (Subsumption)
Inspired by ISO 13482 safety standards for personal care robots, the navigation stack is subordinate to a primary safety override loop.
* **Preemptive Collision Avoidance:** A dedicated `checkObstacle()` state function continuously polls the ultrasonic telemetry. If the time-of-flight data indicates an object within the 20cm hazard zone, the system immediately drops PWM signals to 0 and enters a hardware blocking state.
* **Hysteresis & Recovery:** The system utilizes a 'Stop-and-Wait' methodology to prevent oscillation. It remains in a halted state until the path clears, at which point it dynamically recalculates its remaining waypoint distance and resumes the mission.

## 4. Systems Engineering Process (V-Model)
1.  **Requirements Analysis:** Translated Voice of Customer (VOC) into actionable "No-Collision" safety metrics and navigation accuracy tolerances (<5% drift).
2.  **Subsystem Design:** Selected the ATmega2560 for its high interrupt count (6 hardware interrupts) to seamlessly handle asynchronous dual-encoder telemetry.
3.  **Integration:** Breadboard prototyping to validate H-Bridge logic levels and sensor fusion state machines.
4.  **System Validation:** Verified kinematic fidelity via a multi-stage waypoint test (Forward -> 90° Turn -> Half-Distance -> 180° Pivot). Performed stress testing on inclined surfaces to validate torque sufficiency and center-of-gravity stability.

## 5. Media
[Link to Demo Video](https://github.com/user-attachments/assets/c3757b37-e308-423b-8d45-2012a37088b3)
