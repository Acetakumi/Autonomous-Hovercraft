Autonomous Hovercraft Navigation System (Team Project)
Project Overview
This project focuses on the design and implementation of an autonomous hovercraft capable of navigating a maze and stopping accurately under a target bar. The system integrates embedded hardware, real-time sensor processing, and control logic written in C on a microcontroller platform.

Key Objectives
Autonomous maze navigation
Obstacle detection and avoidance
Stable hover and directional control
Precise stopping at a defined end condition
Reliable real-time performance on constrained hardware
System Architecture
The hovercraft uses a modular embedded architecture consisting of a microcontroller handling real-time control logic, multiple sensors for environment awareness, and independent lift, thrust, and steering mechanisms. Sensor data is processed continuously to make navigation decisions and adjust actuation.

Hardware Components
Microcontroller: ATmega328P
Sensors:
Ultrasonic sensors (side obstacle detection)
Infrared distance sensor (front detection)
IMU (MPU6050) for orientation and motion tracking
Actuators:
Lift fan
Thrust fan
Servo motor (steering)
Power: Dual battery configuration
Software & Tools
Language: C (embedded)
Development Environment: PlatformIO, VS Code
Version Control: Git, GitHub
Simulation & Design: Tinkercad
My Contributions
Developed embedded C control logic for autonomous navigation
Integrated and validated ultrasonic, IR, and IMU sensors
Implemented real-time obstacle avoidance and steering decisions
Debugged timing, sensor noise, and reliability issues
Collaborated using Git/GitHub for version control
Repository Structure
Hovercraft-Team19/ ├── src/ ├── include/ ├── lib/ ├── test/ ├── platformio.ini └── README.md

Skills Demonstrated
Embedded systems programming
Real-time control systems
Sensor integration and data processing
Hardware–software integration
Team-based engineering development
Project Status
Final version completed and validated.
