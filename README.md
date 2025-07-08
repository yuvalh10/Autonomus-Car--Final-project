# Autonomous Vehicle Project 🚗🤖

An **autonomous vehicle system** using **Arduino Mega 2560** and **Raspberry Pi 5** with **GPS navigation**, **YOLO-based human detection**, **real-time obstacle avoidance**, and **dual manual/autonomous operation modes**.

## 🚀 Features

### Dual Operation Modes
- **Manual Control**: Bluetooth-based remote control via mobile app
- **Autonomous Mode**: GPS-guided navigation with intelligent pathfinding

### Advanced Safety Systems
- **Human Detection**: **YOLO-based** person detection with immediate emergency stop
- **Multi-Sensor Obstacle Avoidance**: **LiDAR Lite V3** + dual **HC-SR04** ultrasonic sensors
- **Smart Maneuvering**: Three-mode obstacle avoidance algorithm
- **Visual Indicators**: LED warning system and directional signals

### Navigation & Control
- **GPS Navigation**: Accurate positioning with **15-meter target precision**
- **Compass Integration**: **MPU-9250/6500** magnetometer for heading correction
- **Hybrid Navigation**: Backup inertial navigation when GPS signal is lost
- **Real-time Processing**: Interrupt-driven sensor management

## 🛠 Hardware Components

### Main Controllers
- **Arduino Mega 2560**: Primary control unit for motors, sensors, and navigation
- **Raspberry Pi 5**: Computer vision processing and human detection

### Sensors & Modules
- **LiDAR Lite V3**: High-precision distance measurement (up to 40m)
- **HC-SR04 Ultrasonic Sensors**: Side obstacle detection
- **Pi Camera V2**: Real-time video processing for human detection
- **Neo-6M GPS Module**: Global positioning
- **MPU-9250/6500**: 9-axis IMU with magnetometer
- **HC-06 Bluetooth**: Wireless communication

### Actuators & Power
- **4x DC Motors**: Independent wheel control via **L293D motor shield**
- **LED Indicators**: Direction signals and warning lights
- **Buzzer**: Audio alerts for destination arrival
- **Portable Charger**: 5V/2.1A power supply for Raspberry Pi

## 💻 Software Architecture

### Modular Design
├── Arduino Code

│   ├── Autonomous_Car.ino          # Main program loop

│   ├── CarMovement.cpp/h           # Motor control and navigation

│   ├── CarState.cpp/h              # State management and communication

│   ├── DistanceSensors.cpp/h       # Sensor processing with interrupts

│   └── Config.h                    # System configuration

└── Raspberry Pi Code

└── YOLO.py                     # Human detection and emergency stop

### Key Algorithms
- **Haversine Formula**: Accurate GPS distance calculations
- **YOLO Object Detection**: Real-time human identification
- **State Machine**: Seamless mode transitions
- **Interrupt-Driven Processing**: Non-blocking sensor operations

## 🎯 Capabilities

- **Autonomous Navigation**: Navigate to predefined GPS coordinates
- **Dynamic Obstacle Avoidance**: Smart maneuvering around static and dynamic obstacles  
- **Emergency Stop System**: Immediate halt when humans are detected
- **Smooth Mode Switching**: Seamless transition between manual and autonomous control
- **Real-time Monitoring**: Continuous sensor feedback and system status

## 📱 Mobile App Control

Custom **Bluetooth application** featuring:
- **Directional controls** (forward, backward, left, right)
- **Horn activation**
- **Mode switching** (manual ↔ autonomous)
- **Destination selection** (4 predefined locations)
- **Real-time connection status**

## 🎓 Academic Project

This project was developed as a **final year engineering project** at **Braude College of Engineering, Karmiel** for the **Department of Electrical and Electronics Engineering**. It demonstrates practical implementation of:

- **Embedded systems programming**
- **Computer vision and machine learning**
- **GPS navigation and sensor fusion**
- **Real-time control systems**
- **Modular software architecture**

## 🔧 Technical Specifications

- **Navigation Accuracy**: ±15 meters GPS precision
- **Obstacle Detection Range**: 2cm - 4m (ultrasonic), up to 40m (LiDAR)
- **Processing**: Real-time computer vision at **30fps**
- **Communication**: Bluetooth range up to **10 meters**
- **Power**: Dual power system (7.4V motors, 5V electronics)

## 🤝 Contributors

- **Yuval Hamar** - Lead Developer & Hardware Integration
- **Ido Ben Harush** - Computer Vision & Algorithm Development

---

*Built with ❤️ for autonomous robotics and embedded systems*

**Tags**: `autonomous-vehicle` `arduino` `raspberry-pi` `gps-navigation` `computer-vision` `yolo` `obstacle-avoidance` `robotics` `embedded-systems` `iot`
