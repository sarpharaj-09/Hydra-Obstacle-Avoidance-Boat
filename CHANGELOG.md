# 🧭 Changelog

All notable changes to the **Hydra Autonomous Boat Project** will be documented here.  
This project follows [Semantic Versioning](https://semver.org/) — **MAJOR.MINOR.PATCH**.

---

## [1.0.0] - 2025-10-16
### 🎉 Initial Release

This is the **first stable version** of the Hydra Autonomous Boat firmware, implemented using the **Arduino Nano** and **HC-SR04 ultrasonic sensors**.

### 🚀 Features
- Implemented **three ultrasonic sensors (front, left, right)** for obstacle detection using the `NewPing` library.
- Added **motor control logic** with PWM-based speed adjustment via `L293D` motor driver.
- Developed modular navigation logic:
  - `moveForward()`, `stopMotors()`, `turnLeft()`, `turnRight()`
  - Curve maneuvers: `turnLeftCurve()`, `turnRightCurve()`
  - Advanced avoidance: `reverseAndTurn()`, `strongerCurveTurn()`
- Integrated **distance measurement filtering** using `ping_median()` for more stable readings.
- Implemented **obstacle handling** that dynamically responds to front, left, and right distances.
- Introduced **Serial logging** with `logSensorData()` for monitoring real-time sensor feedback.
- Organized constants for fine-tuning (e.g., `CRITICAL_DISTANCE_FRONT`, `THRESHOLD_DISTANCE_SIDE`, and `MOTOR_SPEED_CURVE_LEFT/RIGHT`).
- Established the base **autonomous navigation loop** with conditional behavior for critical and non-critical obstacles.

### 🧩 Technical Details
- Platform: **Arduino Nano (ATmega328P)**
- Motor Driver: **L293D**
- Sensors: **HC-SR04 Ultrasonic (3 units)**
- Power Supply: 9V/12V battery
- Environment: Developed in **PlatformIO** (C++)

### 📚 Notes
- This is the **foundation code** for Hydra — future versions will focus on stability, PID tuning, and integration with GPS/compass systems.
- Serves as the base for hardware testing and performance validation.

---

## 🧩 Upcoming (Planned)
### 🔮 Future Additions
- Integrate **GPS and Compass modules** for semi-autonomous waypoint navigation.
- Implement **PID-based motor speed control** for smoother movement.
- Add **data logging** and **Bluetooth telemetry** for debugging and real-time monitoring.
- Introduce **object classification** using vision sensors (planned in v2.0.0).
