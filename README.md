# Inertial Gated Haptic Alert System

> Patent Published | ESP32 | Signal Processing | Assistive Technology

## Overview

This project is a wearable assistive device designed to provide context-aware haptic alerts for Deaf and Hard-of-Hearing (DHH) users. Unlike conventional sound alert wearables that vibrate whenever loud sounds are detected, this system intelligently suppresses false alerts caused by the wearer's own actions by combining real-time audio analysis with inertial motion sensing.

The system is implemented on an ESP32 and performs real-time FFT-based audio processing, IMU-based motion analysis, and cross-spectral correlation to determine whether a detected sound originates from the environment or from the user's own movement.

---

## Key Features

- Real-time FFT audio analysis
- IMU-based motion classification
- Cross-domain spectral correlation
- Intelligent false-alert suppression
- Context-aware haptic feedback
- ESP32 Wi-Fi dashboard for parameter tuning
- Runtime calibration and alert logging

---

## Hardware

- ESP32
- INMP441 Digital MEMS Microphone
- MPU6050 6-Axis IMU
- Vibration Motor
- Li-Po Battery

---

## Technologies Used

- Arduino Framework
- C++
- ESP32
- I2S
- I2C
- ArduinoFFT
- Wi-Fi Access Point
- HTTP Server

---

## Repository Structure

```
firmware/
docs/
images/
```

---

## Patent

This repository accompanies work that resulted in a published patent.

**Title:**

*Method and System for Predictive Haptic Alert Gating in a Wearable using Cross Domain FFT Correlation and Graded Motion Reduction*

---

## License

This project is shared for educational and research purposes.
