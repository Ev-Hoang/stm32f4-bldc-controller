# STM32F4 BLDC Motor Controller with PID Regulation

This is a 6-step BLDC motor controller implemented on the STM32F407 using Hall sensors and PWM. The controller supports closed-loop speed regulation with a PID algorithm and is designed for real-time embedded applications, such as robotics, drones, or electric propulsion systems.

---

## 🚀 Features

- ✅ 6-step commutation using 3 Hall sensors (sensor-based)
- ✅ Real-time PWM control with 6 output channels
- ✅ Circular buffer for commutation queue
- ✅ External interrupts for Hall sensor change detection (EXTI line)
- ✅ Full timer configuration using register-level code
- ✅ Modular and maintainable code structure
- ✅ Designed for commercial scalability and educational clarity
- ⏳ Coming soon: PID speed regulation and UART telemetry

---

## ⚙️ Hardware Requirements

- **MCU**: STM32F407VET6
- **Motor**: 3-phase BLDC motor with Hall sensors
- **Gate driver**: IR2136 / IR2101 or similar
- **Power stage**: MOSFET bridge
- **Other**:
  - UART to USB converter (for future telemetry/debug)
  - External power supply for motor
  - Optional OLED/UART display for RPM

---

## 🧠 System Overview

### 📌 Commutation Table (Hall → Step):

| Hall (ABC) | Step | High Side | Low Side |
|------------|------|-----------|----------|
| 001        | 0    | H3        | L2       |
| 011        | 1    | H2        | L1       |
| 010        | 2    | H3        | L1       |
| 110        | 3    | H1        | L3       |
| 100        | 4    | H1        | L2       |
| 101        | 5    | H2        | L3       |

### 🔁 Control Loop

1. Hall state changes → interrupt triggered (`EXTI9_5`)
2. Commutation step looked up and pushed to queue
3. `main()` loop pulls from buffer and runs `handleCommutation()`
4. PWM updated via CCRx values of TIM2/TIM3

---

## 📷 Media & Diagrams

> Coming soon:
- Circuit schematic
- Commutation timing diagram
- Oscilloscope waveform
- Demo video running motor with PID control

---
