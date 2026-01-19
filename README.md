# 🌱 ESP32-S3 Environmental & Acoustic Monitor

## Overview

This project was developed as part of an **Embedded Systems course** and implements a **low-power, multi-sensor environmental and sound monitoring system** based on the ESP32-S3.

The device collects air-quality, climate, and acoustic data and streams it wirelessly via **Bluetooth Low Energy (BLE)** to a companion **React Native mobile application** for real-time visualization.

The firmware uses **FreeRTOS multitasking**, real-time signal processing for sound pressure level (SPL), and automatic **low-power idle mode** when no BLE client is connected.

---

## Project Abstract

This project focuses on the design and implementation of a low-power, multi-sensor environmental and acoustic monitoring platform based on the ESP32-S3 microcontroller.

The system integrates multiple digital sensors for air quality and climate monitoring, including CO₂, TVOC, eCO₂, temperature, humidity, pressure, and altitude, in addition to an I2S digital microphone for real-time sound acquisition. Each sensing module is implemented as an independent FreeRTOS task, enabling concurrent sampling, processing, and communication.

Environmental and acoustic data are packaged into binary frames and transmitted wirelessly over Bluetooth Low Energy (BLE) to a companion mobile application developed in React Native, which provides real-time visualization and monitoring.

The firmware also implements dynamic power management, automatically entering a low-power mode when no BLE client is connected by suspending sensor tasks, reducing CPU frequency, and stopping periodic measurements. Normal operation is restored immediately upon reconnection.

This project demonstrates practical application of embedded multitasking, digital communication protocols (I2C, I2S, BLE), real-time signal processing, and power-aware firmware design in a complete end-to-end IoT sensing system.

---

## Features

- 📡 BLE real-time data streaming  
- 📱 React Native mobile dashboard  
- 🌡 Temperature, humidity, pressure, altitude  
- 🧪 Air quality sensing (TVOC, eCO₂, IAQ, CO₂)  
- 🎤 I2S microphone audio sampling  
- 🔊 Real-time RMS & SPL calculation  
- ⚡ Automatic low-power mode when idle  
- 🧵 Independent FreeRTOS tasks per sensor  

---

## Hardware

- **ESP32-S3**
- **SCD40** – CO₂, temperature, humidity  
- **ENS160** – TVOC, eCO₂, IAQ  
- **BME680** – pressure & altitude  
- **I2S MEMS microphone**
- Status LED

---

## System Architecture

Each sensor runs in its own FreeRTOS task:

| Task        | Function |
|-------------|----------|
| BME680 Task | Pressure & altitude |
| ENS160 Task | TVOC, eCO₂, IAQ |
| SCD40 Task  | CO₂, temperature, humidity |
| SPL Task    | Audio sampling, RMS & SPL calculation |

Data is packed into binary frames and sent over BLE notifications.

A low-power mode is entered automatically after inactivity.

---

## BLE Protocol

Each BLE packet is structured as:

```
[1 byte Sensor ID][Binary sensor struct]
```

### Sensor IDs

| Sensor | ID |
|--------|----|
| BME680 | 0x01 |
| ENS160 | 0x02 |
| SCD40  | 0x03 |
| SPL    | 0x04 |

Each payload includes a timestamp (ms since boot).

---

## Sound Processing

- 22,050 Hz sampling rate  
- DC offset removal  
- RMS calculation over 1 second  
- SPL estimation using calibrated reference  

---

## Low-Power Behavior

- Enters low-power mode after 10s without BLE connection  
- Suspends all sensor tasks  
- Reduces CPU frequency  
- Stops SCD40 periodic measurements  
- Automatically resumes when a client reconnects  

---

## Build & Flash

### Requirements

- ESP32 board support (Arduino IDE or PlatformIO)
- Libraries:
  - SparkFun SCD4x
  - Adafruit BME680
  - ScioSense ENS160
  - ESP32 BLE Arduino

### Flash

- Board: `ESP32S3 Dev Module`  
- Upload speed: `921600`  

---

## Example Output

```
SCD40 | Temp: 24.3 °C | Hum: 41 % | CO2: 612 ppm
ENS160 | TVOC: 112 ppb | eCO2: 530 ppm | IAQ: 2
BME680 | Pressure: 1012.4 hPa | Altitude: 8.3 m
SPL | RMS: 0.00432 | SPL: 37.8 dB
```

---

## Author

Abdulrahman Almajdalawi  
MSc Applied Computer Science – Embedded Systems  

