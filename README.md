# YOLO/Stereo Vision Indoor Navigating Car

## Project Overview

**ECE 490/491 - Group 26 | University of Alberta**

This project implements a real-time indoor obstacle detection and 3D localization system for a small motorized vehicle. It is specifically optimized for challenging **low-light** and **poorly textured** environments (e.g., featureless white walls) where traditional vision algorithms typically fail.

## System Architecture: The "Brain-Stem" Model

The software follows a bifurcated architecture to ensure both high-level intelligence and low-level safety:

- **The Brain (NVIDIA Jetson Nano):** Handles high-level perception, including YOLOv8 object detection, synchronized RGB-D data processing, and complex decision-making logic.

- **The Stem (ESP32):** Manages real-time motor control, executes timing-critical reflexes, and handles hardware failsafes through ultrasonic sensors.

## Hardware Design

### Chassis

The vehicle chassis is designed to provide a stable platform for the stereo vision system and computing hardware. The chassis accommodates the Intel RealSense D435 camera at an optimal height for indoor navigation, ensuring clear line-of-sight for obstacle detection.

![Chassis View 1](hardware/chassis/chassis1.jpeg)
*Front view of the vehicle chassis showing camera mounting and overall structure*

![Chassis View 2](hardware/chassis/chassis2.jpeg)
*Side view of the chassis highlighting the compact design and component placement*

### PCB Design

The custom PCB integrates the ESP32 microcontroller with motor drivers, ultrasonic sensor interfaces, and communication modules. The board is designed to handle real-time motor control while maintaining reliable communication with the Jetson Nano via UART.

![PCB Design 1](hardware/pcb/pcb_design1.png)
*Top view of the PCB layout showing component placement and routing*

![PCB Design 2](hardware/pcb/pcb_design2.png)
*Bottom view of the PCB showing ground planes and power distribution*

![PCB Schematic](hardware/pcb/Schematic.jpg)
*Complete schematic diagram of the PCB showing all electrical connections, including ESP32, motor drivers, ultrasonic sensor interfaces, and communication modules*

## Key Technical Features

### Multiprocessing Pipeline
Three parallel processes (Camera I/O, Perception Engine, and UART Communication) minimize latency and prevent camera fetch rates from being bottlenecked.

### Hysteresis-Based Fusion
Logic switches between "Semantic Mode" (YOLOv8) and "Geometric Mode" (Stereo Depth) based on ambient light levels to prevent "mode flickering".

### Active Stereo Sensing
Leverages the Intel RealSense D435's internal IR projector to create artificial texture on featureless surfaces, achieving a depth map fill rate of >99% on white walls.

### Safety Redundancy
Hardware-level ultrasonic interrupts (front and side) override the Jetson Nano and stop the vehicle if an obstacle is detected within 0.25m.

## Performance & Safety Requirements

- **Perception Inference:** ≤40 ms (~15 FPS) utilizing TensorRT optimization
- **Reaction Latency:** End-to-end reaction time under 200ms
- **Heartbeat Fail-safe:** Automatic motor shutdown if communication from the Jetson is lost for more than 500ms
- **Watchdog Timer:** 1-second hardware watchdog to reset the ESP32 in case of firmware hangs
