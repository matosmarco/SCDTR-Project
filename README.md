# Real-Time Cooperative Control of a Distributed Illumination System

[![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![Raspberry Pi Pico](https://img.shields.io/badge/Raspberry%20Pi%20Pico-A22846?style=flat-square&logo=Raspberry%20Pi&logoColor=white)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=flat-square&logo=arduino&logoColor=white)](https://www.arduino.cc/en/software)

This repository contains the software developed for the **Distributed Real-Time Control Systems (SCDTR)** course, part of the Master's degree in Electrical and Computer Engineering at Instituto Superior Técnico.

## Project Overview

The main objective of this project is to design and implement a networked, real-time controller for a small-scale office illumination system. The system controls the dimming levels of individual smart luminaires situated above office desks to maximize user comfort (maintaining visibility bounds and preventing flicker) while simultaneously minimizing the global energy consumption.

The system operates without a central master node, relying on distributed cooperative optimization algorithms where nodes exchange messages via a CAN-BUS network to reach a consensus on the optimal lighting states.

## Hardware Architecture

The physical model simulates an office space using a controlled physical environment. Each smart luminaire node consists of:
*   **Microcontroller:** Raspberry Pi Pico (RP2040)
*   **Actuator:** Light-Emitting Diode (LED) driven by a high-frequency PWM signal
*   **Sensor:** Light-Dependent Resistor (LDR) in a voltage divider circuit to measure reflected illuminance (LUX)
*   **Network Interface:** Joy-It MCP2515 CAN-BUS drivers for asynchronous inter-node communication

## Repository Structure

The project was developed in two main stages, separated into their respective directories:

### `fase1/` - Stage 1: Individual Local Control
The first phase establishes the fundamental hardware infrastructure and local feedback control for a single luminaire operating independently.
*   **Luxmeter & LED Driver:** Conversion of analog LDR voltage readings into LUX units using logarithmic calibration, and an actuation system using digital PWM.
*   **PID Controller:** Implementation of a discrete C++ PID controller equipped with anti-windup mechanisms and set-point weighting to ensure fast responses to user commands while smoothly rejecting external light disturbances.
*   **PC Interface:** A serial command-line interface to read/write system data, configure reference bounds, and calculate real-time performance metrics (Energy consumption in Joules, Visibility error, and Flicker error).

### `fase2/` - Stage 2: Distributed Cooperative Control
The second phase scales the system into a distributed control network comprising multiple luminaires operating cooperatively.
*   **Real-Time OS (FreeRTOS):** Utilized FreeRTOS to manage concurrent tasks, ensuring strict real-time deadlines and non-blocking execution of the control loop, communication protocols, and user interactions.
*   **CAN-BUS Communications:** Implementation of an asynchronous message exchange protocol between the nodes over the CAN-BUS network.
*   **PC Hub Function:** Configuration of one luminaire to act as a hub, concurrently routing commands and telemetry between the serial PC connection and the CAN network without blocking real-time control tasks.
*   **System Calibration & Start-up:** Boot sequence mechanisms for nodes to dynamically discover each other on the network and calibration routines to model the light cross-coupling effects between neighboring desks.
*   **Distributed Control Algorithms:** Decentralized optimization controllers where each independent node decides its own actuation to jointly minimize the global energy cost function while satisfying minimum illumination constraints for occupied and unoccupied states.

## Getting Started

1. **Prerequisites:** 
   * Install the [Arduino IDE](https://www.arduino.cc/en/software).
   * Install the Raspberry Pi Pico board manager and the required CAN-BUS libraries.
2. **Compilation:**
   * Open the `.ino` or C++ files located in `fase1/` or `fase2/`.
   * Compile and upload to the Raspberry Pi Pico via USB. For the initial upload, ensure the `BOOTSEL` button is held during connection.
3. **Usage:**
   * Open the Arduino Serial Monitor (115200 baud).
   * Use the implemented command-line interface (e.g., `o 1 h` to set desk 1 occupancy to HIGH, `r 1 100` to set reference to 100 LUX) to interact with the system.

---
