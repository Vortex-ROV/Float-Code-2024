# Vertical Profiling Float (VPF) Software Repository

Welcome to the repository for the Vertical Profiling Float software. This README will provide an overview of the software's file structure, and functionality.

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Repository Structure](#repository-structure)

## Introduction

The Vertical Profiling Float software is designed to control and monitor the float as it profiles the water column. It manages data acquisition from onboard sensors, controls vertical movement, and facilitates communication with surface units. This software ensures smooth profiling operations and precise data collection to perform competition challenges.

## Installation

To install and run the float software, follow these steps:

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/YourOrg/VPF-Code.git
   cd VPF-Code
   ```

2. **Flash the float code unto the ESP32**

3. **Flash the reciever code onto the reciever ESP32**

## Repository Structure

- **`/float`:** Main source code for controlling float operations, sensor data handling, and communication.
- **`/reciever`:** Main source code for controlling reviever ESP32 and serial transmission to the copilot's computer.
- **`/tests`:** Unit tests to ensure the software performs as expected under different conditions.

This structure helps to efficiently manage float operations and ensures smooth vertical profiling for data collection.
