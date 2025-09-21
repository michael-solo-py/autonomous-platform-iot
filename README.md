# autonomous-platform-iot
IoT-based autonomous mobile platform for environmental monitoring. Combines ESP32 sensors, Raspberry Pi, GPS, and cloud server integration for data collection, mission control, and telemetry logging. Developed as part of a diploma project.
Integrated Mobile Platform Control System with Cloud Technologies

This repository contains the source code and documentation for my diploma project:  
An autonomous mobile platform for environmental monitoring with integration of cloud technologies.

Project Overview
The system combines ESP32 microcontrollers, a Raspberry Pi 4, and a laboratory control server.  
It is capable of collecting environmental data, navigating via GPS, executing missions, and transmitting results to a laboratory or cloud system.
- Object of research: processes of autonomous collection and transmission of environmental data.  
- Goal: develop an integrated control system for a mobile platform to perform monitoring tasks.
- Methods: system analysis, embedded programming, testing, and experimental validation.

 Features
- Environmental monitoring with BME680 (temperature, humidity, pressure, VOC)  
- GPS navigation with automatic home return and mission execution  
- Mission control modes:  
  - Home mode (standby)  
  - Mission mode (route execution)  
  - Return home (safety function)  
- Data logging to SD card and transfer via UART to Raspberry Pi  
- Laboratory server for mission management, heartbeat monitoring, and file transfers  
- Cloud-ready architecture for remote access and IoT integration  

System Architecture
- Hardware:  
  - Raspberry Pi 4 (4GB RAM)  
  - ESP32 (sensor interface & buffering)  
  - BME680 sensor, GPS GT-U7  module, SD card, motor driver, LiPo battery 12V/5Ah  
- Software:  
  - ESP32 firmware (C++ / MicroPython)  
  - Raspberry Pi platform controller (Python)  
  - Laboratory server (Python)  
  - Protocols: JSON, TCP/IP, UART, NMEA GPS

Repository Structure
- `/firmware` – ESP32 code (`esp_modules.cpp`, `Esp32 Rpi Uart.py`).  
- `/platform` – Raspberry Pi control scripts (`autonomous_platform.py`).  
- `/server` – Laboratory server (`lab_server_complete.py`).
- `/docs` – Project documentation, presentations, and thesis excerpts.
- `/attachments` – Images and other.

How to Run
1. Flash ESP32 with `/firmware` code.  
2. Run `autonomous_platform.py` on Raspberry Pi.  
3. Start `lab_server_complete.py` on your lab server machine.  
4. Connect hardware as shown in `/docs/hardware_schematics`.  
5. Issue commands and monitor telemetry through the lab server.  

Results
- Navigation accuracy: ±2 m  
- Uptime: 99% during tests  
- Autonomy: 25 min continuous work  
-150 records of environmental data collected per mission

Practical Applications
- Agricultural monitoring  
- Environmental safety control  
- Smart city IoT infrastructure  
- Research and educational projects  
