# Lynbrook Red Robot Tour 2024

This repository contains the source code for Lynbrook Red's Robot Tour device at Science Olympiad
2024.

## File structure

- `main.py`: main entrypoint for ESP32 with micropython
- `app.py`: application logic
- `drivers/`: relevant device drivers
  - `mpuXXXX.py`: MPU motion sensor
  - `ak8963.py`: magnetic sensor
  - `ssd1306.py`: OLED driver
  - `vl53l0x.py`: LIDAR driver
