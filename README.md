# Wi-Fi Microphone Project


This repository contains the complete project for a high-quality, ESP32-based wireless microphone. The system streams audio over Wi-Fi and features full digital control of analog parameters, status monitoring, and charge-and-play functionality.

## Features

*   **High-Quality Audio:** 24-bit, 48 kHz audio stream over Wi-Fi.
*   **Low-Noise Design:** A carefully designed power architecture isolates analog and digital circuits to minimize noise.
*   **Digital Control:** Analog parameters like compression and noise gate thresholds are adjusted digitally via I2C.
*   **User Interface:** An OLED display and a rotary encoder for real-time control and monitoring.
*   **Charge-and-Play:** The device can be used while charging from a USB power source.

## Project Structure


The repository is organized into two main parts: the ESP32 firmware and the Python client script to receive the audio on a computer.

```
.
├── esp32_firmware/    # Source code for the ESP32 microphone device (ESP-IDF project)
│   ├── main/          # Main application source files (C)
│   ├── components/    # ESP-IDF components (e.g., u8g2, u8g2_hal)
│   └── ...            # Build, config, and other ESP-IDF files
├── python_client/     # Python script to receive the audio stream on a PC
│   └── connection_gui_2.py
├── .gitignore         # Git ignore rules for both projects
└── README.md          # This file
```


## Getting Started

### Prerequisites

*   Espressif IDF (ESP-IDF) for building and flashing the firmware.
*   Python 3 for running the client script.

### 1. ESP32 Firmware

1.  Перейдите в директорию `esp32_firmware`.
2.  Используйте ESP-IDF для конфигурирования, сборки и прошивки проекта на вашу ESP32:
    ```bash
    cd esp32_firmware
    idf.py menuconfig
    idf.py build
    idf.py flash
    ```

### 2. Python Client

The Python script receives the audio stream from the microphone.

**Before running the client, make sure to connect your computer to the Wi-Fi access point created by the ESP32!**

1.  Connect to the Wi-Fi network broadcast by your ESP32 device (the SSID and password are set in the firmware configuration).
2.  Navigate to the `python_client` directory.
3.  Run the script:
    ```bash
    cd python_client
    python connection_gui_2.py
    ```

---

## Hardware & Firmware Architecture Details (`esp32_firmware`)

This section describes the technical implementation of the microphone device.

### Power System

The power architecture is designed for maximum isolation between analog and digital circuits to minimize noise.

*   **Power Source**: A 2S Li-Ion battery managed by a BMS/charging module based on the IP2326 chip, which ensures safe charging and supports "Charge-and-Play" mode.
*   **Primary Filtering**: An input π-filter (CLC) is installed to suppress high-frequency interference from the charging module's switching converter.
*   **Power Distribution**: The power bus is split into four independent LDO regulators, creating isolated power domains:
    *   **LDO1 (5.0V) → V_ANALOG_5V**: "Clean" analog power for the SSM2167, MCP6002, and the analog section of the PCM1808 ADC (VCC).
    *   **LDO2 (3.3V) → V_DIGITAL_3V3_ESP**: The main "noisy" digital power supply for the ESP32, CP2102, and OLED display.
    *   **LDO3 (MCP1700-3.3, low-noise) → V_DIGITAL_AUDIO_3V3**: Isolated "clean" digital power for the digital section of the PCM1808 ADC (VDD) and the final volume control (MCP4531).
    *   **LDO4 (5.0V) → V_DIGITAL_5V**: Isolated power for 5V I2C devices and the high-voltage side of the level shifter.
*   **Grounding**: A common ground plane concept is implemented without separating AGND & DGND due to the use of a two-layer PCB.

### Analog Signal Path

*   **Source**: Monacor MP-110 dynamic microphone capsule.
*   **Preamplifier/Compressor**: The SSM2167 provides preamplification, compression, and a noise gate.
    *   **Noise Gate**: Controlled by an MCP4531 digital potentiometer (10kΩ) at I2C address `0x2F`.
    *   **Compression**: Controlled by a DS1803 digital potentiometer (100kΩ) at I2C address `0x28`.
*   **Additional Gain**: An MCP6002 operational amplifier provides an additional +8 dB of gain.
*   **Volume Control**: An MCP4531 (10kΩ) at I2C address `0x2E` adjusts the final analog signal level.
*   **ADC**: The PCM1808 digitizes the signal into a 24-bit, 48 kHz I2S stream.

### Digital Section and Logic

*   **Microcontroller (ESP32)**: The "brain" of the system.
    *   Manages all I2C devices using the thread-safe Master Bus API (`driver/i2c_master.h`).
    *   Processes user input from a quadrature encoder.
    *   Displays information on an SSD1306 OLED screen using the U8g2 library.
    *   Reads the I2S stream from the PCM1808 and sends it over Wi-Fi.
*   **Software Architecture**: The firmware is divided into 4 logical modules (`hardware_control.c`, `app_tasks.c`, `network_stream.c`, `main.c`) with a central configuration file (`project_config.h`) for maximum readability and maintainability.
