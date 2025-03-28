# ESP32 Plant Monitoring Scale

## Project Description

This project implements a low-power plant monitoring system using an ESP32 microcontroller. It collects data such as plant weight and environmental parameters (temperature, humidity, air pressure) via HX711 and BME280 sensors. The ESP32 periodically wakes from deep sleep, performs measurements, sends data to ThingSpeak, and goes back to sleep. Power efficiency is prioritized for long-term, battery-powered operation.

The core functionality is the **detection of weight stagnation**, which indicates that the plant is no longer absorbing water – a sign that watering may be necessary.

---

## Features

- ESP32 with deep sleep and OTA update support
- Weight measurement with HX711 + load cell
- Environmental sensing with BME280
- EEPROM-persistent calibration and tare
- Detection of weight stagnation to estimate plant dryness
- Data upload to ThingSpeak
- First-boot setup via Serial Monitor

---

## Stagnation Detection

The system uses a custom algorithm to determine when watering is needed based on weight stagnation:

- Each time the device wakes up, the current weight is added to a **rolling buffer** of the last 7 measurements (`HISTORY_SIZE`).
- After an initial phase of 3 measurements (`IGNORE_FIRST_READINGS`), the system evaluates how often the weight has **decreased** between successive readings.
- A drop is only considered valid if the weight difference exceeds **1.3 grams** (`VALID_DROP_THRESHOLD`).
- If **fewer than 3 valid drops** (`MIN_VALID_DROPS`) are found in the history, it assumes that the weight has stagnated — and thus `wateringNeeded = true`.
- If sufficient drops are found, `wateringNeeded` is reset to `false`.

This approach helps to avoid false positives due to measurement noise or temporary load changes.

---

## Hardware Setup

| Component        | Description                             |
|------------------|-----------------------------------------|
| ESP32            | Preferably with 18650 battery support   |
| HX711            | Load cell amplifier                     |
| Load Cell        | 5 kg recommended                        |
| BME280           | I²C sensor for temperature/humidity     |
| 18650 Battery    | Power supply                            |

GPIO configuration:

| Function     | Pin        |
|--------------|------------|
| HX711 DOUT   | GPIO 32    |
| HX711 SCK    | GPIO 33    |
| I²C SDA      | GPIO 26    |
| I²C SCL      | GPIO 27    |
| Battery Read | GPIO 34    |

---

## Setup Instructions

1. **Clone the Repository**:
```bash
git clone <your-repo-url>
```

2. **Configure Credentials**:
   - Copy `cred_example.cpp` to `cred.cpp`
   - Fill in your Wi-Fi and ThingSpeak credentials
   - Place it in the `src` folder

3. **Flash to ESP32** using Arduino IDE or PlatformIO

4. **Initial Setup (First Boot)**:
   - Open Serial Monitor (115200 baud)
   - Press any key within 10 seconds to open the setup menu
   - Follow instructions to:
     - Calibrate the scale
     - Apply tare (set zero)

---

## Data Upload (ThingSpeak)

Each cycle, the following data is sent to ThingSpeak:

| Field       | Description                       |
|-------------|-----------------------------------|
| Field 1     | Weight in grams                   |
| Field 2     | Temperature (°C)                  |
| Field 3     | Humidity (%)                      |
| Field 4     | Air pressure (hPa)                |
| Field 5     | Watering alert (0 = no, 1 = yes)  |
| Field 6     | Battery voltage (V)               |

---

## Power Management

Before entering deep sleep:

- Sensors are powered down
- Serial, I²C, Bluetooth, and Wi-Fi are disabled
- RTC memory is used to retain weight history and status flags
- Wake-up is set using a timer (default: 900 seconds)

---

## Enclosure

STL and CAD files for 3D printing the enclosure are located in the `stl` directory.

---

## Planned Features

- BLE configuration interface (active during first 30 seconds after boot)
- Smartphone app for:
  - Real-time status
  - Alert notifications
  - Configuration of thresholds and Wi-Fi settings

---

## Contributing

Pull requests and suggestions are welcome. Please open an issue for bug reports or feature ideas.

---
