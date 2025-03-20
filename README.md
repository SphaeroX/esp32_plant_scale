# ESP32 Plant Scale 🌱⚖️

## Overview
This project utilizes an ESP32 module powered by an 18650 battery to measure the weight and environmental conditions (temperature, humidity, pressure) of a plant using the BME280 and HX711 sensors. The ESP32 records measurements at defined intervals (default every 15 minutes), sends data to ThingSpeak, and then enters deep sleep mode to save power.

The primary goal is to detect stagnation in weight, indicating dryness of the plant. Once stagnation is detected, the flag `wateringNeeded` becomes `true`. You can monitor this status and the sensor data remotely via ThingSpeak.

---

## ⚙️ Features
- **Periodic Measurement**: Weight, temperature, humidity, and air pressure measurement every defined interval (default: 15 minutes).
- **Deep Sleep Mode**: ESP32 enters deep sleep after sending data to optimize battery life.
- **Automatic Watering Alert**: Detects weight stagnation (plant dryness) and sets a `wateringNeeded` flag, visible on ThingSpeak (Field 6).
- **Flexible Thresholds**: Customizable parameters for detecting stagnation:
  - `stagnationThreshold`: Sensitivity for stagnation detection.
  - `aggressivenessFactor`: Adjusts stagnation detection aggressiveness.
  - `resetThreshold`: Weight increase (in grams) needed to automatically reset the watering alert (default: 300 grams).
- **Temperature Compensation**: Adjust weight measurement accuracy using:
  - `TEMP_COEFF`: Coefficient for temperature compensation.
  - `REFERENCE_TEMP`: Reference temperature for weight compensation.

---

## 🛠️ Setup Instructions

### Hardware Connections
| HX711 (Load Cell Amplifier) | BME280 Sensor |
|-----------------------------|---------------|
| SCK, DT pins defined in code | I2C Pins (SDA, SCL) defined in code |

### Initial Setup
1. Clone this repository:
```bash
git clone <your-repo-url>
```

2. Rename and configure credentials:
   - Locate the file `src/cred_example.h`.
   - Enter your Wi-Fi and ThingSpeak credentials.
   - Rename it to `cred.h`.

3. Upload the sketch to your ESP32 using Arduino IDE or PlatformIO.

4. On first boot, open the Serial Monitor (115200 baud). Press any key to enter setup menu:
   - **Calibrate** your load cell first.
   - **Tare** your scale afterward.

5. Place your plant on the scale and power it on.

---

## 📊 Data Visualization with ThingSpeak
- Create a free account at [ThingSpeak](https://thingspeak.com).
- Monitor sensor data and plant watering status online.
- Field 6 indicates the watering status:
  - `0`: No watering needed
  - `1`: Watering required

---

## 🔋 Components List (BOM)
Below are the search terms you can use on AliExpress (approximate total cost ~17€):

- `HX711 5kg with load cell` *(select appropriate load range for your plant)*
- `ESP32 18650` *(ESP32 module with integrated 18650 battery holder)*
- `18650 battery` *(Li-ion battery, one piece is sufficient)*
- `BME280` *(ensure it is BME280 and NOT BMP280; module with 4 pins)*

---

## 🖨️ Enclosure
The enclosure STL / CAD files for 3D printing can be found in the `stl` folder.

---

## 📱 Planned Features (TODO)
- **Smartphone App (BLE Integration)**:
  - ESP32 enables BLE for 30 seconds at each reboot for setup via mobile.
  - Configure Wi-Fi credentials, ThingSpeak settings, and sensor parameters directly via app.
  - View ThingSpeak data directly in the app.
  - Receive notifications on smartphone when watering is needed.

---

## 📌 Contributions
Feel free to submit issues, feature requests, or pull requests. Your contributions are warmly welcomed!

---

📬 **Contact**
For questions or support, please open an issue or reach out directly through GitHub.