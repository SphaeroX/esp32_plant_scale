#ifndef CONFIG_H
#define CONFIG_H

// GPIO Pin Definitions
#define DOUT_PIN 32     // HX711 data pin
#define SCK_PIN 33      // HX711 clock pin
#define SDA_PIN 26      // I²C SDA pin
#define SCL_PIN 27      // I²C SCL pin
#define PIN_BAT_VOLT 34 // Battery Pin for T-Display ESP32 WiFi

// Deep Sleep Configuration
#define SLEEP_TIME 900 // Deep sleep time in seconds (900 = 15 minutes)

// EEPROM Configuration
#define EEPROM_SIZE 8       // EEPROM size in bytes
#define EEPROM_TARE_ADDR 0  // EEPROM address for tare offset (4 bytes)
#define EEPROM_CALIB_ADDR 4 // EEPROM address for calibration factor (4 bytes)

// Measurement Settings
#define NUM_MEASUREMENTS 10     // Number of measurements per weight value
#define HISTORY_SIZE 10         // Number of stored weight values for evaluation
#define IGNORE_FIRST_READINGS 3 // Number of initial measurements to ignore after startup

// Stagnation Detection Settings
#define MIN_VALID_DROPS 5        // Minimum number of valid drops required from HISTORY_SIZE
#define VALID_DROP_THRESHOLD 1.2 // Minimum drop (in grams) to consider a value valid

#endif // CONFIG_H