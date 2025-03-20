#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <EEPROM.h>
#include "cred.h"
#include "HX711.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <math.h>

// Define GPIO pins for HX711
#define DOUT_PIN 32
#define SCK_PIN 33

// Define custom I²C pins
#define SDA_PIN 26 // Alternate SDA pin
#define SCL_PIN 27 // Alternate SCL pin

// Deep Sleep time in seconds
#define SLEEP_TIME 900 // 15 minutes

// EEPROM definitions (8 bytes: 4 for tare offset, 4 for calibration factor)
#define EEPROM_SIZE 8
#define EEPROM_TARE_ADDR 0
#define EEPROM_CALIB_ADDR 4

// Measurement settings
#define NUM_MEASUREMENTS 10     // Number of measurements per weight value
#define HISTORY_SIZE 5          // Number of stored weight values for evaluation
#define IGNORE_FIRST_READINGS 3 // Number of initial measurements to ignore after startup

// Array for storing recent weight measurements
RTC_DATA_ATTR float weightHistory[HISTORY_SIZE] = {0};
RTC_DATA_ATTR int historyIndex = 0;
RTC_DATA_ATTR int readingCount = 0;      // Tracks the number of recorded measurements
RTC_DATA_ATTR bool historyReady = false; // Set to true when the array is fully populated for the first time

// Temperature compensation constants
const float TEMP_COEFF = -0.0171;  // Temperature coefficient (per °C)
const float REFERENCE_TEMP = 22.0; // Reference temperature in °C

// Stagnation detection settings
float aggressivenessFactor = 1.0; // Adjustable factor for stagnation sensitivity
float stagnationThreshold = 0.5;  // Threshold in grams for detecting stagnation
float resetThreshold = 300.0;     // Weight increase threshold to reset marker
bool wateringNeeded = false;      // Marker for watering requirement

/*
Stagnation Detection Tuning:
----------------------------------
aggressivenessFactor (float):
- Sensitivity multiplier for detection
- >1.0 = More sensitive (earlier detection)
- <1.0 = More tolerant (requires bigger change)
- Typical: 0.5-3.0

stagnationThreshold (float) [grams]:
- Minimum expected weight change per SLEEP_TIME interval
- Set to smallest meaningful change to detect
- Based on sensor accuracy & system behavior

Example (15-minute intervals):
Normal operation: ~2g decrease per interval
To detect when decrease <0.5g:
stagnationThreshold = 0.5  // Minimum interesting change
aggressivenessFactor = 1.0  // Use threshold directly
Detection: (ActualChange < 0.5 * 1.0) → Stagnation

Tuning Steps:
1. Set threshold = minimum change worth detecting
2. Adjust factor based on false positives/negatives
*/

RTC_DATA_ATTR float referenceAvgWeight = 0.0;
RTC_DATA_ATTR bool stagnationMode = false;
RTC_DATA_ATTR bool isFirstEvaluation = true;

RTC_DATA_ATTR bool firstBoot = true;

// Global variables
float calibrationFactor = 1.0;
float tareOffset = 0.0;

HX711 scale;
WiFiClient client;
Adafruit_BME280 bme;

// Function prototypes
bool waitForScaleReady(unsigned long timeoutMillis);
float getFilteredWeight();
void addWeightToHistory(float newWeight);
void checkForStagnation();
void showMenu();
void calibrateScale();
void setTare();
void clearEEPROM();
void showSensorData();
void menuLoop();

// Wait for the scale to be ready up to a timeout
bool waitForScaleReady(unsigned long timeoutMillis)
{
  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMillis)
  {
    if (scale.is_ready())
    {
      return true;
    }
    delay(10);
  }
  return false;
}

float getFilteredWeight()
{
  return (scale.get_units(NUM_MEASUREMENTS));
}

void addWeightToHistory(float weight)
{
  weightHistory[historyIndex] = weight;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  readingCount++;

  if (historyIndex == 0 && !historyReady)
  {
    historyReady = true;
  }
}

void checkForStagnation()
{
  // Prüfe ob genug Daten vorhanden sind
  if (!historyReady || readingCount <= IGNORE_FIRST_READINGS)
  {
    Serial.println("Initialisierungsphase: " + String(readingCount) + "/" +
                   String(IGNORE_FIRST_READINGS + HISTORY_SIZE));
    return;
  }

  // Berechne gleitenden Durchschnitt
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++)
  {
    sum += weightHistory[i];
  }
  float currentAvg = sum / HISTORY_SIZE;

  // Erste gültige Auswertung
  if (isFirstEvaluation)
  {
    referenceAvgWeight = currentAvg;
    isFirstEvaluation = false;
    Serial.println("Erste stabile Basis: " + String(referenceAvgWeight, 1) + " g");
    return;
  }

  // Stagnationsmodus-Logik
  if (stagnationMode)
  {
    float diff = currentAvg - referenceAvgWeight;

    if (diff > resetThreshold)
    {
      stagnationMode = false;
      wateringNeeded = false;
      Serial.println("Reset durch Gewichtsanstieg: " + String(diff, 1) + " g");
    }
    else if (abs(diff) < stagnationThreshold * aggressivenessFactor)
    {
      wateringNeeded = true;
      Serial.println("Anhaltende Stagnation: Δ" + String(diff, 1) + " g");
    }
  }
  else
  {
    float diff = referenceAvgWeight - currentAvg;

    if (abs(diff) < stagnationThreshold * aggressivenessFactor)
    {
      stagnationMode = true;
      wateringNeeded = true;
      Serial.println("Stagnation erkannt! Referenz: " + String(currentAvg, 1) + " g");
      referenceAvgWeight = currentAvg;
    }
    else
    {
      referenceAvgWeight = currentAvg;
      Serial.println("Normalbereich: " + String(currentAvg, 1) + " g");
    }
  }
}
// Display the main menu via Serial
void showMenu()
{
  Serial.println("\n=== Main Menu ===");
  Serial.println("1 - Calibrate");
  Serial.println("2 - Tare");
  Serial.println("3 - Clear EEPROM");
  Serial.println("4 - Display all sensor data");
  Serial.println("5 - Exit");
  Serial.println("Please enter an option:");
}

// Calibrate the scale using a known weight
void calibrateScale()
{
  Serial.println("Calibration mode activated.");
  Serial.println("Ensure the scale is empty. Press any key when ready.");
  while (Serial.available() == 0)
  {
    // Waiting for user input
  }
  while (Serial.available() > 0)
  {
    Serial.read();
  }

  // Reset scale factor and tare the scale
  scale.set_scale();
  scale.tare();
  Serial.println("Scale set to zero. Now place a known weight on the scale and enter the weight in grams.");
  while (Serial.available() == 0)
  {
    // Waiting for user input
  }
  float knownWeight = Serial.parseFloat();
  while (Serial.available() > 0)
  {
    Serial.read();
  }

  if (knownWeight <= 0)
  {
    Serial.println("Invalid input for known weight.");
    return;
  }
  else
  {
    Serial.print("Known weight: ");
    Serial.print(knownWeight);
    Serial.println(" g");
  }

  // Wait for the reading to stabilize
  delay(1000);
  float reading = scale.get_units(10);
  Serial.print("Measured weight with known load: ");
  Serial.println(reading, 3);

  // Calculate calibration factor: (measured reading) / (known weight)
  calibrationFactor = reading / knownWeight;
  scale.set_scale(calibrationFactor);
  EEPROM.put(EEPROM_CALIB_ADDR, calibrationFactor);
  EEPROM.commit();

  Serial.print("Calibration completed. New calibration factor: ");
  Serial.println(calibrationFactor, 6);
}

// Tare the scale and store the tare offset in EEPROM
void setTare()
{
  Serial.println("Taring... Please remove any load from the scale.");
  if (waitForScaleReady(5000))
  {
    // Get the current raw reading (without calibration factor)
    long rawReading = scale.read_average(10);
    // Store this value as the offset
    scale.set_offset(rawReading);
    // Save the offset for future use
    tareOffset = rawReading;
    EEPROM.put(EEPROM_TARE_ADDR, tareOffset);
    EEPROM.commit();
    Serial.print("Taring completed. New offset saved: ");
    Serial.println(tareOffset, 3);
  }
  else
  {
    Serial.println("Error: HX711 not ready!");
  }
}

// Clear stored EEPROM values for tare offset and calibration factor
void clearEEPROM()
{
  Serial.println("Clearing EEPROM...");
  float resetVal = 0.0;
  EEPROM.put(EEPROM_TARE_ADDR, resetVal);
  EEPROM.put(EEPROM_CALIB_ADDR, 1.0);
  EEPROM.commit();
  tareOffset = resetVal;
  calibrationFactor = 1.0;
  Serial.println("EEPROM cleared!");
}

// Display all sensor data: load cell raw data, calculated weight, and BME280 readings
void showSensorData()
{
  Serial.println("\n=== Sensor Data ===");
  if (waitForScaleReady(5000))
  {
    // Get raw reading from the load cell (already averaged)
    float rawValue = getFilteredWeight();
    // Read temperature, humidity, and pressure from BME280
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;
    // Apply temperature compensation to the weight reading
    float compensation = 1.0 + TEMP_COEFF * (temperature - REFERENCE_TEMP);
    float compensatedWeight = rawValue / compensation;

    Serial.print("HX711 scale.get_units(5): ");
    Serial.println(scale.get_units(5), 3);

    Serial.print("Load cell raw data (average): ");
    Serial.print(rawValue, 3);
    Serial.println(" (calibrated units)");

    Serial.print("Calculated weight (temperature compensated): ");
    Serial.print(compensatedWeight, 3);
    Serial.println(" g");

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    Serial.print("Pressure: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");
  }
  else
  {
    Serial.println("Error: HX711 not ready!");
  }
  Serial.println("=== End of Sensor Data ===\n");
}

// Persistent menu loop that prevents deep sleep until exit is selected
void menuLoop()
{
  bool inMenu = true;
  while (inMenu)
  {
    showMenu();
    while (Serial.available() == 0)
    {
      // Waiting for user input
    }
    char option = Serial.read();
    while (Serial.available() > 0)
    {
      Serial.read();
    }
    switch (option)
    {
    case '1':
      calibrateScale();
      break;
    case '2':
      setTare();
      break;
    case '3':
      clearEEPROM();
      break;
    case '4':
      showSensorData();
      break;
    case '5':
      Serial.println("Exiting menu. Resuming normal operation...");
      inMenu = false;
      break;
    default:
      Serial.println("Invalid option. Please select again.");
      break;
    }
    delay(200);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  delay(100);

  // Load stored calibration factor from EEPROM
  EEPROM.get(EEPROM_CALIB_ADDR, calibrationFactor);
  if (isnan(calibrationFactor) || calibrationFactor == 0)
  {
    calibrationFactor = 1.0;
  }
  Serial.print("Stored calibration factor: ");
  Serial.println(calibrationFactor, 6);

  // Load and apply stored tare offset
  EEPROM.get(EEPROM_TARE_ADDR, tareOffset);
  if (isnan(tareOffset))
  {
    tareOffset = 0.0;
  }
  Serial.print("Stored tare offset: ");
  Serial.println(tareOffset, 3);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize custom I²C pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize BME280 sensor (I2C address 0x76)
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
    {
      delay(1000);
    }
  }

  // Initialize HX711
  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_scale(calibrationFactor);
  scale.set_offset(tareOffset);
  Serial.println("HX711 started");
  delay(500);

  // Check for menu activation (5 seconds)
  if (firstBoot)
  {
    Serial.println("Press any key for menu (5 seconds)...");
    unsigned long startTime = millis();
    bool menuActivated = false;
    while (millis() - startTime < 5000)
    {
      if (Serial.available() > 0)
      {
        menuActivated = true;
        break;
      }
    }
    if (menuActivated)
    {
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      menuLoop();
    }
  }
}

void loop()
{
  scale.power_up();

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1, // Temperature
                  Adafruit_BME280::SAMPLING_X1, // Pressure
                  Adafruit_BME280::SAMPLING_X1, // Humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5);

  delay(500);

  if (waitForScaleReady(5000))
  {
    // Get calibrated weight reading (tare is already subtracted)
    float weight = getFilteredWeight();

    addWeightToHistory(weight);
    checkForStagnation();

    // Read sensor values from BME280
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    // Apply temperature compensation
    float compensation = 1.0 + TEMP_COEFF * (temperature - REFERENCE_TEMP);
    float compensatedWeight = weight / compensation;

    Serial.print("Filtered weight (calibrated and temperature compensated): ");
    Serial.print(compensatedWeight, 1);
    Serial.println(" g");

    Serial.print("Measured temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" °C");

    // Send data to ThingSpeak (Field 1: weight, Field 2: temperature, etc.)
    ThingSpeak.setField(1, compensatedWeight);
    ThingSpeak.setField(2, temperature);
    ThingSpeak.setField(3, humidity);
    ThingSpeak.setField(4, pressure);
    ThingSpeak.setField(5, weight);
    ThingSpeak.setField(6, wateringNeeded ? 1 : 0);
    ThingSpeak.writeFields(channelID, apiKey);
  }
  else
  {
    Serial.println("Error: HX711 not ready!");
  }

  Serial.println("Turning off HX711 and BME280...");
  scale.power_down();

  bme.setSampling(Adafruit_BME280::MODE_SLEEP,
                  Adafruit_BME280::SAMPLING_NONE, // Temperature
                  Adafruit_BME280::SAMPLING_NONE, // Pressure
                  Adafruit_BME280::SAMPLING_NONE, // Humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5);

  Serial.println("Entering deep sleep mode...");
  delay(500);

  // Put ESP32 into deep sleep
  client.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  Serial.end();
  Wire.end();
  delay(100);

  // Enable lowest power consumption mode
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_AUTO); // AUTO due to RTC_DATA_ATTR
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_AUTO); // AUTO due to RTC_DATA_ATTR
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

  firstBoot = false;

  esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000ULL);
  esp_deep_sleep_start();
}
