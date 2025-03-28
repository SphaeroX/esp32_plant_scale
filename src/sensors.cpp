#include "sensors.h"
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"

// Define the global variables declared in the header
HX711 scale;
Adafruit_BME280 bme;
float calibrationFactor = 1.0;
float tareOffset = 0.0;
bool wateringNeeded = false;

// Weight history variables
RTC_DATA_ATTR float weightHistory[HISTORY_SIZE] = {0};
RTC_DATA_ATTR int historyIndex = 0;
RTC_DATA_ATTR int readingCount = 0;

// Stagnation detection variables
RTC_DATA_ATTR float referenceAvgWeight = 0.0;
RTC_DATA_ATTR bool stagnationMode = false;
RTC_DATA_ATTR bool isFirstEvaluation = true;

void initializeSensors()
{
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
}

void loadCalibrationData()
{
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

  // Apply loaded values to the scale
  scale.set_scale(calibrationFactor);
  scale.set_offset(tareOffset);
}

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
  return scale.get_units(NUM_MEASUREMENTS);
}

void addWeightToHistory(float weight)
{
  weightHistory[historyIndex] = weight;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  readingCount++;
}

void checkForStagnation()
{
  // Check if enough data is available
  if (readingCount < (IGNORE_FIRST_READINGS + HISTORY_SIZE))
  {
    Serial.println("Initialization phase: " + String(readingCount) + "/" +
                   String(IGNORE_FIRST_READINGS + HISTORY_SIZE));
    return;
  }

  int validCount = 0;
  // Loop through the weight history and count valid drops
  for (int i = 1; i < HISTORY_SIZE; i++)
  {
    // Check if the previous measurement is at least 1g higher than the current measurement
    if ((weightHistory[i - 1] - weightHistory[i]) >= VALID_DROP_THRESHOLD)
    {
      validCount++;
    }
  }

  if (validCount >= MIN_VALID_DROPS)
  {
    wateringNeeded = false;
    Serial.println("No stagnation detected: " + String(validCount) + " valid drops.");
  }
  else
  {
    wateringNeeded = true;
    Serial.println("Stagnation detected: Only " + String(validCount) + " valid drops.");
  }
}

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

void showSensorData()
{
  Serial.println("\n=== Sensor Data ===");
  if (waitForScaleReady(5000))
  {
    // Get raw reading from the load cell (already averaged)
    float rawValue = getFilteredWeight();
    // Read temperature, humidity, and pressure from BME280
    float temperature = getTemperature();
    float humidity = getHumidity();
    float pressure = getPressure();

    Serial.print("HX711 scale.get_units(5): ");
    Serial.println(scale.get_units(5), 3);

    Serial.print("Load cell raw data (average): ");
    Serial.print(rawValue, 3);
    Serial.println(" (calibrated units)");

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

void powerUpSensors()
{
  scale.power_up();

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1, // Temperature
                  Adafruit_BME280::SAMPLING_X1, // Pressure
                  Adafruit_BME280::SAMPLING_X1, // Humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5);

  delay(500);
}

void powerDownSensors()
{
  scale.power_down();

  bme.setSampling(Adafruit_BME280::MODE_SLEEP,
                  Adafruit_BME280::SAMPLING_NONE, // Temperature
                  Adafruit_BME280::SAMPLING_NONE, // Pressure
                  Adafruit_BME280::SAMPLING_NONE, // Humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5);
}

// Sensor data getters
float getTemperature()
{
  return bme.readTemperature();
}

float getHumidity()
{
  return bme.readHumidity();
}

float getPressure()
{
  return bme.readPressure() / 100.0F; // Convert to hPa
}

bool isWateringNeeded()
{
  return wateringNeeded;
}