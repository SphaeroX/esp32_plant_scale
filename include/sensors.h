#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <HX711.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include "config.h"

// Function declarations
void initializeSensors();
bool waitForScaleReady(unsigned long timeoutMillis);
float getFilteredWeight();
void calibrateScale();
void setTare();
void powerDownSensors();
void powerUpSensors();
void loadCalibrationData();
void showSensorData();

// Weight history management
void addWeightToHistory(float newWeight);
void checkForStagnation();

// Sensor data getters
float getTemperature();
float getHumidity();
float getPressure();
bool isWateringNeeded();

// External variables that need to be accessed from main.cpp
extern HX711 scale;
extern Adafruit_BME280 bme;
extern float calibrationFactor;
extern float tareOffset;
extern bool wateringNeeded;

// Weight history variables
extern RTC_DATA_ATTR float weightHistory[HISTORY_SIZE];
extern RTC_DATA_ATTR int historyIndex;
extern RTC_DATA_ATTR int readingCount;
extern RTC_DATA_ATTR bool historyReady;

// Stagnation detection variables
extern RTC_DATA_ATTR float referenceAvgWeight;
extern RTC_DATA_ATTR bool stagnationMode;
extern RTC_DATA_ATTR bool isFirstEvaluation;

#endif // SENSORS_H