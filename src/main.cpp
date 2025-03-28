#include <Arduino.h>
#include <EEPROM.h>
#include "cred.h"
#include "config.h"
#include "sensors.h"
#include "network.h"
#include "menu.h"
#include "eeprom_handler.h"
#include <Wire.h>
#include <ArduinoOTA.h>
#include <esp_adc_cal.h>

// Track if this is the first boot since power-on
RTC_DATA_ATTR bool firstBoot = true;

// Global flag for stagnation detection
bool stagnationDetected = false;

// Function to read battery voltage
float readBatteryVoltage()
{
  // Calculate battery voltage
  float batteryVoltage = (float)(analogRead(PIN_BAT_VOLT)) * 3600 / 4095 * 2;

  if (batteryVoltage > 4300)
  {
    return 0;
  }
  else
  {
    return batteryVoltage;
  }
}

void setupOTA()
{
  ArduinoOTA.setHostname("esp32-plant-scale");
  ArduinoOTA.onStart([]()
                     { Serial.println("Start OTA update..."); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nOTA update complete."); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
      Serial.printf("OTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  delay(100);

  pinMode(PIN_BAT_VOLT, INPUT);

  // Load calibration data from EEPROM
  loadCalibrationData();

  // Initialize network and connect to WiFi
  initializeNetwork();
  connectToWiFi();

  // Initialize sensors
  initializeSensors();

  // Check for menu activation (10 seconds)
  if (firstBoot)
  {
    Serial.println("Press any key for menu (10 seconds)...");
    unsigned long startTime = millis();
    bool menuActivated = false;
    while (millis() - startTime < 10000)
    {
      if (Serial.available() > 0)
      {
        menuActivated = true;
        break;
      }
      ArduinoOTA.handle(); // OTA-Update Ready
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
  // Power up sensors
  powerUpSensors();

  if (waitForScaleReady(5000))
  {
    // Get calibrated weight reading
    float weight = getFilteredWeight();

    // Add to history and check for stagnation
    addWeightToHistory(weight);
    checkForStagnation(); // This function should set 'stagnationDetected' to true when stagnation is detected

    // Read sensor values from BME280
    float temperature = getTemperature();
    float humidity = getHumidity();
    float pressure = getPressure();
    bool needsWatering = isWateringNeeded();
    float batteryVoltage = readBatteryVoltage();

    Serial.print("Weight: ");
    Serial.print(weight, 1);
    Serial.println(" g");

    Serial.print("Measured temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" °C");

    Serial.print("Needs watering: ");
    Serial.println(needsWatering, 1);

    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage, 1);
    Serial.println(" V");

    // Send data to ThingSpeak using network module
    sendSensorData(weight, temperature, humidity, pressure, needsWatering, batteryVoltage);
  }
  else
  {
    Serial.println("Error: HX711 not ready!");
  }

  Serial.println("Turning off sensors...");
  powerDownSensors();

  Serial.println("Entering deep sleep mode...");
  delay(500);

  // Disconnect network
  disconnectNetwork();

  Serial.end();
  Wire.end();
  btStop();
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
