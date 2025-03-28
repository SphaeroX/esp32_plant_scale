#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// Function declarations
void initializeNetwork();
void connectToWiFi();
bool sendSensorData(float weight, float temperature, float humidity, float pressure, float batteryVoltage, bool needsWatering);
void disconnectNetwork();

// External variables
extern WiFiClient client;

#endif // NETWORK_H