#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include "network.h"
#include "cred.h"

// Global WiFi client
WiFiClient client;

// Initialize network components
void initializeNetwork()
{
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

// Connect to WiFi network
void connectToWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi");
  }
}

// Send sensor data to ThingSpeak
bool sendSensorData(float weight, float temperature, float humidity, float pressure, float batteryVoltage, bool needsWatering)
{
  // Check if connected to WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected. Attempting to reconnect...");
    connectToWiFi();
    if (WiFi.status() != WL_CONNECTED)
    {
      return false;
    }
  }

  // Set fields for ThingSpeak
  ThingSpeak.setField(1, weight);
  ThingSpeak.setField(2, temperature);
  ThingSpeak.setField(3, humidity);
  ThingSpeak.setField(4, pressure);
  ThingSpeak.setField(5, batteryVoltage);
  ThingSpeak.setField(6, needsWatering ? 1 : 0);

  // Send data to ThingSpeak
  int httpCode = ThingSpeak.writeFields(channelID, apiKey);

  if (httpCode == 200)
  {
    Serial.println("Data sent to ThingSpeak successfully");
    return true;
  }
  else
  {
    Serial.print("Error sending data to ThingSpeak. HTTP error code: ");
    Serial.println(httpCode);
    return false;
  }
}

// Disconnect from network and power down WiFi
void disconnectNetwork()
{
  client.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Network disconnected");
}