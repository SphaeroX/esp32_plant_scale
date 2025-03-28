#include "menu.h"
#include "eeprom_handler.h"
#include "sensors.h"

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