#include "eeprom_handler.h"

// Clear stored EEPROM values for tare offset and calibration factor
void clearEEPROM()
{
  Serial.println("Clearing EEPROM...");
  float resetVal = 0.0;
  EEPROM.put(EEPROM_TARE_ADDR, resetVal);
  EEPROM.put(EEPROM_CALIB_ADDR, 1.0);
  EEPROM.commit();
  Serial.println("EEPROM cleared!");
}