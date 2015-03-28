#include <avr/eeprom.h>
#include "MultiHoTTSensor.h"
#include <Arduino.h>

#define EEPROM_SETTINGS_VERSION 2 

void readSettings() {
  eeprom_read_block((void*)&MultiHoTTModuleSettings, (void*)0, sizeof(MultiHoTTModuleSettings));
}

void writeSettings() {
  eeprom_write_block((const void*)&MultiHoTTModuleSettings, (void*)0, sizeof(MultiHoTTModuleSettings));
}

void checkSettings() {
  if (MultiHoTTModuleSettings.version == EEPROM_SETTINGS_VERSION) {
    return;
  } else {
    MultiHoTTModuleSettings.version     = EEPROM_SETTINGS_VERSION;
    MultiHoTTModuleSettings.alarmVBat   = 36;
    MultiHoTTModuleSettings.alarmTemp1  = 50;    
    MultiHoTTModuleSettings.maxAltitude = 300;
    //MultiHoTTModuleSettings.cellcount   = 3;
    writeSettings();
  }
}
