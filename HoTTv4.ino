#include "HoTTv4.h"
#include "MultiHoTTSensor.h"
#include "Baro.h"
#include <Arduino.h>
#include "SoftwareSerial.h"
#include "MultiWii.h"


#define OFFSET_HEIGHT 500
#define OFFSET_M2S 120
#define OFFSET_M3S 120

#define ARMED MultiHoTTModule.flags & 0x01

static uint8_t outBuffer[173];

#define MAXPAGENO 5

uint8_t CharNo;  // zum Testen

static uint8_t vbat = 100;

SoftwareSerial hottV4Serial(HOTTV4_RXTX , HOTTV4_RXTX);

// =================================================================================
// Common setup method for HoTTv4
// =================================================================================
void hottV4Setup() {
  hottV4Serial.begin(19200);
  hottV4EnableReceiverMode();
 
  // Read and check settings from EEPROM  
  readSettings();
  checkSettings();
  MultiHoTTModule.cellcount = Lipo.cellcount;   
  //MultiHoTTModuleSettings.alarmVBat = 36; // 3,6V  
}

// =================================================================================
// Enables RX and disables TX
// =================================================================================
static inline void hottV4EnableReceiverMode() {
  DDRD &= ~(1 << HOTTV4_RXTX);
  PORTD |= (1 << HOTTV4_RXTX);
}

// =================================================================================
// Enabels TX and disables RX
// =================================================================================
static inline void hottV4EnableTransmitterMode() {
  DDRD |= (1 << HOTTV4_RXTX);
}

// =================================================================================
// Writes out given byte to HoTT serial interface.
// If in debug mode, data is also written to UART serial interface. 
// =================================================================================
static void hottV4SerialWrite(uint8_t c) {
  hottV4Serial.write(c);
}

// =================================================================================
// HoTTv4 Text Mode
// =================================================================================
static void hottV4ClearAllTextLines() {
  memset(&HoTTv4ElectricalAirTextModule.text[0], ' ', 8*21);
}

// =================================================================================
// Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule
// =================================================================================
static void hottV4WriteLine(uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = text[index];
    } else {
      writeText = 0;
      HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = ' ';
    }
  }
}

// =================================================================================
// Writes out a single inversed text line of max. 21 chars into HoTTv4ElectricalAirTextModule
// =================================================================================
static void hottV4WriteInvLine(uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = 128 + text[index];
    } else {
      writeText = 0;
      HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = ' ';
    }
  }
}

// =================================================================================
// Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule.
// If row == line it gets a selection indicator and given row is also highlighted.
// =================================================================================
static void hottV4WriteLine(uint8_t line, const char *text, uint8_t row, uint8_t col) {
  char lineText[21];
  uint8_t inCol = 0;

  enum {
    IDLE,
    COLON,
    SPACE,
    COL,
    DONE,
  } state = IDLE;

  const char selectionIndicator = (line == row) ? '>' : ' ';  
  snprintf(lineText, 21, "%c%s", selectionIndicator, text);  
  
  for (uint8_t index = 0 ; index < 21 ; index++) {
    uint8_t c = lineText[index];
    
    if (IDLE == state) {
      state = (':' == c) ? COLON : IDLE;
    } else if (COLON == state) {
      state = (' ' == c) ? SPACE : COLON; 
    } else if (SPACE == state) {
      if ('.' <= c) {
        inCol++;
        state = COL;
      } else {
        state = SPACE;
      }
    } else if (COL == state) {
      if (' ' == c) {
        state = SPACE;
      } else if (0x0 == c) {
        state = DONE;
      } else {
        state = COL;
      }
    } else if (DONE == c) {
      break;
    }
    
    // invertierter Text nach ": "
    if ((COL == state) && (inCol == col) && (line == row)) {
      lineText[index] += 128;
    } 
  }
  hottV4WriteLine(line, lineText);
}

// =================================================================================
// Expects an array of at least size bytes. All bytes till size will be transmitted
// to the HoTT capable receiver. Last byte will always be treated as checksum and is
// calculated on the fly.
// =================================================================================
static void hottV4SendData(uint8_t *data, uint8_t size) {
  hottV4Serial.flush();
  
  // Protocoll specific waiting time
  // to avoid collisions
  delay(5);
  
  // wenn kein anderes Modul sendet, dann können wir
  if (hottV4Serial.available() == 0) {
    hottV4EnableTransmitterMode();
    
    uint16_t crc = 0;

    for (uint8_t i = 0; i < (size - 1); i++) {
      crc += data[i];     
      hottV4SerialWrite(data[i]);
      
      // Protocoll specific delay between each transmitted byte
      delayMicroseconds(HOTTV4_TX_DELAY);
    }
    
    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);
    
    hottV4EnableReceiverMode();  
  }
}

// *********************************************************************************
// ***************  M U L T I W I I  G P S  ****************************************
// *********************************************************************************
#ifdef HOTTV4_GPS  

// =================================================================================
// =================================================================================
  static void hottV4GPSUpdate() {

    //number of satelites
    HoTTV4GPSModule.GPSNumSat = MultiHoTTModule.GPS_numSat;   
       
    uint8_t Fixchar;  
    if (HoTTV4GPSModule.GPSNumSat<3) {
     Fixchar = 0x2d;  // "-"
    } 
    if (HoTTV4GPSModule.GPSNumSat>=3) {
      Fixchar = 0x2e; // "."
    }    
    if (HoTTV4GPSModule.GPSNumSat>=5) {
      Fixchar = 0x66; // "f"
    }    
      
    HoTTV4GPSModule.climbrateLow = 30000 & 0x00FF;
    HoTTV4GPSModule.climbrateHigh= 30000>>8;
    HoTTV4GPSModule.climbrate3s  = OFFSET_M3S;
       
    if (MultiHoTTModule.GPS_fix > 0) { 
      /** GPS fix */
      HoTTV4GPSModule.GPS_fix    = Fixchar; 
      HoTTV4GPSModule.GPSFixChar = Fixchar; 
      
      //latitude MultiHoTTModule.GPS_latitude =510322600
      HoTTV4GPSModule.LatitudeNS=(MultiHoTTModule.GPS_latitude<0);     
      uint8_t deg  = MultiHoTTModule.GPS_latitude / 10000000;                   // = 51°
      uint32_t sec = (MultiHoTTModule.GPS_latitude - (deg * 10000000)) * 6;    // = 1935600
      uint8_t min  = sec / 1000000;                                             // = 1,9356' --> 1
      sec = sec % 1000000;                                                     // = 935600 
      sec = sec * 6;                                                           // = 5613600
      sec = sec / 1000;                                                        // = 5613
      uint16_t degMin = (deg * 100) + min;                                     // = 5101
      HoTTV4GPSModule.LatitudeMinLow = degMin;                                 
      HoTTV4GPSModule.LatitudeMinHigh = degMin >> 8;                           
      HoTTV4GPSModule.LatitudeSecLow = sec; 
      HoTTV4GPSModule.LatitudeSecHigh = sec >> 8;
          
      //longitude 
      HoTTV4GPSModule.longitudeEW=(MultiHoTTModule.GPS_longitude<0);
      deg = MultiHoTTModule.GPS_longitude / 10000000;
      sec = (MultiHoTTModule.GPS_longitude - (deg * 10000000)) * 6;
      min = sec / 1000000;
      sec = sec % 1000000;
      sec = sec * 6;
      sec = sec / 1000;  
      degMin = (deg * 100) + min;
      HoTTV4GPSModule.longitudeMinLow = degMin;
      HoTTV4GPSModule.longitudeMinHigh = degMin >> 8; 
      HoTTV4GPSModule.longitudeSecLow = sec; 
      HoTTV4GPSModule.longitudeSecHigh = sec >> 8;
      
      /** GPS Speed in km/h */
      uint16_t speed = (MultiHoTTModule.GPS_speed / 100) * 36; // 0.1m/s * 0.36 = km/h
      HoTTV4GPSModule.GPSSpeedLow = speed & 0x00FF;
      HoTTV4GPSModule.GPSSpeedHigh = speed >> 8;
      /** Distance to home */
      HoTTV4GPSModule.distanceLow = MultiHoTTModule.GPS_distanceToHome & 0x00FF;
      HoTTV4GPSModule.distanceHigh = MultiHoTTModule.GPS_distanceToHome >> 8; 
      /** Altitude */
      HoTTV4GPSModule.altitudeLow = MultiHoTTModule.GPS_altitude & 0x00FF;
      HoTTV4GPSModule.altitudeHigh = MultiHoTTModule.GPS_altitude >> 8;
      /** Homedir */
      HoTTV4GPSModule.HomeDirection = MultiHoTTModule.GPS_directionToHome;
      
      // Flugrichtung
      HoTTV4GPSModule.flightDirection = MultiHoTTModule.GPS_ground_course/10;
      
    } else {
      // kein GPS-Fix (<5 Sats)
      HoTTV4GPSModule.GPS_fix = Fixchar; 
      HoTTV4GPSModule.GPSFixChar = Fixchar;
      HoTTV4GPSModule.altitudeLow = (OFFSET_HEIGHT + MultiHoTTModule.altitude/100) & 0x00FF;
      HoTTV4GPSModule.altitudeHigh = (OFFSET_HEIGHT + MultiHoTTModule.altitude/100) >> 8;      
    }
  }


// =================================================================================
// Sends HoTTv4 capable GPS telemetry frame.
// =================================================================================

static void hottV4SendGPS() {
  /** Minimum data set for EAM */
  HoTTV4GPSModule.startByte     = 0x7C;
  HoTTV4GPSModule.sensorID      = HOTTV4_GPS_SENSOR_ID;
  HoTTV4GPSModule.sensorTextID  = HOTTV4_GPS_SENSOR_TEXT_ID;
  HoTTV4GPSModule.endByte       = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GPSModule.alarmTone     = 0x0;
  HoTTV4GPSModule.alarmInverse1 = 0x0;
    
  hottV4GPSUpdate();
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy GPS data to output buffer
  memcpy(&outBuffer, &HoTTV4GPSModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}
#endif
// *********************************************************************************
// *******E N D **  M U L T I W I I  G P S  ****************************************
// *********************************************************************************


// *********************************************************************************
// *************** G E N E R A L  A I R  M O D U L E *******************************
// *********************************************************************************
#ifdef HOTTV4_GAM 
// =================================================================================
// Writes cell 1-4 high, low values and if not available 
// calculates vbat.
// =================================================================================

static void hottV4GAMUpdateBattery() {
  // Multiwii : Zellspannungen in Schritten von 0,01V
  // ElectricAirModule: Zellspannungen in Schritten von 0,02V !!
  // 326 == 3,26V
  MultiHoTTModule.cell1 = Lipo.cell1 * 100;
  MultiHoTTModule.cell2 = Lipo.cell2 * 100;
  MultiHoTTModule.cell3 = Lipo.cell3 * 100;
  MultiHoTTModule.cell4 = Lipo.cell4 * 100;
  
  HoTTV4GeneralAirModule.cell[0] = MultiHoTTModule.cell1/2; //#7 Volt Cell 1 (in 20 mV increments, 210 == 4.20 V)
  HoTTV4GeneralAirModule.cell[1] = MultiHoTTModule.cell2/2; //#8 Volt Cell 2 (in 20 mV increments, 210 == 4.20 V)
  HoTTV4GeneralAirModule.cell[2] = MultiHoTTModule.cell3/2; //#9 Volt Cell 3 (in 20 mV increments, 210 == 4.20 V)
  HoTTV4GeneralAirModule.cell[3] = MultiHoTTModule.cell4/2; //#10 Volt Cell 4 (in 20 mV increments, 210 == 4.20 V)
  HoTTV4GeneralAirModule.cell[4] = 0;                       //#11 Volt Cell 5 (in 20 mV increments, 210 == 4.20 V)
  HoTTV4GeneralAirModule.cell[5] = 0;                       //#12 Volt Cell 6 (in 20 mV increments, 210 == 4.20 V)
  
  MultiHoTTModule.vbat1 = (MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3 + MultiHoTTModule.cell4) /10 ; 
  
  HoTTV4GeneralAirModule.battery1= MultiHoTTModule.vbat1;
  HoTTV4GeneralAirModule.battery2= 0; 
  HoTTV4GeneralAirModule.fuel_procent= HoTTV4GeneralAirModule.battery1 *100 / (42*MultiHoTTModule.cellcount);//#19 Fuel capacity in %. Values 0--100
                                                                                                             //graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
  HoTTV4GeneralAirModule.main_voltage=HoTTV4GeneralAirModule.battery1;//#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                                                                      //#32 MSB (Appears in GAM display right as alternate display.)
  HoTTV4GeneralAirModule.min_cell_volt = Lipo.GetWeakCell() * 50;     //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  HoTTV4GeneralAirModule.min_cell_volt_num = Lipo.weakcellno;         //#38 number of the cell with the lowest voltage

  if ( MultiHoTTModule.vbat1 <= MultiHoTTModuleSettings.alarmVBat * MultiHoTTModule.cellcount) {
    HoTTV4GeneralAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4GeneralAirModule.alarmInverse1 |= 0x2; // Invert Voltage display
  } 
}


// =================================================================================
// =================================================================================

static void hottV4GAMUpdateTemperatures() {
  HoTTV4GeneralAirModule.temperature1 = 20 + MultiHoTTModule.temp;
  HoTTV4GeneralAirModule.temperature2 = 20;
  //if (HoTTV4ElectricAirModule.temp1 >= (20 + MultiHoTTModuleSettings.alarmTemp1)) {
  //  HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationMaxTemperature;  
  //  HoTTV4ElectricAirModule.alarmInverse |= 0x8; // Invert Temp1 display
  //}
}


// =================================================================================
// =================================================================================
static void hottV4GAMUpdateBaro() {
  // Altitude in cm
  MultiHoTTModule.altitude = 100 * Baro.altitude;  // in cm
  
  if (MultiHoTTModule.altitude0==0) {
    // Starthöhe speichern
    MultiHoTTModule.altitude0 = MultiHoTTModule.altitude;
  }  
  MultiHoTTModule.temp = Baro.temperature;
  
  // berechnung der Steigrate
  uint32_t now = millis();
  int16_t m2s;
  int16_t time;
  int16_t dist;
  if (now-MultiHoTTModule.lastbaroread >200) {
    time = (now - MultiHoTTModule.lastbaroread); //ms
    dist = (MultiHoTTModule.altitude - MultiHoTTModule.lastaltitude); // cm
    m2s =  dist / time /1000;  // cm/s
    MultiHoTTModule.m2s = m2s;
  
    // aktuellen Stand speichern
    MultiHoTTModule.lastbaroread = now;
    MultiHoTTModule.lastaltitude = MultiHoTTModule.altitude;
  }
  
  HoTTV4GeneralAirModule.altitude = OFFSET_HEIGHT + MultiHoTTModule.altitude/100 - MultiHoTTModule.altitude0/100;
  HoTTV4GeneralAirModule.pressure = Baro.pressure / 10000.0;  // Baro.pressure = in Pascal
                                                             // Hott will in 0.1 bar Schritten
  HoTTV4GeneralAirModule.climbrate = 30000 + m2s;                                                         
}


// =================================================================================
// Sends HoTTv4 capable EAM telemetry frame.
// =================================================================================

static void hottV4SendGAM() {
  /** Minimum data set for GAM */
  HoTTV4GeneralAirModule.startByte     = 0x7C;
  HoTTV4GeneralAirModule.sensorID      = HOTTV4_GENERAL_AIR_SENSOR_ID;
  HoTTV4GeneralAirModule.sensorTextID  = HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID;
  HoTTV4GeneralAirModule.endByte       = 0x7D;
  /** ### */
  
  /** Reset alarms */
  HoTTV4GeneralAirModule.alarmTone     = 0x0;
  HoTTV4GeneralAirModule.alarmInverse1 = 0x0; 
  
  hottV4GAMUpdateBattery();
  hottV4GAMUpdateTemperatures();
  hottV4GAMUpdateBaro();
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4GeneralAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize); 
}
#endif
// *********************************************************************************
// ******* E N D * G E N E R A L  A I R  M O D U L E *******************************
// *********************************************************************************


// *********************************************************************************
// *************** E L E C T R I C  A I R  M O D U L E *****************************
// ********************************************************************************* 

#ifdef HOTTV4_EAM 
// =================================================================================
// Writes cell 1-4 high, low values and if not available 
// calculates vbat.
// =================================================================================
static void hottV4EAMUpdateBattery() { 
 
  // Multiwii : Zellspannungen in Schritten von 0,01V
  // ElectricAirModule: Zellspannungen in Schritten von 0,02V !!
  // 326 == 3,26V
  MultiHoTTModule.cell1 = Lipo.cell1 * 100;
  MultiHoTTModule.cell2 = Lipo.cell2 * 100;
  MultiHoTTModule.cell3 = Lipo.cell3 * 100;
  MultiHoTTModule.cell4 = Lipo.cell4 * 100;
    
  HoTTV4ElectricAirModule.cell1L = MultiHoTTModule.cell1/2;
  if (HoTTV4ElectricAirModule.cell1H < MultiHoTTModule.cell1/2) {
    HoTTV4ElectricAirModule.cell1H = MultiHoTTModule.cell1/2;
  }
  
  HoTTV4ElectricAirModule.cell2L = MultiHoTTModule.cell2/2;
  if (HoTTV4ElectricAirModule.cell2H < MultiHoTTModule.cell2/2) {
    HoTTV4ElectricAirModule.cell2H = MultiHoTTModule.cell2/2;
  }
  
  HoTTV4ElectricAirModule.cell3L = MultiHoTTModule.cell3/2;
  if (HoTTV4ElectricAirModule.cell3H < MultiHoTTModule.cell3/2) {
    HoTTV4ElectricAirModule.cell3H = MultiHoTTModule.cell3/2;
  }

  HoTTV4ElectricAirModule.cell4L = MultiHoTTModule.cell4/2;
  if (HoTTV4ElectricAirModule.cell4H < MultiHoTTModule.cell4/2) {
    HoTTV4ElectricAirModule.cell4H = MultiHoTTModule.cell4/2;
  }  
   
  MultiHoTTModule.vbat1 = (MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3 + MultiHoTTModule.cell4) /10 ; 
  
  HoTTV4ElectricAirModule.driveVoltageLow  = MultiHoTTModule.vbat1 & 0xFF;
  HoTTV4ElectricAirModule.driveVoltageHigh = MultiHoTTModule.vbat1 >> 8;
  HoTTV4ElectricAirModule.battery1Low      = MultiHoTTModule.vbat1 & 0xFF; 
  HoTTV4ElectricAirModule.battery1High     = MultiHoTTModule.vbat1 >> 8; 
  
  #ifdef MultiWii_VBat
    HoTTV4ElectricAirModule.battery2Low  = MultiHoTTModule.vbat2 & 0xFF; 
    HoTTV4ElectricAirModule.battery2High = MultiHoTTModule.vbat2 >> 8;
  #endif

  if ( MultiHoTTModule.vbat1 <= MultiHoTTModuleSettings.alarmVBat * MultiHoTTModule.cellcount) {
    HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse1 |= 0x80; // Invert Voltage display
  } 
}

// =================================================================================
// =================================================================================
static void hottV4EAMUpdateTemperatures() {
  HoTTV4ElectricAirModule.temp1 = 20 + MultiHoTTModule.temp;
  HoTTV4ElectricAirModule.temp2 = 20;

  //if (HoTTV4ElectricAirModule.temp1 >= (20 + MultiHoTTModuleSettings.alarmTemp1)) {
  //  HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationMaxTemperature;  
  //  HoTTV4ElectricAirModule.alarmInverse |= 0x8; // Invert Temp1 display
  //}
}


// =================================================================================
// =================================================================================
static void hottV4EAMUpdateBaro() {
  // Altitude in cm
  MultiHoTTModule.altitude = 100 * Baro.altitude;  // in cm
  
  if (MultiHoTTModule.altitude0==0) {
    // Starthöhe speichern
    MultiHoTTModule.altitude0 = MultiHoTTModule.altitude;
  }  
  MultiHoTTModule.temp = Baro.temperature;
  
  // berechnung der Steigrate
  uint32_t now = millis();
  int16_t m2s;
  int16_t time;
  int16_t dist;
  if (now-MultiHoTTModule.lastbaroread >200) {
    time = (now - MultiHoTTModule.lastbaroread); //ms
    dist = (MultiHoTTModule.altitude - MultiHoTTModule.lastaltitude); // cm
    m2s =  dist / time /1000;  // cm/s
    MultiHoTTModule.m2s = m2s;
  
    // aktuellen Stand speichern
    MultiHoTTModule.lastbaroread = now;
    MultiHoTTModule.lastaltitude = MultiHoTTModule.altitude;
  }
  
  HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + MultiHoTTModule.altitude/100 - MultiHoTTModule.altitude0/100;  
}


// =================================================================================
// Sends HoTTv4 capable EAM telemetry frame.
// =================================================================================
 
static void hottV4SendEAM() {
  /** Minimum data set for EAM */
  HoTTV4ElectricAirModule.startByte     = 0x7C;
  HoTTV4ElectricAirModule.sensorID      = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
  HoTTV4ElectricAirModule.sensorTextID  = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTV4ElectricAirModule.endByte       = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4ElectricAirModule.alarmTone     = 0x0;
  HoTTV4ElectricAirModule.alarmInverse1 = 0x0;
  
  #ifdef READ_LIPO
  hottV4EAMUpdateBattery();
  #endif
  
  hottV4EAMUpdateTemperatures();
  
  #ifdef READ_MWC
  HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + MultiHoTTModule.altitude/100; 
  #endif
  #ifdef READ_BARO
  hottV4EAMUpdateBaro();
  #endif

  HoTTV4ElectricAirModule.current       = MultiHoTTModule.current / 10; 
    
  // Steigrate
  HoTTV4ElectricAirModule.m2s           = MultiHoTTModule.m2s + OFFSET_M2S; 
  HoTTV4ElectricAirModule.m3s           = OFFSET_M3S;
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}
#endif


// =================================================================================
// Sends HoTTv4 capable EAM text frame.
// =================================================================================
static void hottV4SendText(uint8_t row, uint8_t col, uint8_t page) {
  char text[21];
  float CellVoltage;  
  int CellV;
  int CellN;
  uint8_t flightmode;
  char x;

  /** Minimum data set for EAM Text mode */
  HoTTv4ElectricalAirTextModule.startByte    = 0x7B;
  HoTTv4ElectricalAirTextModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTv4ElectricalAirTextModule.endByte      = 0x7D;
  
  HoTTv4ElectricalAirTextModule.alarm  = MultiHoTTModule.alarm;

  // Clear output buffer
  memset(&outBuffer, 0x0, sizeof(outBuffer));
  
  hottV4ClearAllTextLines();
  
  switch (page) { 
    case PAGE_STATUS : {
      hottV4WriteInvLine(0, "MultiHoTTSensor(MW) >" );
      //                     123456789|1234?6789|1
      
      snprintf(text,21,"Profil    :   %1i  ",MultiHoTTModule.currentprofile);
      hottV4WriteLine(1,text,row,col);  
      
      snprintf(text,21,"ARMED     : [%c]",(MultiHoTTModule.flags & getValueOfBoxID(BOXARM))? 'x' : ' ');
      hottV4WriteLine(2, text, row, col);
      
      flightmode = (MultiHoTTModule.flags & 0x6) >>1; 
      switch (flightmode) {
        case 0 :
          snprintf(text,21,"F-MODE    : ACRO");
          break; 
        case 1 : 
          snprintf(text,21,"F-MODE    : ANGLE");
          break; 
        case 2 : 
          snprintf(text,21,"F-MODE    : HORIZON");        
          break; 
      }      
      hottV4WriteLine(3,text,row,col);

      snprintf(text,21,"MAG       : [%c]",(MultiHoTTModule.flags & getValueOfBoxID(BOXMAG))? 'x' : ' ');     
      hottV4WriteLine(4,text,row,col);

      snprintf(text,21,"BARO      : [%c]",(MultiHoTTModule.flags & getValueOfBoxID(BOXBARO))? 'x' : ' ');
      hottV4WriteLine(5,text,row,col);
      
      break;            
    }  
    case PAGE_SENSOR  : {
      hottV4WriteInvLine(0, "(2) Flugdaten      <>");
      //                     123456789|123456789|1      
      
      snprintf(text,21, "Zellen   : %4i", MultiHoTTModule.cellcount);
      //                 123456789|123456789|1      
      hottV4WriteLine(1, text, row, col);

      // kleinste Zellspannung
      CellVoltage = Lipo.GetWeakCell();  
      CellV = CellVoltage;
      CellN = (CellVoltage - CellV) *100;
      snprintf(text, 21, "Umin (%1i) : %1i.%2i V",Lipo.weakcellno,CellV,CellN);     
      //                  123456789|123456789|1      
      hottV4WriteLine(2,text,row,col);
      
      // Höhe      
      snprintf(text, 21, "Hoehe    :   %3i m",MultiHoTTModule.altitude/100);
      hottV4WriteLine(4, text, row, col);
      
      // Winkel X/Y      
      snprintf(text, 21, "X        :  %4i `",MultiHoTTModule.attitudeAngles1/10);
      hottV4WriteLine(5, text, row, col);
      
      snprintf(text, 21, "Y        :  %4i `",MultiHoTTModule.attitudeAngles2/10);
      //                 123456789|123456789|1      
      hottV4WriteLine(6, text, row, col);
   
      break;
    }
    case PAGE_ALARM : {
      hottV4WriteInvLine(0, "(3) Alarme         <>");
      //                     123456789|123456789|1
      
      //Write formatted output to sized buffer
      //int snprintf ( char * s, size_t n, const char * format, ... );
      snprintf(text, 21, "ALARM VOLT : %2i.%1i V", MultiHoTTModuleSettings.alarmVBat / 10, MultiHoTTModuleSettings.alarmVBat % 10);
      hottV4WriteLine(2, text, row, col);
 
       //snprintf(text, 21, "Cycletime:  %4i",MultiHoTTModule.cycletime);
      //hottV4WriteLine(3,text,row,col);
      
      //snprintf(text, 21, "ALARM TEMP1:  %3i `C", MultiHoTTModuleSettings.alarmTemp1);
      //hottV4WriteLine(3, "", row, col);       
      //hottV4WriteLine(4, "", row, col);
      //hottV4WriteLine(5, "", row, col);
      //hottV4WriteLine(6, "", row, col);
      //hottV4WriteLine(7, "", row, col);     

      break;      

    }
    case PAGE_PID: {
      hottV4WriteInvLine(0, "(4) PID-Werte      <>");
      //                     123456789|123456789|1
      
      //ROLL / PITCH / YAW / ALT / POS / POSR / NAVR / LEVEL /MAG
      //0      3       6     9     12    15     18     21     24
 
      snprintf(text, 21, "ROLL/PITCH P : %3i",MultiHoTTModule.pidvalues[0]);
      hottV4WriteLine(1,text,row,col);
      
      snprintf(text, 21, "           I : %3i",MultiHoTTModule.pidvalues[1]);
      hottV4WriteLine(2,text,row,col);
      
      snprintf(text, 21, "           D : %3i",MultiHoTTModule.pidvalues[2]);
      hottV4WriteLine(3,text,row,col);
      
      snprintf(text, 21, "YAW        P : %3i",MultiHoTTModule.pidvalues[6]);
      hottV4WriteLine(4,text,row,col);
      
      snprintf(text, 21, "           I : %3i",MultiHoTTModule.pidvalues[7]);
      hottV4WriteLine(5,text,row,col);
      
      snprintf(text, 21, "LEVEL      P : %3i",MultiHoTTModule.pidvalues[21]);
      hottV4WriteLine(6,text,row,col);

      snprintf(text, 21, "           I : %3i",MultiHoTTModule.pidvalues[22]);
      hottV4WriteLine(7,text,row,col);
      break;
    }
    case PAGE_RCTUNING : {      
      hottV4WriteInvLine(0, "(5) RC-Rates        <"); 
      //RC_RATE/RC_EXPO/RollPitchRate/YawRate/DynThrPID/Throttle_MID/Throttle_EXPO
      //0       1       2             3       4         5            6 
      snprintf(text, 21, "RC Rate       : %3i",MultiHoTTModule.rcrates[0]);
      hottV4WriteLine(1,text,row,col);

      snprintf(text, 21, "RC Expo Rate  : %3i",MultiHoTTModule.rcrates[1]);
      hottV4WriteLine(2,text,row,col);

      snprintf(text, 21, "Roll/Pitch R. : %3i",MultiHoTTModule.rcrates[2]);
      hottV4WriteLine(3,text,row,col);

      snprintf(text, 21, "Yaw Rate      : %3i",MultiHoTTModule.rcrates[3]);
      hottV4WriteLine(4,text,row,col);

      //snprintf(text, 21, "DynThrPID    : %3i",MultiHoTTModule.rcrates[4]);
      //hottV4WriteLine(5,text,row,col);

      snprintf(text, 21, "Throttle Mid  : %3i",MultiHoTTModule.rcrates[5]);
      hottV4WriteLine(5,text,row,col);
      
      snprintf(text, 21, "Throttle Expo : %3i",MultiHoTTModule.rcrates[6]);
      hottV4WriteLine(6,text,row,col);
      
      break;
    }  
//    case 6 : {
//      //hottV4WriteLine(0,"Seite 3   Charset   <");
//      //                  123456789|123456789|1
//      hottV4WriteLine(0, " 12345678901234567890");
//      //                  123456789|123456789|1
//
//      CharNo=1;
//      for (int zeile=1;zeile<8;zeile++) {
//        snprintf(text, 21, "                     ");
//        for (int spalte=1;spalte<21;spalte++) {
//          text[spalte]= CharNo<128 ? (char)CharNo++ : (char)32;
//        }
//        hottV4WriteLine(zeile, text);  
//      }    
//      break;
//    }  
  }
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4ElectricalAirTextModule, kHoTTv4TextPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4TextPacketSize);
}

// *********************************************************************************
// *******E N D ** E L E C T R I C  A I R  M O D U L E *****************************
// ********************************************************************************* 

// *********************************************************************************
// ******* T E X T  F R A M E S ****************************************************
// ********************************************************************************* 


// =================================================================================
// =================================================================================
void hottV4HandleDefaultTextFrame(byte key) {
  
  static uint8_t row  = 2;  // default row
  static uint8_t col  = 0;  // default column
  static uint8_t page = 1;  // default page  
  
  // text pages 
  switch (key) {
    
    // *** next page ****
    case HOTTV4_BUTTON_NEXT:
      if (page < MAXPAGENO) {
        page++;
      }
      break;
      
    // *** previous page ****
    case HOTTV4_BUTTON_PREV:
     if (page > 1) {
       page--;
      }
      break;
      
    // ***  down/up or value +/- *** 
    case HOTTV4_BUTTON_DEC: {
      if (col) {
        if (page==PAGE_ALARM) {
          if (row == 2) {  // Settings
            MultiHoTTModuleSettings.alarmVBat -= 1;
          } else if (row == 3) {
            MultiHoTTModuleSettings.alarmTemp1 -= 1;
          } 
        } else if (page ==PAGE_PID) {  //PID
          switch (row) {
            case 1:
              MultiHoTTModule.pidvalues[0] -=1;
              MultiHoTTModule.pidvalues[3] -=1;
              break;
            case 2:
              MultiHoTTModule.pidvalues[1] -=1;
              MultiHoTTModule.pidvalues[4] -=1;
              break;
            case 3:
              MultiHoTTModule.pidvalues[2] -=1;
              MultiHoTTModule.pidvalues[5] -=1;
              break;
            case 4:
              MultiHoTTModule.pidvalues[6] -=1;
              break;
            case 5:
              MultiHoTTModule.pidvalues[7] -=1;
              break;
            case 6:
              MultiHoTTModule.pidvalues[21] -=1;
              break;
            case 7:
              MultiHoTTModule.pidvalues[22] -=1;
              break;
          }
          MultiHoTTModule.pidvalueschanged = true;
        } 
        else if (page==PAGE_RCTUNING) {
          if (row<5) {
            if (MultiHoTTModule.rcrates[row-1]>0) MultiHoTTModule.rcrates[row-1] -=1;
          }  
          if (row==5) {
            if (MultiHoTTModule.rcrates[5]>0) MultiHoTTModule.rcrates[5] -=1;
          }   
          if (row==6) {
            if (MultiHoTTModule.rcrates[6]>0) MultiHoTTModule.rcrates[6] -=1;
          }   
          MultiHoTTModule.rcvalueschanged = true;
        }  
      } else {
        if (page>=PAGE_PID) {
          row = row > 1 ? row - 1 : row;  // at page #4 Data start at line 1
        } else { //<>4
          row = row > 2 ? row - 1 : row;  //normally data start at line 2
        }
      }
      break;
    }  
    case HOTTV4_BUTTON_INC:
      if (col) {
        if (page==PAGE_ALARM) {
          if (row == 2) {
            MultiHoTTModuleSettings.alarmVBat += 1;
          } else if (row == 3) {
            MultiHoTTModuleSettings.alarmTemp1 += 1;
          } 
        } else if (page==PAGE_PID ) {  //PID
          switch (row) {
            case 1:
              MultiHoTTModule.pidvalues[0] +=1; // Roll  P
              MultiHoTTModule.pidvalues[3] +=1; // Pitch P
              break;
            case 2:
              MultiHoTTModule.pidvalues[1] +=1; // Roll I
              MultiHoTTModule.pidvalues[4] +=1; // Pitch I
              break;
            case 3:
              MultiHoTTModule.pidvalues[2] +=1; // Roll D
              MultiHoTTModule.pidvalues[5] +=1; // Pitch D
              break;
            case 4:
              MultiHoTTModule.pidvalues[6] +=1;
              break;
            case 5:
              MultiHoTTModule.pidvalues[7] +=1;
              break;
            case 6:
              MultiHoTTModule.pidvalues[21] +=1;
              break;
            case 7:
              MultiHoTTModule.pidvalues[22] +=1;
              break;         
          }
          MultiHoTTModule.pidvalueschanged = true;
        } 
        else if (page==PAGE_RCTUNING) {
          if (row<5) {
            if (MultiHoTTModule.rcrates[row-1]<250) MultiHoTTModule.rcrates[row-1] +=1;
          }  
          if (row==5) {
            if (MultiHoTTModule.rcrates[5]<250) MultiHoTTModule.rcrates[5] +=1;
          }   
          if (row==6) {
            if (MultiHoTTModule.rcrates[6]<250) MultiHoTTModule.rcrates[6] +=1;
          }   
          MultiHoTTModule.rcvalueschanged = true;          
        }  
      } else {
        row = row < 7 ? row + 1 : row;
      }
      break;
      
    // *** start editing in a field *** 
    case HOTTV4_BUTTON_SET:
    
      if (page==PAGE_ALARM) {
        // Spalte toggeln
        col = col? col=0 : col=1;
      } 
      else if (!(ARMED)) {  //Multiwii setup changeable if not armed
        if ((page==PAGE_PID) || (page==PAGE_RCTUNING)) {
          if (col) {
            col = 0;
            MultiHoTTModule.setvalues = true;        
          }
          else {
            col = 1;
          }
        }        
      }       
      break;
  }        
  hottV4SendText(row, col, page);  
}

// all sensors use the EAM textframe code
void handleEAMTextFrame(byte key) {
  hottV4HandleDefaultTextFrame(key);  
}

void handleGAMTextFrame(byte key) {
  hottV4HandleDefaultTextFrame(key);    
}

void handleGPSTextFrame(byte key) {
  hottV4HandleDefaultTextFrame(key);    
}


// =================================================================================
// =================================================================================
void hottV4HandleTextFrame(byte sensor, byte key) {
  
   switch (sensor) {

     case HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID :
       handleEAMTextFrame(key);
       break;
     case HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID :
       handleGAMTextFrame(key);
       break;
     case HOTTV4_GPS_SENSOR_TEXT_ID :
       handleGPSTextFrame(key);
       break;
     default :
       hottV4HandleDefaultTextFrame(key);
       break;
   }  
}

// =================================================================================
// Entry point to send HoTTv4 capable data frames according to the
// requested module.
// =================================================================================
void hottV4SendTelemetry() {
  static enum _hottV4_state {
    IDLE,
    BINARY,
    TEXT,
  } hottV4_state = IDLE;
  
  if (hottV4Serial.available() > 1) {
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t c = hottV4Serial.read();

      if (hottV4_state == IDLE) {
        switch (c) {
          case HOTTV4_BINARY_REQUEST:
            hottV4_state = BINARY;
            break;
          case HOTTV4_TEXT_REQUEST:
            hottV4_state = TEXT;
            break;
          default:
            hottV4_state = IDLE;
        }
      } else if (hottV4_state == BINARY ) {
        // output binary packets
        switch (c) {
          #ifdef HOTTV4_EAM
          
          case HOTTV4_ELECTRICAL_AIR_SENSOR_ID:
            hottV4SendEAM();
            hottV4_state = IDLE;
            break;
          #endif
          
          #ifdef HOTTV4_GAM
          case HOTTV4_GENERAL_AIR_SENSOR_ID:
            hottV4SendGAM();
            hottV4_state = IDLE;
            break;
          #endif
          
          #ifdef HOTTV4_GPS
          case HOTTV4_GPS_SENSOR_ID:
            hottV4SendGPS();
            hottV4_state = IDLE;
            break;
          #endif
          
          #ifdef HOTTV4_VARIO
          case HOTTV4_VARIO_SENSOR_ID:
            break;
          #endif
          
          default:
            hottV4_state = IDLE;
        }
      } else if (hottV4_state == TEXT) {
        // output of text pages

        byte id_sensor = (c >> 4); // High-Nibble = Sensor
        byte id_key = c & 0x0f;    // Low-Nibble = Key

        hottV4HandleTextFrame(id_sensor,id_key);

        hottV4_state = IDLE;
      }
    }
  }
}
