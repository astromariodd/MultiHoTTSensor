#ifndef MultiHoTTSensor_h
#define MultiHoTTSensor_h

#include "Arduino.h"

// liest die Zellspannungen über den Arduino aus
#define READ_LIPO

// liest den Baro am Arduino aus
//#define READ_BARO

// MultiWii soll ausgelesen werden
#define READ_MWC

// die MultiWii hat einen Baro drin
#define MWC_HASBARO

// als welcher HoTT-Sensor geben wir uns aus (mehrere möglich)
//#define HOTTV4_GPS
//#define HOTTV4_VARIO
#define HOTTV4_EAM
//#define HOTTV4_GAM 

#define LED 13

// Übertragungsgeschwindigkeit zur MultiWii
//#define SERIAL_SPEED 115200
#define SERIAL_SPEED 57600   // Pegelkonverter kann nicht mehr als...

#define HOTTV4_RXTX 3  // beim Board mit dem Baro 5 (3/4 defekt), sonst 3
#define HOTTV4_TX_DELAY 1000

// Maximum in MWii 2.3 sind 22
#define CHECKBOXITEMS 22


// Maximum in MWii 2.3 sind 10
#define PIDITEMS 10

// allg. Datenspeicher der Telemetriedaten
struct {
  uint16_t sensors;        // BARO<<1|MAG<<2|GPS<<3|SONAR<<4 
  
  uint32_t flags;          // a bit variable to indicate which BOX are active, the bit position depends on the BOX which are configured 
                           // 0 BOXARM
                           // 1 BOXANGLE
                           // 2 BOXHORIZON
                           // 3 BOXBARO
                           // 4 BOXMAG
                           // 5 BOXHEADFREE
                           // 6 BOXHEADADJ
                           // 
 
                           //const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
                           //  0, //"ARM;"
                           //  1, //"ANGLE;"
                           //  2, //"HORIZON;"
                           //  3, //"BARO;"
                           //  4, //"VARIO;"
                           //  5, //"MAG;"
                           //  6, //"HEADFREE;"
                           //  7, //"HEADADJ;"  
                           //  8, //"CAMSTAB;"
                           //  9, //"CAMTRIG;"
                           //  10, //"GPS HOME;"
                           //  11, //"GPS HOLD;"
                           //  12, //"PASSTHRU;"
                           //  13, //"BEEPER;"
                           //  14, //"LEDMAX;"
                           //  15, //"LEDLOW;"
                           //  16, //"LLIGHTS;"
                           //  17, //"CALIB;"
                           //  18, //"GOVERNOR;"
                           //  19, //"OSD_SWITCH;"
                           //  20, //"MISSION;"
                           //  21, //"LAND;" 
  uint8_t currentprofile;  
  uint16_t cycletime;
  uint16_t i2cerrors;  
                           
  uint16_t vbat1;          // VBat MultiWiiModule
  uint8_t  vbat2;          // VBat MultiWii FlightControll
  uint16_t intPowerMeterSum; 
  uint16_t rssi;
  uint16_t amperage;
  
  uint8_t cellcount;       // Anzahl Zellen
  uint16_t cell1;          // Zellspannungen,
  uint16_t cell2;
  uint16_t cell3;
  uint16_t cell4;
  
  uint8_t temp;            // Temperatur
  
  int16_t attitudeAngles1;
  int16_t attitudeAngles2;
  int16_t attitudeHeading;
  
  int16_t altitude0;       // Bezugshöhe
  int16_t altitude;        // akt. Höhe
  uint32_t lastbaroread;   // Zeitpunkt als m2s das letzte Mal berechnet wurde
  int16_t lastaltitude;    // letzte Höhe als m2s das letzte Mal berechnet wurde
  int16_t m2s;             // Steigrate
  
  uint16_t current;        // Strom

  uint8_t GPS_fix;         // GPS-Daten
  uint8_t GPS_numSat;
  uint32_t GPS_latitude;
  uint32_t GPS_longitude;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_ground_course;

  //MSP_COMP_GPS
  uint16_t GPS_distanceToHome;
  uint16_t GPS_directionToHome;
  uint8_t GPS_update;    

  //MSP_HEADING
  uint8_t magMode;
  uint16_t heading;
  uint16_t magHold;
  uint16_t headFreeModeHold;
  
  uint8_t pidvalues[3*PIDITEMS];
  uint8_t rcrates[7];              // RC_RATE/RC_EXPO/RollPitchRate/YawRate/DynThrPID/Throttle_MID/Throttle_EXPO
  uint8_t pidvalueschanged;
  uint8_t rcvalueschanged;
  uint8_t setvalues; 
  
  uint8_t boxitems[CHECKBOXITEMS];
  
  //uint8_t crc; 
  uint8_t alarm;
} MultiHoTTModule;

struct {
  uint8_t version;
  uint8_t alarmVBat;
  uint8_t alarmTemp1;
  int16_t maxAltitude;
} MultiHoTTModuleSettings;

#endif
