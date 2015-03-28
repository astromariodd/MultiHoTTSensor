#ifndef HoTTv4_h
#define HoTTv4_h

/** ###### HoTT module specifications ###### */

//#define DEBUG

#define HOTTV4_VARIO_SENSOR_ID               0x89
#define HOTTV4_GPS_SENSOR_ID                 0x8A   // GPS Sensor ID
#define HOTTV4_ESC_SENSOR_ID                 0x8C
#define HOTTV4_GENERAL_AIR_SENSOR_ID         0x8D
#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID      0x8E   // Electric Air Sensor ID

#define HOTTV4_VARIO_SENSOR_TEXT_ID          0x90   // Vario Module Text ID
#define HOTTV4_GPS_SENSOR_TEXT_ID            0xA0   // GPS Module Text ID
#define HOTTV4_ESC_SENSOR_TEXT_ID            0xC0
#define HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID    0xD0
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID 0xE0   // Electric Air Module Text ID

#define HOTTV4_BINARY_REQUEST                0x80
#define HOTTV4_TEXT_REQUEST                  0x7F

#define HOTTV4_BUTTON_PREV                   0x07
#define HOTTV4_BUTTON_SET                    0x09
#define HOTTV4_BUTTON_DEC                    0x0B
#define HOTTV4_BUTTON_INC                    0x0D
#define HOTTV4_BUTTON_NEXT                   0x0E
#define HOTTV4_BUTTON_NIL                    0x0F

#define PAGE_STATUS                          1
#define PAGE_SENSOR                          2
#define PAGE_ALARM                           3
#define PAGE_PID                             4
#define PAGE_RCTUNING                        5

const uint8_t kHoTTv4BinaryPacketSize = 45; 
const uint8_t kHoTTv4TextPacketSize = 173;

typedef enum {
  HoTTv4NotificationErrorCalibration     = 0x01,
  HoTTv4NotificationErrorReceiver        = 0x02,
  HoTTv4NotificationErrorDataBus         = 0x03,
  HoTTv4NotificationErrorNavigation      = 0x04,
  HoTTv4NotificationErrorError           = 0x05,
  HoTTv4NotificationErrorCompass         = 0x06,
  HoTTv4NotificationErrorSensor          = 0x07,
  HoTTv4NotificationErrorGPS             = 0x08,
  HoTTv4NotificationErrorMotor           = 0x09,
  
  HoTTv4NotificationMaxTemperature       = 0x0A,
  HoTTv4NotificationAltitudeReached      = 0x0B,
  HoTTv4NotificationWaypointReached      = 0x0C,
  HoTTv4NotificationNextWaypoint         = 0x0D,
  HoTTv4NotificationLanding              = 0x0E,
  HoTTv4NotificationGPSFix               = 0x0F,
  HoTTv4NotificationUndervoltage         = 0x10,
  HoTTv4NotificationGPSHold              = 0x11,
  HoTTv4NotificationGPSHome              = 0x12,
  HoTTv4NotificationGPSOff               = 0x13,
  HoTTv4NotificationBeep                 = 0x14,
  HoTTv4NotificationMicrocopter          = 0x15,
  HoTTv4NotificationCapacity             = 0x16,
  HoTTv4NotificationCareFreeOff          = 0x17,
  HoTTv4NotificationCalibrating          = 0x18,
  HoTTv4NotificationMaxRange             = 0x19,
  HoTTv4NotificationMaxAltitude          = 0x1A,
  
  HoTTv4Notification20Meter              = 0x25,
  HoTTv4NotificationMicrocopterOff       = 0x26,
  HoTTv4NotificationAltitudeOn           = 0x27,
  HoTTv4NotificationAltitudeOff          = 0x28,
  HoTTv4Notification100Meter             = 0x29,
  HoTTv4NotificationCareFreeOn           = 0x2E,
  HoTTv4NotificationDown                 = 0x2F,
  HoTTv4NotificationUp                   = 0x30,
  HoTTv4NotificationHold                 = 0x31,
  HoTTv4NotificationGPSOn                = 0x32,
  HoTTv4NotificationFollowing            = 0x33,
  HoTTv4NotificationStarting             = 0x34,
  HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;


//  HoTTV4 GENERAL AIR MODULE ********************************************
//                                                   
struct {
  byte startByte;     //#01 start byte constant value 0x7c
  byte sensorID; //#02 EAM sensort id. constat value 0x8d=GENRAL AIR MODULE
  byte alarmTone; //#03 1=A 2=B ... 0x1a=Z 0 = no alarm
                      /* VOICE OR BIP WARNINGS
                      Alarme sonore A.. Z, octet correspondant 1 à 26
                      0x00 00 0 No alarm
                      0x01 01 A
                      0x02 02 B Negative Difference 2 B
                      0x03 03 C Negative Difference 1 C
                      0x04 04 D
                      0x05 05 E
                      0x06 06 F Min. Sensor 1 temp. F
                      0x07 07 G Min. Sensor 2 temp. G
                      0x08 08 H Max. Sensor 1 temp. H
                      0x09 09 I Max. Sensor 2 temp. I
                      0xA 10 J Max. Sens. 1 voltage J
                      0xB 11 K Max. Sens. 2 voltage K
                      0xC 12 L
                      0xD 13 M Positive Difference 2 M
                      0xE 14 N Positive Difference 1 N
                      0xF 15 O Min. Altitude O
                      0x10 16 P Min. Power Voltage P // We use this one for Battery Warning
                      0x11 17 Q Min. Cell voltage Q
                      0x12 18 R Min. Sens. 1 voltage R
                      0x13 19 S Min. Sens. 2 voltage S
                      0x14 20 T Minimum RPM T
                      0x15 21 U
                      0x16 22 V Max. used capacity V
                      0x17 23 W Max. Current W
                      0x18 24 X Max. Power Voltage X
                      0x19 25 Y Maximum RPM Y
                      0x1A 26 Z Max. Altitude Z
                      */
  byte sensorTextID;     //#04 constant value 0xd0
  byte alarmInverse1; //#05 alarm bitmask. Value is displayed inverted
                      //Bit# Alarm field
                      // 0 all cell voltage
                      // 1 Battery 1
                      // 2 Battery 2
                      // 3 Temperature 1
                      // 4 Temperature 2
                      // 5 Fuel
                      // 6 mAh
                      // 7 Altitude
  byte alarm_invers2; //#06 alarm bitmask. Value is displayed inverted
                      //Bit# Alarm Field
                      // 0 main power current
                      // 1 main power voltage
                      // 2 Altitude
                      // 3 m/s
                      // 4 m/3s
                      // 5 unknown
                      // 6 unknown
                      // 7 "ON" sign/text msg active
  byte cell[6];       //#7 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
                      //#8 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
                      //#9 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
                      //#10 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
                      //#11 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
                      //#12 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
  uint16_t battery1;  //#13 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                      //#14 MSB
  uint16_t battery2;  //#15 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                      //#16 MSB
  byte temperature1;  //#17 Temperature 1. Offset of 20. a value of 20 = 0°C
  byte temperature2;  //#18 Temperature 2. Offset of 20. a value of 20 = 0°C
  byte fuel_procent;  //#19 Fuel capacity in %. Values 0--100
                      //graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
  uint16_t fuel_ml;   //#20 LSB Fuel in ml scale. Full = 65535!
                      //#21 MSB
  uint16_t rpm;       //#22 RPM in 10 RPM steps. 300 = 3000rpm
                      //#23 MSB
  uint16_t altitude;  //#24 altitude in meters. offset of 500, 500 = 0m
                      //#25 MSB
  uint16_t climbrate; //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                      //#27 MSB
  byte climbrate3s;   //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint16_t current;   //#29 current in 0.1A steps 100 == 10,0A
                      //#30 MSB current display only goes up to 99.9 A (continuous)
  uint16_t main_voltage; //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                      //#32 MSB (Appears in GAM display right as alternate display.)
  uint16_t batt_cap;  //#33 LSB used battery capacity in 10mAh steps
                      //#34 MSB
  uint16_t speed;     //#35 LSB (air?) speed in km/h(?) we are using ground speed here per default
                      //#36 MSB speed
  byte min_cell_volt; //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  byte min_cell_volt_num; //#38 number of the cell with the lowest voltage
  uint16_t rpm2;      //#39 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
                      //#40 MSB
  byte general_error_number; //#41 General Error Number (Voice Error == 12) TODO: more documentation
  byte pressure;       //#42 High pressure up to 16bar. 0,1bar scale. 20 == 2.0 bar
                       // 1 bar = 10 hoch 5 Pa
  byte version;        //#43 version number (Bytes 35 .43 new but not yet in the record in the display!)
  byte endByte;        //#44 stop byte 0x7D
  byte parity;         //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
} HoTTV4GeneralAirModule; 

//  HoTTV4ElectricAirModule EAM ********************************************
//                                                   
struct {
  uint8_t startByte;              // Byte  1:
  uint8_t sensorID;               // Byte  2:
  uint8_t alarmTone;              // Byte  3: Alarm
  uint8_t sensorTextID;           // Byte  4:
  uint8_t alarmInverse1;          // Byte  5:
  uint8_t alarmInverse2;          // Byte  6:

  uint8_t cell1L;                 // Byte  7: Low Voltage Cell 1 in 0,02 V steps
  uint8_t cell2L;                 // Byte  8: Low Voltage Cell 2 in 0,02 V steps
  uint8_t cell3L;                 // Byte  9: Low Voltage Cell 3 in 0,02 V steps
  uint8_t cell4L;                 // Byte 10: Low Voltage Cell 4 in 0,02 V steps
  uint8_t cell5L;                 // Byte 11: Low Voltage Cell 5 in 0,02 V steps
  uint8_t cell6L;                 // Byte 12: Low Voltage Cell 6 in 0,02 V steps
  uint8_t cell7L;                 // Byte 13: Low Voltage Cell 7 in 0,02 V steps
  uint8_t cell1H;                 // Byte 14: High Voltage Cell 1 in 0.02 V steps
  uint8_t cell2H;  
  uint8_t cell3H;  
  uint8_t cell4H;
  uint8_t cell5H;
  uint8_t cell6H;
  uint8_t cell7H;

  uint8_t battery1Low;           /* Battery 1 LSB/MSB in 100mv steps; 50 == 5V */
  uint8_t battery1High;          /* Battery 1 LSB/MSB in 100mv steps; 50 == 5V */
  uint8_t battery2Low;           /* Battery 2 LSB/MSB in 100mv steps; 50 == 5V */
  uint8_t battery2High;          /* Battery 2 LSB/MSB in 100mv steps; 50 == 5V */

  uint8_t temp1;                 /* Temp 1; Offset of 20. 20 == 0C */
  uint8_t temp2;                 /* Temp 2; Offset of 20. 20 == 0C */

  uint16_t height;               /* Height. Offset -500. 500 == 0 */
  uint16_t current;              /* 1 = 0.1A */
  
  uint8_t driveVoltageLow;
  uint8_t driveVoltageHigh;
  uint16_t capacity;             /* mAh */
  uint16_t m2s;                  /* Steigrate m2s; 0x48 == 0 */
  uint8_t m3s;                   /* Steigrate m3s; 0x78 == 0 */
  
  uint16_t rpm;                  /* RPM. 10er steps; 300 == 3000rpm */
  
  uint8_t minutes;
  uint8_t seconds;
  
  uint8_t speed;

  uint8_t version;
  uint8_t endByte;
  uint8_t chksum;
} HoTTV4ElectricAirModule;

struct {
  uint8_t startByte;
  uint8_t sensorTextID;
  uint8_t alarm;   //1=A 2=B ...
                  // Q    Min cell voltage sensor 1
                  // R    Min Battery 1 voltage sensor 1
                  // J    Max Battery 1 voltage sensor 1
                  // F    Min temperature sensor 1
                  // H    Max temperature sensor 1
                  // S    Min Battery 2 voltage sensor 2
                  // K    Max Battery 2 voltage sensor 2
                  // G    Min temperature sensor 2
                  // I    Max temperature sensor 2
                  // W    Max current
                  // V    Max capacity mAh
                  // P    Min main power voltage
                  // X    Max main power voltage
                  // O    Min altitude
                  // Z    Max altitude
                  // C    negative difference m/s too high
                  // A    negative difference m/3s too high
                  // N    positive difference m/s too high
                  // L    positive difference m/3s too high
                  // T    Minimum RPM
                  // Y    Maximum RPM
  uint8_t text[8*21];
  uint8_t endByte;
  uint8_t chksum;
} HoTTv4ElectricalAirTextModule;


// HoTTV4GPSModule ****************************************************************
// 
struct {
  uint8_t startByte;               /* Byte 1: 0x7C = Start byte data */
  uint8_t sensorID;                /* Byte 2: 0x8A = GPS Sensor */
  uint8_t alarmTone;               /* Byte 3: 0…= warning beeps */
  uint8_t sensorTextID;            /* Byte 4: 160 0xA0 Sensor ID Neu! */
  
  uint8_t alarmInverse1;           /* Byte 5: 01 inverse status */
  uint8_t alarmInverse2;           /* Byte 6: 00 inverse status status 1 = kein GPS Signal */
  
  uint8_t flightDirection;         /* Byte 7: 119 = Flugricht./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West) */
  uint8_t GPSSpeedLow;             /* Byte 8: 8 = Geschwindigkeit/GPS speed low byte 8km/h */
  uint8_t GPSSpeedHigh;            /* Byte 9: 0 = Geschwindigkeit/GPS speed high byte */
  
  uint8_t LatitudeNS;              /* Byte 10: 000 = N = 48°39’988 */
  uint8_t LatitudeMinLow;          /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
  uint8_t LatitudeMinHigh;         /* Byte 12: 018 18 = 0x12 */
  uint8_t LatitudeSecLow;          /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
  uint8_t LatitudeSecHigh;         /* Byte 14: 016 3 = 0x03 */
 
  uint8_t longitudeEW;             /* Byte 15: 000  = E= 9° 25’9360 */
  uint8_t longitudeMinLow;         /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
  uint8_t longitudeMinHigh;        /* Byte 17: 003 3 = 0x03 */
  uint8_t longitudeSecLow;         /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
  uint8_t longitudeSecHigh;        /* Byte 19: 004 36 = 0x24 */
  
  uint8_t distanceLow;             /* Byte 20: 027 123 = Entfernung/distance low byte 6 = 6 m */
  uint8_t distanceHigh;            /* Byte 21: 036 35 = Entfernung/distance high byte */
  uint8_t altitudeLow;             /* Byte 22: 243 244 = Höhe/Altitude low byte 500 = 0m */
  uint8_t altitudeHigh;            /* Byte 23: 001 1 = Höhe/Altitude high byte */
  uint8_t climbrateLow;            /* Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
  uint8_t climbrateHigh;           /* Byte 25: 117 = High Byte m/s resolution 0.01m */
  uint8_t climbrate3s;             /* Byte 26: climbrate in m/3s resolution, value of 120 = 0 m/3s*/
  uint8_t GPSNumSat;               /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
  uint8_t GPSFixChar;              /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
  uint8_t HomeDirection;           /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
  uint8_t angleXdirection;         /* Byte 30: angle x-direction (1 byte) */
  uint8_t angleYdirection;         /* Byte 31: angle y-direction (1 byte) */
  uint8_t angleZdirection;         /* Byte 32: angle z-direction (1 byte) */
  
//  int8_t gps_time_h;  //#33 UTC time hours
//  int8_t gps_time_m;  //#34 UTC time minutes
//  int8_t gps_time_s;  //#35 UTC time seconds
//  int8_t gps_time_sss;//#36 UTC time milliseconds
//  int8_t msl_altitude_L;  //#37 mean sea level altitude
//  int8_t msl_altitude_H;  //#38
  
  uint8_t gyroXLow;                /* Byte 33: gyro x low byte (2 bytes) */
  uint8_t gyroXHigh;               /* Byte 34: gyro x high byte */
  uint8_t gyroYLow;                /* Byte 35: gyro y low byte (2 bytes) */
  uint8_t gyroYHigh;               /* Byte 36: gyro y high byte */
  uint8_t gyroZLow;                /* Byte 37: gyro z low byte (2 bytes) */
  uint8_t gyroZHigh;               /* Byte 38: gyro z high byte */
  
  uint8_t vibration;               /* Byte 39: vibration (1 bytes) */
  uint8_t Ascii4;                  /* Byte 40: 00 ASCII Free Character [4] appears right to home distance */
  uint8_t Ascii5;                  /* Byte 41: 00 ASCII Free Character [5] appears right to home direction*/
  uint8_t GPS_fix;                 /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
  uint8_t version;                 /* Byte 43: 00 version number */
  uint8_t endByte;                 /* Byte 44: 0x7D Ende byte */
  uint8_t chksum;                  /* Byte 45: Parity Byte */
} HoTTV4GPSModule;

struct {
  uint8_t startByte;
  uint8_t sensorTextID;
  uint8_t alarm;
  uint8_t text[8*21];
  uint8_t endByte;
  uint8_t chksum;
} HoTTv4GPSTextModule;


#endif
