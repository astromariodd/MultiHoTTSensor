
#define REQUEST_DATA_DELAY 300 // millisekunden bis zum nächsten Request
#define INPUT_BUFFER_SIZE 128  // Eingangspuffer der von der Multiwii kommend gefüllt wird

// from http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
// http://www.multiwii.com/forum/viewtopic.php?f=8&t=1516


#define MSP_IDENT     100   //out message         multitype + multiwii 
#define MSP_STATUS    101   //out message         cycletime & errors_count & sensor present & box activation 
#define MSP_RAW_IMU   102   //out message         acc, gyro, mag
#define MSP_SERVO     103   //out message         8 x Servos
#define MSP_MOTOR     104   //out message         8 x Motor
#define MSP_RC        105   //out message         ROLL/PITCH/YAW/THROTTLE/AUX1/AUX2/AUX3AUX4
#define MSP_RAW_GPS   106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS  107   //out message         distance home, direction home
#define MSP_ATTITUDE  108   //out message         2 angles 1 heading 
#define MSP_ALTITUDE  109   //out message         altitude in cm, vario cm/s 
#define MSP_ANALOG    110   //out message         vbat 1/10 volt , powermetersum, rssi, ampere 
#define MSP_RC_TUNING 111   //out message         7 x uint8_t; RC_RATE/RC_EXPO/RollPitchRate/YawRate/DynThrPID/Throttle_MID/Throttle_EXPO; range [0;100] 
#define MSP_PID       112   //out message         
#define MSP_BOX       113   //out message         
#define MSP_MISC      114   //out message 
#define MSP_BOXNAMES  116   //out message
#define MSP_BOXIDS    119   //out message         CHECKBOXITEMS x UINT 8, See enum MultiWii.cpp (0: ARM, 1 ANGLE, 2 HORIZON, …) 
#define MSP_PIDNAMES  117   //out message         string
#define MSP_SET_PID       202 //in message !!       30 x uint8, je 3x ROLL / PITCH / YAW / ALT / POS / POSR / NAVR / LEVEL /MAG / VEL
#define MSP_SET_RC_TUNING 204 // in message!!
#define MSP_EEPROM_WRITE  250 // in Message!!
//#define MSP_HEADING   125   //out message         headings and MAG configuration 

#define BOXARM     0
#define BOXANGLE   1
#define BOXHORIZON 2
#define BOXBARO    3
#define BOXVARIO   4
#define BOXMAG     5

