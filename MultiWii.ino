#include "MultiWii.h"
#include "MultiHoTTSensor.h"
/**
 * Requests via serial interface data from MultWii (>= 2.1) and stores requested
 * data in MultiHoTTModule struct. Requests at least REQUEST_DATA_DELAY milliseconds 
 * a new data frame. Frames are requested in Round Robin.
 */

#define ARMED (MultiHoTTModule.flags & 0x01)

static uint8_t inBuffer[INPUT_BUFFER_SIZE];
static uint8_t oBuffer[30];

static uint8_t indRX;

struct {
  uint8_t cmdList[10];
  uint8_t index;
} mspSchedule;

// =================================================================================
void mwSetSchedule(uint8_t status) {
  // Status 
  // 0 : not armed 
  // 1 : armed
  // 2 :
  mspSchedule.index = 0;  
  switch (status) {
    case 0: { // not armed
      mspSchedule.cmdList[0] = MSP_ANALOG;
      mspSchedule.cmdList[++mspSchedule.index] = MSP_STATUS; 
      mspSchedule.cmdList[++mspSchedule.index] = MSP_PID;
      mspSchedule.cmdList[++mspSchedule.index] = MSP_RC_TUNING;
      mspSchedule.cmdList[++mspSchedule.index] = MSP_ATTITUDE;   
      break;
    }
    case 1: { // armed
      // keine PID, keine RCRates
      mspSchedule.cmdList[0] = MSP_ANALOG;
      #ifdef MWC_HASBARO
        mspSchedule.cmdList[++mspSchedule.index] = MSP_ALTITUDE;
      #endif
      mspSchedule.cmdList[++mspSchedule.index] = MSP_ATTITUDE;
      mspSchedule.cmdList[++mspSchedule.index] = MSP_STATUS; 
      #ifdef HOTTV4_GPS
        mspSchedule.cmdList[++mspSchedule.index] = MSP_RAW_GPS; 
        mspSchedule.cmdList[++mspSchedule.index] = MSP_COMP_GPS;
      #endif   
      break;
    }
    
  }  
}

// =================================================================================
// Auswahl der nächsten Multiwii-Abfrage
// =================================================================================
uint8_t mwGetNextScheduledCommand() {
  uint8_t nextcmd;
  //static uint8_t index = 0;   
  // TODO: kontextsensitive Auswahl  
        
  //nextcmd = schedule[index++ % sizeof(schedule)]; // dass index überläuft ist nicht schlimm
  nextcmd = mspSchedule.cmdList[mspSchedule.index];
  
  if (mspSchedule.index==0) {
    // neu laden und index setzen
    if (ARMED) {
      mwSetSchedule(1);
    } else {
      mwSetSchedule(0);
    }  
  } else {
    mspSchedule.index--;
  }  
  
  // steht dem Request etwas entgegen?    
  if ( (nextcmd==MSP_PID) && MultiHoTTModule.pidvalueschanged) 
    nextcmd = mwGetNextScheduledCommand();
    
  // steht dem Request etwas entgegen?    
  if ( (nextcmd==MSP_RC_TUNING) && MultiHoTTModule.rcvalueschanged) 
    nextcmd = mwGetNextScheduledCommand();    
    
  return nextcmd;   
}  

// =================================================================================
// liest 8 bit und erhöht den Index des Puffers um 1
// =================================================================================
uint8_t read8()  {
  return inBuffer[indRX++]&0xff;
}

// =================================================================================
// liest 2 x 8bit und erhöht damit indirekt auch den Index um 2
// =================================================================================
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}

// =================================================================================
// liest 32 bit = 2 x 16 bit = 4 x 8 bit und erhöht damit auch den Index um 4
// =================================================================================
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}


void mwEvaluateMSP_BOXIDS() {
  for (uint8_t i=0;i<6;i++) {
    MultiHoTTModule.boxitems[i] = read8();
                           //  0, //"ARM;"
                           //  1, //"ANGLE;"
                           //  2, //"HORIZON;"
                           //  3, //"BARO;"
                           //  4, //"VARIO;"
                           //  5, //"MAG;"
  }  
}  

uint8_t getValueOfBoxID(uint8_t BoxID) {
                           //  0, //"ARM;"
                           //  1, //"ANGLE;"
                           //  2, //"HORIZON;"
                           //  3, //"BARO;"
                           //  4, //"VARIO;"
                           //  5, //"MAG;"
  for (uint8_t i=0;i<6;i++) {
    if (BoxID == MultiHoTTModule.boxitems[i]) {
      return ( 1<<i ); // Bit-Value
    }  
  }
  return 0;  
}  

// =================================================================================
// liest die PID-Werte aus der MultiWii aus
// =================================================================================
static void mwEvaluateMSP_PID() {
  if (MultiHoTTModule.pidvalueschanged==false) { 
    // wenn wir geänderte Werte hätten, dürfen wird die nicht mit alten Werten überschreiben
    for (uint8_t i=0;i<10;i++) {
      MultiHoTTModule.pidvalues[3*i]   = read8();
      MultiHoTTModule.pidvalues[3*i+1] = read8();
      MultiHoTTModule.pidvalues[3*i+2] = read8();     
    }
  } 
}

// =================================================================================
// sendet die PID-Werte an die MultiWii
// =================================================================================
static void mwRequestMSP_SET_PID() {
  //oBuffer mit den PID füllen
  for (uint8_t i=0;i<30;i++) {
    oBuffer[i] = MultiHoTTModule.pidvalues[i];
  }
}

// =================================================================================
// sendet die RC-Tuning Werte an die MultiWii
// =================================================================================
static void mwRequestMSP_SET_RC_TUNING() {
  //oBuffer mit den rcrates füllen
  for (uint8_t i=0;i<7;i++) {
    oBuffer[i] = MultiHoTTModule.rcrates[i];
  }
}  

// =================================================================================
// Reads MSP_STATUS.
// =================================================================================
static void mwEvaluateMSP_STATUS() {
  /*
   cycleTime 	    UINT 16
   i2c_errors_count UINT 16
   sensor 	    UINT 16
   flag 	    UINT 32
   global_conf.currentSet 	UINT 8
  */
  MultiHoTTModule.cycletime = read16(); 
  MultiHoTTModule.i2cerrors = read16(); 
  MultiHoTTModule.sensors = read16();  
  MultiHoTTModule.flags = read32();
  MultiHoTTModule.currentprofile = read8();  
}


// =================================================================================
// Reads rcRates
// =================================================================================
static void mwEvaluateMSP_RC_TUNING() {
  if (MultiHoTTModule.rcvalueschanged==false) { 
    // wenn wir geänderte Werte hätten, dürfen wird die nicht mit alten Werten überschreiben
    for (uint8_t i=0;i<7;i++) {
      MultiHoTTModule.rcrates[i] = read8();
    }
  }
}

// =================================================================================
// Reads VBAT from given MSP data frame and stores it for later usage.
// =================================================================================
static void mwEvaluateMSP_BAT() {
  MultiHoTTModule.vbat2            = read8();   //data[0];
  MultiHoTTModule.intPowerMeterSum = read16();  //data[1]+(data[2]*0x100);
}

// =================================================================================
// Reads altitude from MSP data frame and stores it for later usage.
// =================================================================================
static void mwEvaluateMSP_ALTITUDE() {
  MultiHoTTModule.altitude = read32();  //data[0]+(data[1]*0x100)+(data[2]*0x10000)+(data[4]*0x1000000);
  MultiHoTTModule.m2s = read16();
}

// =================================================================================
// Reads attitude from MSP data frame and stores it for later usage.
// =================================================================================
static void mwEvaluateMSP_ATTITUDE() {
  MultiHoTTModule.attitudeAngles1 = read16();  //data[0]+(data[1]*0x100); Range [-1800;1800] (unit: 1/10 degree) 
  MultiHoTTModule.attitudeAngles2 = read16();  //data[2]+(data[3]*0x100);
  MultiHoTTModule.attitudeHeading = read16();  //data[4]+(data[5]*0x100); Range [-180;180] 
}

// =================================================================================
// Reads RAW_GPS from MSP data frame and stores it for later usage.
// =================================================================================
#ifdef HOTTV4_GPS
static void mwEvaluateMSP_RAW_GPS() {
    MultiHoTTModule.GPS_fix       = read8();   //data[0];
    MultiHoTTModule.GPS_numSat    = read8();   //data[1];

    MultiHoTTModule.GPS_latitude  = read32();  //data[2]+(data[3]*0x100)+(data[4]*0x10000)+(data[5]*0x1000000);
    MultiHoTTModule.GPS_longitude = read32();  //data[6]+(data[7]*0x100)+(data[8]*0x10000)+(data[9]*0x1000000);
    MultiHoTTModule.GPS_altitude  = read16();  //data[10]+(data[11]*0x100);
    MultiHoTTModule.GPS_speed     = read16();  //data[12]+(data[13]*0x100);
    MultiHoTTModule.GPS_ground_course = read16(); // unit: degree*10 
  }
  
// =================================================================================
// Reads COMP_GPS from MSP data frame and stores it for later usage.
// =================================================================================
  static void mwEvaluateMSP_COMP_GPS() { 
    MultiHoTTModule.GPS_distanceToHome  = read16();  //data[0]+(data[1]*0x100);
    MultiHoTTModule.GPS_directionToHome = read16();  //data[2]+(data[3]*0x100);
    MultiHoTTModule.GPS_update          = read8();   //data[4];
  }
#endif

// =================================================================================
// Reads HEADING from MSP data frame and stores it for later usage.
// =================================================================================
//static void mwEvaluateMSP_HEADING() {
//  MultiHoTTModule.magMode          = read8();   //data[0];
//  MultiHoTTModule.heading          = read16();  //data[1]+(data[2]*0x100);
//  MultiHoTTModule.magHold          = read16();  //data[3]+(data[4]*0x100);
//  MultiHoTTModule.headFreeModeHold = read16();  //data[5]+(data[6]*0x100);
//}

// =================================================================================
// Evaluates valid MultiWii Serial Protocol message in inBuffer and stores 
// needed data for later transmission via HoTT. 
// =================================================================================
static void mwEvaluateMSPResponse(uint8_t cmd) {
  switch(cmd) {
    case MSP_STATUS:
      mwEvaluateMSP_STATUS();
      break;
    case MSP_ANALOG:
      mwEvaluateMSP_BAT();
      break;
    case MSP_ATTITUDE:
      mwEvaluateMSP_ATTITUDE();
      break;
    case MSP_ALTITUDE:
      mwEvaluateMSP_ALTITUDE();
      break;
    #ifdef HOTTV4_GPS
      case MSP_RAW_GPS:
        mwEvaluateMSP_RAW_GPS();
        break;
      case MSP_COMP_GPS:
        mwEvaluateMSP_COMP_GPS();
        break;
    #endif
    case MSP_IDENT:
      //mwEvaluateMSP_IDENT();
      break;
    case MSP_MISC:
      //mwEvaluateMSP_MISC();
    break; 
    case MSP_PID:
      mwEvaluateMSP_PID();
      break; 
    //case MSP_BOXNAMES:
      //mwEvaluateMSP_BOXNAMES();
      //break;
    case MSP_BOXIDS:
      mwEvaluateMSP_BOXIDS();
      break;  
    case MSP_RC_TUNING:  
      mwEvaluateMSP_RC_TUNING();
      break;
  }
}

// =================================================================================
// Reads a MultWii command from serial interface and stores result in inBuffer.
// then mwEvaluateMSPResponse(cmd)
// =================================================================================
void mwEvaluateResponse() {
  uint8_t c = 0;
  uint8_t cmd = 0;
  uint8_t checksum = 0;
  uint8_t payloadSize = 0;
  uint8_t offset = 0;
  
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;
  
  while(Serial.available()) {
    c = Serial.read();

    if (IDLE == c_state) {
      c_state = ('$' == c) ? HEADER_START : IDLE;
    } 
    else if (HEADER_START == c_state) {
      c_state = ('M' == c) ? HEADER_M : IDLE;
    } 
    else if (HEADER_M == c_state) {
      c_state = ('>' == c) ? HEADER_ARROW : IDLE;
    } 
    else if (HEADER_ARROW == c_state) {
      if (c > INPUT_BUFFER_SIZE) {  // now we are expecting the payload size
        c_state = IDLE;
        continue;
      }
      checksum = 0;
      offset = 0;
      indRX = 0;
      memset(inBuffer, 0, sizeof(inBuffer));
      
      payloadSize = c;
      checksum ^= c;
      
      c_state = HEADER_SIZE;    
    } 
    else if (HEADER_SIZE == c_state) {
      cmd = c;
      checksum ^= c;      
      c_state = HEADER_CMD;
    } 
    else if (HEADER_CMD == c_state && offset < payloadSize) {
      checksum ^= c;
      inBuffer[offset++] = c; 
    } 
    else if (HEADER_CMD == c_state && offset >= payloadSize) { 
      if (checksum == c) {
        mwEvaluateMSPResponse(cmd);
      }
      c_state = IDLE;
    }
  }
}


// =================================================================================
// Sends a request to MultiWii with given cmd
// $M<[data length][code][data][checksum]
// =================================================================================
static void mwRequestData(uint8_t cmd) {
  char cmdBuffer[6] = { '$', 'M', '<', 0x0, cmd, cmd }; 

  // wenn Armed, dann keine PID abfragen
  if ( !(ARMED && (cmd == MSP_PID)) ) {

    //wenn nichts anliegt, dann senden
    if (0 == Serial.available()) {
      for (uint8_t i = 0; i < 6; i++) {
        Serial.write(cmdBuffer[i]);
      }
    }
  } 
}

// =================================================================================
// sendet einen dataframe an die MultiWii
// =================================================================================
static void mwSendData(uint8_t cmd) {
  uint8_t size;
  uint8_t crc;

  switch (cmd) {
    case MSP_SET_PID:
      mwRequestMSP_SET_PID(); // oBuffer füllen
      size = 30;
      break;
    case MSP_SET_RC_TUNING:
      mwRequestMSP_SET_RC_TUNING(); // oBuffer füllen
      size = 7;   
      break;  
  }
  char cmdBuffer[5] = { '$', 'M', '<', size, cmd}; 
  crc = 0;
  crc ^= size;
  crc ^= cmd;
  
  //wenn nichts anliegt, dann senden
  if (Serial.available()==0) {
    for (uint8_t i = 0; i < 5; i++) {
      Serial.write(cmdBuffer[i]);
    }
    for (uint8_t i = 0; i < size;i++) {
      Serial.write(oBuffer[i]);
      crc ^= oBuffer[i];
    }  
    Serial.write((byte) crc);
  }
  // gleich noch einen Request hinterher
  // multiwii soll die Daten auch im Eprom abspeichern
  mwRequestData(MSP_EEPROM_WRITE);
}


// =================================================================================
// Main method of MultiWii integration.
// alle  REQUEST_DATA_DELAY Millisekunden absetzen eines Request aus schedule
// oder einlesen einer Antwort
// =================================================================================
void multiWiiRequestData() {
  static uint32_t previousMillis = 0;
  
  if ((millis() - previousMillis) > REQUEST_DATA_DELAY) {
    previousMillis = millis();
    
    if (Serial.available() == 0) { 
      
      // Daten senden wenn nicht armed     
      if ( (MultiHoTTModule.setvalues==true) && !ARMED) {
        if (MultiHoTTModule.pidvalueschanged==true) {
          mwSendData(MSP_SET_PID);
          MultiHoTTModule.pidvalueschanged=false;            
        } else if (MultiHoTTModule.rcvalueschanged==true) {
          mwSendData(MSP_SET_RC_TUNING);
          MultiHoTTModule.rcvalueschanged=false;
        }
        MultiHoTTModule.setvalues=false;
      } 
      else { 
        // Daten anfordern        
        mwRequestData(mwGetNextScheduledCommand());
      }  
    } else {
      // Daten liegen an und müssen verarbeitet werden
      mwEvaluateResponse();
    }
  }    
}

// =================================================================================
// wird nur beim Einschalten durchlaufen
// =================================================================================
void multiwiiSetup() {
  MultiHoTTModule.pidvalueschanged==false;
  MultiHoTTModule.rcvalueschanged==false;  
  MultiHoTTModule.setvalues==false; 
 
 // Abfrage Boxnames etc. 
 mwRequestData(MSP_BOXIDS);
}  
