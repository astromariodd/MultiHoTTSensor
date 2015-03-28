// =================================================================================
/**
 * MultiHoTT-Module is a stand alone Arduino based Application that acts as a bridge between
 * MultiWii FlightController and HoTTv4 capable devices to transmit telemetry information. 
 *
 * Credits to Oliver Bayer & Carsten Giesen, 07/2012
 *  
 * Version with Baro BMP180 and voltage measuring
 * Mario Weidner 12/2014..01/2015
 */
// =================================================================================
#include "MultiHoTTSensor.h"
#include <avr/io.h> 
#include "Lipo.h"
#include "Baro.h"
#include <Wire.h>
#include <Arduino.h>

Lipo Lipo;               // Objektinstanz für Spannungsmessung
Barometer Baro;          // Objektinstanz für Luftdruckmessung

// =================================================================================
void setup() {
  pinMode(LED, OUTPUT);       // LED-PIN als Ausgang  
  Serial.begin(SERIAL_SPEED); // Debugging und Verbindung zur MultiWii
  analogReference(DEFAULT);   // Referenzspannung 

  #ifdef READ_LIPO
  Lipo.init();                // Initialisierung der Spannungsmessung
  #endif
  
  #ifdef READ_BARO
  Baro.init();                // u.a. Kalibrierung des Barometers
  #endif
  
  hottV4Setup();
  
  #ifdef READ_MWC
  delay(1000);               // warten bis die Multiwii hochgefahren ist
  multiwiiSetup();
  #endif
  

}

// =================================================================================
// Blinken der LED mit "Aussetzer" beim vierten mal 
// =================================================================================
static void blink() {
  static uint8_t blink = LOW;
  static uint8_t cnt = 0;

  if (cnt == 3) {
    cnt = 0;
  } else {
    digitalWrite(LED, blink);
    blink = !blink;
    cnt++;
  }
}

// =================================================================================
// MAIN LOOP
// =================================================================================
void loop() {
  static uint32_t last = 0;     // letzte Sensorabfrage, ms seit dem Einschalten
  static uint8_t state = 0;     // welchen Sensor als nächstes abfragen

  uint32_t now = millis();      // alle 200 ms einen der Sensoren abfragen

  if ((now - last) > 200) {
    last = now;
    
    /** Be alive blink */
    blink();

    switch (state) {
      case 0:
        #ifdef READ_LIPO
        Lipo.ReadVoltages();    // Spannungsmessung
        #endif
        state++;
        break;

      case 1:
        #ifdef READ_BARO
        Baro.ReadAltitude();    // Baro per I2C auslesen
        #endif
        state++;
        break;

     default:
        state = 0;
    }
  }

  #ifdef READ_MWC
  multiWiiRequestData();      // Daten von der MultiWii holen
  #endif
  
  hottV4SendTelemetry();      // auf Telemetrie-Requests antworten per HoTTv4 
}
