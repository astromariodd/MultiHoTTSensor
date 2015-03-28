// Spannungsmessung über die Spannungsteiler
// R1/R2, R3/R4, R5/R6, R7/R8
//
#include <Arduino.h>
#include "Lipo.h"

// =================================================================================
void Lipo::init(){
   analogReference(DEFAULT); // reference alim. 3.3/5V
   digitalWrite(CELL1_PIN, LOW); 
   pinMode(CELL1_PIN, INPUT);
   
   digitalWrite(CELL2_PIN, LOW); 
   pinMode(CELL2_PIN, INPUT);
   
   digitalWrite(CELL3_PIN, LOW); 
   pinMode(CELL3_PIN, INPUT);
   
   digitalWrite(CELL4_PIN, LOW); 
   pinMode(CELL4_PIN, INPUT); 
  
   cell1 = 0.0; // Zellspannungen
   cell2 = 0.0;
   cell3 = 0.0;
   cell4 = 0.0;  
   
   cellcount = Detect ();
}

// =================================================================================
/** To avoid sensor jitter as found in: 
 * http://forums.adafruit.com/viewtopic.php?f=25&t=11597
 */
static inline void AvoidJitter(uint8_t pin) {
  analogRead(pin);
  delay(25);
}
 

// =================================================================================
// Spannungsmessung per ADC
uint16_t Lipo::Measure(uint8_t pin){
  AvoidJitter(pin);  
  uint16_t val = analogRead(pin); 
  return val;
}

// =================================================================================
float Lipo::GetVoltage_Cell_1 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  Measure(CELL1_PIN)* CELL1_COEF / 3.3 * ReadVcc();
  val = (MV1 + MV2 + MV3) / 3; // filtrage (Durchschnitt aus 3 Messungen)
  MV1 = MV2; // décalage
  MV2 = MV3;
  //Serial.print("Cell 1:");
  //Serial.println(val);   
  return val;
}

// =================================================================================
float Lipo::GetVoltage_Cell_2 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  Measure(CELL2_PIN)* CELL2_COEF / 3.3 * ReadVcc(); 
  val = (MV1 + MV2 + MV3) / 3; // filtrage (Durchschnitt aus 3 Messungen)
  MV1 = MV2; // décalage
  MV2 = MV3;
  //Serial.print("Cell 2:");
  //Serial.println(val);     
  return val;
}

// =================================================================================
float Lipo::GetVoltage_Cell_3 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  Measure(CELL3_PIN)* CELL3_COEF / 3.3 * ReadVcc();
  val = (MV1 + MV2 + MV3) / 3; // filtrage (Durchschnitt aus 3 Messungen)
  MV1 = MV2; // décalage
  MV2 = MV3;
  //Serial.print("Cell 3:");
  //Serial.println(val);     
  return val;
}

// =================================================================================
float Lipo::GetVoltage_Cell_4 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  Measure(CELL4_PIN)* CELL4_COEF / 3.3 * ReadVcc();
  val = (MV1 + MV2 + MV3) / 3; // filtrage (Durchschnitt aus 3 Messungen)
  MV1 = MV2; // décalage
  MV2 = MV3;
  
  return val;
}

// =================================================================================
// Ermittlung der Zellenzahl
uint8_t Lipo::Detect () {
  float val;
  uint8_t Lipocount;
  Lipocount = 2;  // Minimum : 2S, default.
  
  // Wir beginnen mit falschen Messwerten, um Filter zu initialisieren
  for (uint8_t index = 0; index < 2; index++)
  {  
    val = Lipo::GetVoltage_Cell_1 (); // dummy Spannungsmessungen wg. Mittelwertbildung
    val = Lipo::GetVoltage_Cell_2 (); // 
    val = Lipo::GetVoltage_Cell_3 (); // 
    val = Lipo::GetVoltage_Cell_4 (); //    
  }  

  val = Lipo::GetVoltage_Cell_3 (); // Messung bis Zelle 3
  if (val > 8.0) {         // > 2x4V  
    Lipocount = 3;
  }
      
  val = Lipo::GetVoltage_Cell_4 (); // Messung bis Zelle 4
  if (val > 11.0) {         // > 11V  
    Lipocount = 4;
  }      
      
  return Lipocount;  
} 

// =================================================================================
// schwächste Zelle ermitteln
float Lipo::GetWeakCell (){
  float weak = 0.0;  // Voltage
  weakcellno = 1;
  weak = cell1;
  if (cell2 < weak) {
    weak = cell2;
    weakcellno = 2;
  }

  if ((cellcount >= 3) && (cell3 < weak)){
    weak = cell3;
    weakcellno = 3;
  }

  if ((cellcount >= 4) && (cell4 < weak)){
    weak = cell4;
    weakcellno = 4;
  } 
  
  return weak;
}

// =================================================================================
float Lipo::ReadVcc() {
  //https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  //result = 1126400L / result; // Back-calculate AVcc in mV
  //return result;

  
  /*
    http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
    While the large tolerance of the internal 1.1 volt reference 
    greatly limits the accuracy of this measurement, for individual 
    projects we can compensate for greater accuracy. To do so, 
    simply measure your Vcc with a voltmeter and with our readVcc() 
    function. Then, replace the constant 1125300L with a new constant:
    scale_constant = internal1.1Ref * 1023 * 1000
    internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)  
  */
  float internalRef = 1.111; 
  float voltage = internalRef * 1023 / result;
  //Serial.print("Vcc:");
  //Serial.println(voltage);
  return voltage;
} 

// =================================================================================
/*
 * Reads Voltage value on anlog input and outputs it
 * in 0.1V steps to MultiHoTTModule.vbat.
 */
void Lipo::ReadVoltages() {
  if (cellcount==0) {
    cellcount = Detect();
  }

  // Spannungen messen
  cell1 = GetVoltage_Cell_1();  
  cell2 = GetVoltage_Cell_2();

  if (cellcount>2) {
    cell3 = GetVoltage_Cell_3();
  }
  
  if (cellcount>3) {
    cell4 = GetVoltage_Cell_4();
    cell4 -= cell3;
  }    
  
  if (cellcount>2) {
    cell3 -= cell2;
  }
  cell2 -= cell1; 
  
  #if defined DEBUG
  Serial.print("Cells :"); Serial.println(cellcount);
  Serial.print("Cell 1:"); Serial.println(cell1);
  Serial.print("Cell 2:"); Serial.println(cell2);
  Serial.print("Cell 3:"); Serial.println(cell3);
  Serial.print("Cell 4:"); Serial.println(cell4);
  #endif
  
}
