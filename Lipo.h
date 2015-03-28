#include "Arduino.h"

// Berechnng der Koeffizienten für die Spannungsteiler 
// COEF = ((R1 + R2)/R2) * (VREF/1024) 
// VREF = 3,3V (intern)
#define CELL1_COEF 0.004268     // R1= 14,96K , R2= 4,73K
#define CELL2_COEF 0.008995     // R3= 14,97K , R4= 26,6K
#define CELL3_COEF 0.013962     // R5=  8,17K , R6= 27,0K
#define CELL4_COEF 0.018499     // R7=  8,19K , R8= 38,4K

// Definition der Analogpins zur Spannungsmesssung
#define CELL1_PIN A0
#define CELL2_PIN A1
#define CELL3_PIN A2
#define CELL4_PIN A3


class Lipo{
  
  public:
  uint8_t cellcount; // Zellenzahl
  float cell1;       // Zellspannungen 1..4
  float cell2;
  float cell3;
  float cell4; 
  uint8_t weakcellno;
  void init(); 
  void ReadVoltages();
  float GetWeakCell();
  
  private:
  float GetVoltage_Cell_1 ();
  float GetVoltage_Cell_2 ();
  float GetVoltage_Cell_3 ();
  float GetVoltage_Cell_4 ();

  uint8_t Detect ();
  float ReadVcc();
  //long readTemp();
  uint16_t Measure(uint8_t pin);
};

