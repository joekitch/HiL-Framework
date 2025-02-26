#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

//#define DAC_RESOLUTION (9)

int potentiometerPin = A3; // Potentiometer output connected to analog pin 3
int potentiometerVal = 0; // Variable to store the input from the potentiometer

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("MCP4725 test");

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x60);
    
  //Serial.println("Generating increasing voltages");
  Serial.println("Sensing potentiometer value");
}


void loop() {


  potentiometerVal = analogRead(potentiometerPin);   // read the potentiometer value at the input pin, goes from 0 to 1023
  // 204 = 825 = 1v
  // 409.2 = 1650
  // 613.8 = 2475
  // 818.4 = 3,300
  // 1023 = 4095 = 5v
  // 0 = 0 = 0

  // each 5th of pot is 204
  // each 5th of dac is 819
  // 825 / 204 = 4.0322

  // pot value * 4.0 + some fudge value which goes to zero at 1023?
  // DIDN'T WORK?
  // need to instead read analog voltage on pin A3 and convert 1v into 1v through the 4095 value?
  // dac.setVoltage((potentiometerVal*4125)/5, false);

  dac.setVoltage(potentiometerVal*4.0, false);

  // dac.setVoltage((1*4125)/5, false); //set voltage to 1v, use 4500 since actual vref is 4.6 for some reason
  // Serial.println("voltage set to 1v");
  // delay(2000);
  // dac.setVoltage((2*4125)/5, false);
  // Serial.println("voltage set to 2v");
  // delay(2000);
  // dac.setVoltage((3*4125)/5, false);
  // Serial.println("voltage set to 3v");
  // delay(2000);
  // dac.setVoltage((4*4125)/5, false);
  // Serial.println("voltage set to 4v");
  // delay(2000);
  // dac.setVoltage(4095, false); //set voltage to 5v
  // Serial.println("voltage set to 5v");
  // delay(2000);
}

  // uint32_t counter;
  // // Run through the full 12-bit scale for a triangle wave
  // for (counter = 0; counter < 4095; counter++)
  // {dac.setVoltage(counter, false);
    
  // }
  // for (counter = 4095; counter > 0; counter--)
  // {
  //   dac.setVoltage(counter, false);
  // }

// void setup(void) {
//   Serial.begin(9600);
//   Serial.println("Hello!");

//   // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
//   // For MCP4725A0 the address is 0x60 or 0x61
//   // For MCP4725A2 the address is 0x64 or 0x65
//   dac.begin(0x62);
    
//   Serial.println("Generating a triangle wave");
// }

// void loop(void) {
//     uint32_t counter;
//     // Run through the full 12-bit scale for a triangle wave
//     for (counter = 0; counter < 4095; counter++)
//     {
//       dac.setVoltage(counter, false);
//     }
//     for (counter = 4095; counter > 0; counter--)
//     {
//       dac.setVoltage(counter, false);
//     }
// }