/*
  SerialPassthrough sketch

  Some boards, like the Arduino 101, the MKR1000, Zero, or the Micro,
  have one hardware serial port attached to Digital pins 0-1, and a
  separate USB serial port attached to the IDE Serial Monitor.
  This means that the "serial passthrough" which is possible with
  the Arduino UNO (commonly used to interact with devices/shields that
  require configuration via serial AT commands) will not work by default.

  This sketch allows you to  emulate the serial passthrough behaviour.
  Any text you type in the IDE Serial monitor will be written
  out to the serial port on Digital pins 0 and 1, and vice-versa.

  On the 101, MKR1000, Zero, and Micro, "Serial" refers to the USB Serial port
  attached to the Serial Monitor, and "Serial1" refers to the hardware
  serial port attached to pins 0 and 1. This sketch will emulate Serial passthrough
  using those two Serial ports on the boards mentioned above,
  but you can change these names to connect any two serial ports on a board
  that has multiple ports.

   Created 23 May 2016
   by Erik Nyquist
*/

#include <stdint.h>


void setup() {
  Serial.begin(9600);
}


bool readSerial(uint8_t* buff) {  
  if (Serial.available()) { 
    
    buff[0] = Serial.read();
    
    if (buff[0] == 0xAA) {
      for (int i=1; true; i++) {
        while (!Serial.available()) delay(1);
        buff[i] = Serial.read();

        if(buff[i] == 0xBB) break;
      }

      return true;
    }
  }
  return false;
}


void loop() {

  uint8_t bufLen = 13;
  uint8_t buff[bufLen];


  if (readSerial(buff)){

      uint8_t dataLen = 7;
      uint8_t data[dataLen];
      for (uint8_t i=0; i<dataLen; i++){
        data[i] = buff[i+4];
      }
        
      for (uint8_t i=0; i<bufLen; i++) {
        Serial.print(buff[i], HEX);
      }
      
      Serial.print("  -  ");

      for (uint8_t i=0; i<dataLen; i++) {
        Serial.print(data[i], HEX);
      }
      
      Serial.println("");
  }
  
}
