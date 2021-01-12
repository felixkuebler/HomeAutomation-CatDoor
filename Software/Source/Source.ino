
#include <stdint.h>



#define PIN_MOTOR_A_1 4
#define PIN_MOTOR_A_2 5

#define PIN_MOTOR_ENDSTOP_A_1 9
#define PIN_MOTOR_ENDSTOP_A_2 10

#define PIN_MOTOR_B_1 14
#define PIN_MOTOR_B_2 16

#define PIN_MOTOR_ENDSTOP_B_1 12
#define PIN_MOTOR_ENDSTOP_B_2 13


#define DIR_IDLE 0
#define DIR_OPEN 1
#define DIR_CLOSE 2
#define DIR_WAIT 3

uint8_t direction = DIR_IDLE;

const uint32_t timeThresh = 2000;
uint32_t timeOpened = 0;



const uint8_t dataLen = 7;
const uint8_t dataOffset = 4;
const uint8_t bufLen = 13;

uint8_t accessToken[dataLen] = {0x03, 0x84, 0x00, 0x0C, 0x05, 0xB2, 0xA8};



bool readSerial(uint8_t* buff, uint8_t len) {  
  if (Serial.available()) { 
    
    buff[0] = Serial.read();
    
    if (buff[0] == 0xAA) {
      for (int i=1; true; i++) {
        while (!Serial.available()) delay(1);
        buff[i] = Serial.read();

        if(buff[i] == 0xBB && i>=len-1) break;
      }

      return true;
    }
  }
  return false;
}

uint8_t compairBuffer(uint8_t* buff1, uint8_t* buff2, uint8_t len) {
  uint8_t error=0;
  for (uint8_t i=0; i<len; i++){
    if (buff1[i]!=buff2[i]){
      error++;
    }
  }
  return error;
}


void setup() {

  pinMode(PIN_MOTOR_A_1, OUTPUT);
  pinMode(PIN_MOTOR_A_2, OUTPUT);

  pinMode(PIN_MOTOR_B_1, OUTPUT);
  pinMode(PIN_MOTOR_B_2, OUTPUT);

  pinMode(PIN_MOTOR_ENDSTOP_A_1, INPUT);
  pinMode(PIN_MOTOR_ENDSTOP_A_2, INPUT);

  pinMode(PIN_MOTOR_ENDSTOP_B_1, INPUT);
  pinMode(PIN_MOTOR_ENDSTOP_B_2, INPUT);
  
  Serial.begin(9600);

}

void loop() 
{
/*
  if(Serial.available()>0)
  {
    while(Serial.available()>0)
    {
      Serial.read();
    }
    
    if(direction==1) direction = 2;
    else direction = 1;

    stop_a=false;
    stop_b=false;
  }
*/

  
  uint8_t buff[bufLen];

  if (readSerial(buff, bufLen)){

      uint8_t data[dataLen];
      for (uint8_t i=0; i<dataLen; i++){
        data[i] = buff[i+dataOffset];
      }
        
      for (uint8_t i=0; i<bufLen; i++) {
        Serial.print("0x");
        Serial.print(buff[i], HEX);
        Serial.print(" ");
      }

      Serial.print("   -   ");

      for (uint8_t i=0; i<dataLen; i++) {
        Serial.print("0x");
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }

      Serial.print("   -   ");

      if (compairBuffer(accessToken, data, dataLen) == 0){
        Serial.print("Access ID correct.");
        direction = DIR_OPEN;
      }
      else {
        Serial.print("Not a valid access ID.");
      }
      
      Serial.println("");
   }


  if(direction == DIR_OPEN)
  {
    if(digitalRead(PIN_MOTOR_ENDSTOP_A_1)==HIGH)
    {
      //Serial.println("ENDSTOPT_A_1");
      analogWrite(PIN_MOTOR_A_1, 0);
      analogWrite(PIN_MOTOR_A_2, 0);
    }
    else
    {
      //Serial.println("a to 1");
      analogWrite(PIN_MOTOR_A_2, 0);
      analogWrite(PIN_MOTOR_A_1, 1024);
    }  
    
    if(digitalRead(PIN_MOTOR_ENDSTOP_B_1)==HIGH) 
    {
      //Serial.println("ENDSTOPT_B_1");
      analogWrite(PIN_MOTOR_B_1, 0);
      analogWrite(PIN_MOTOR_B_2, 0);
    }
    else
    {
      //Serial.println("b to 1");
      analogWrite(PIN_MOTOR_B_2, 0);
      analogWrite(PIN_MOTOR_B_1, 1024);
    }

    if(digitalRead(PIN_MOTOR_ENDSTOP_A_1)==HIGH && digitalRead(PIN_MOTOR_ENDSTOP_B_1)==HIGH){
      //Serial.println("ENDSTOPT_B_1 and ENDSTOPT_A_1");
      timeOpened =  millis();
      direction=DIR_WAIT;
    }
  }
  else if (direction==DIR_CLOSE)
  {
    if(digitalRead(PIN_MOTOR_ENDSTOP_A_2)==HIGH) 
    {                                                                                                                   
      //Serial.println("ENDSTOPT_A_2");
      analogWrite(PIN_MOTOR_A_1, 0);
      analogWrite(PIN_MOTOR_A_2, 0);
    }
    else
    {
      //Serial.println("a to 2");
      analogWrite(PIN_MOTOR_A_1, 0);
      analogWrite(PIN_MOTOR_A_2, 1024);
    }

    if(digitalRead(PIN_MOTOR_ENDSTOP_B_2)==HIGH)
    {
      //Serial.println("ENDSTOPT_B_2");
      analogWrite(PIN_MOTOR_B_1, 0);
      analogWrite(PIN_MOTOR_B_2, 0);
    }
    else
    {
      //Serial.println("b to 2");
      analogWrite(PIN_MOTOR_B_1, 0);
      analogWrite(PIN_MOTOR_B_2, 1024);
    }
  }
  else if (direction==DIR_WAIT){
    uint32_t timeCurrent = millis();

    uint32_t timeDiff = timeCurrent - timeOpened;
    //Serial.print("Time Waited: ");
    //Serial.println(timeDiff);

    if(timeDiff >= timeThresh){
      direction=DIR_CLOSE;
    }
  }
  
  delay(50);            
}
