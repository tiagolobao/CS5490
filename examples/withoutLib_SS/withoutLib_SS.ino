/*
  1 - THIS DOES NOT USES CS5490 LIBRARY, use only if you want to understand
only if you are having some trouble or needs to understante how it works

  2 - THIS EXAMPLES USES SoftwareSerial library, it means that will be useful
only for ARDUINO UNO, ESP8622 and others
*/

#include <SoftwareSerial.h>

#define rx 14
#define tx 12

SoftwareSerial mySerial(rx, tx); // RX, TX
int incbyte;

void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  // set the data rate for the SoftwareSerial port
  mySerial.begin(600);
  //Defines the page for the register address
  mySerial.write(0b10010000); //Select Page 16
}

void loop(){
  mySerial.write(0x31); //Read Address 49
  while (mySerial.available() > 3){ //Wait for 3 bytes to arrive
    char inByte = mySerial.read();
    Serial.print(inByte,HEX);
    inByte = mySerial.read();
    Serial.print(inByte,HEX);
    inByte = mySerial.read();
    Serial.print(inByte,HEX);
    //The result must be 9A991
  }
  Serial.println("");
  delay(1000);
}
