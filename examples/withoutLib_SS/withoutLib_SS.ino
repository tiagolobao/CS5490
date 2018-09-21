
/*
  1 - THIS EXAMPLE DOES NOT USE CS5490 LIBRARY, use only if you
  are having some trouble or need to understand how it works

  2 - THIS EXAMPLE USES SoftwareSerial library, it means that it
  will be useful only for ARDUINO UNO, ESP8622 and others
*/

#include <SoftwareSerial.h>

#define rx 11
#define tx 12

SoftwareSerial mySerial(rx, tx); // RX, TX
byte data[3]; //data buffer

void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(600);
  delay(100); //Avoid Arduino UNO Bug
}

void loop(){

  /* The default value is: 01999A */
  clearmySerialBuffer();
  mySerial.write(0b10010000); //Select Page 16
  mySerial.write(0b00110001); //Read Address 49
	//Wait for 3 bytes to arrive
	while(mySerial.available() < 3);
  //Read 3 byte information
	for(int i=0; i<3; i++){
		data[i] = mySerial.read();
	}
  //Data concatenation
  uint32_t value = 0;
  value = value + data[2] << 8;
  value = value + data[1] << 8;
  value = value + data[0];

  Serial.println(value,HEX);
  delay(1000);
}

/* Clearing mySerial Buffer */
void clearmySerialBuffer(){
  while(mySerial.available()){
    mySerial.read();
  }
}
