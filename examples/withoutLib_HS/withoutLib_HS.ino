/*
  1 - THIS DOES NOT USES CS5490 LIBRARY, use only if you want to understand
only if you are having some trouble or needs to understante how it works

  2 - THIS EXAMPLES USES SoftwareSerial library, it means that will be useful
only for ARDUINO MEGA, ESP32 and others with multiple hardware serial components
*/

void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  // set the data rate for the SoftwareSerial port
  Serial2.begin(600);
  //Defines the page for the register address
  Serial2.write(0b10010000); //Select Page 16
}

void loop(){
  Serial2.write(0x31); //Read Address 49
  while (Serial2.available() > 3){ //Wait for 3 bytes to arrive
    char inByte = Serial2.read();
    Serial.print(inByte,HEX);
    inByte = Serial2.read();
    Serial.print(inByte,HEX);
    inByte = Serial2.read();
    Serial.print(inByte,HEX);
    //The result must be 9A991
  }
  Serial.println("");
  delay(1000);
}
