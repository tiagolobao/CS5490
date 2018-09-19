/*
  1 - THIS EXAMPLE DOES NOT USES CS5490 LIBRARY, use only if you want to understand
only if you are having some trouble or needs to understante how it works

  2 - THIS EXAMPLES USES SoftwareSerial library, it means that will be useful
only for ARDUINO MEGA, ESP32 and others with multiple hardware serial components
*/

byte data[3]; //data buffer

void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);
  // set the data rate for the HardwareSerial 2 port
  Serial2.begin(600);
}

void loop(){

  /* The default value is: 01999A */
  clearSerial2Buffer();
  Serial2.write(0b10010000); //Select Page 16
  Serial2.write(0b00110001); //Read Address 49
	//Wait for 3 bytes to arrive
	while(Serial2.available() < 3);
  //Read 3 byte information
	for(int i=0; i<3; i++){
		data[i] = Serial2.read();
	}
  //Data concatenation
  uint32_t value = 0;
  value = value + data[2] << 8;
  value = value + data[1] << 8;
  value = value + data[0];

  Serial.println(value,HEX);
  delay(1000);
}

/* Clearing Serial2 Buffer */
void clearSerial2Buffer(){
  while(Serial2.available()){
    Serial2.read();
  }
}
