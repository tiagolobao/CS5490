#include <SoftwareSerial.h>

/* Leitura de um registrador sem a biblioteca */

SoftwareSerial mySerial(14, 12); // RX, TX

int incbyte;



void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(600);
  mySerial.write(0b10010000);

}

void loop() // run over and over
{
  mySerial.write(0x31);
  while (mySerial.available() > 3){
    char inByte = mySerial.read();
    Serial.print(inByte,HEX);
    inByte = mySerial.read();
    Serial.print(inByte,HEX);
    inByte = mySerial.read();
    Serial.print(inByte,HEX);
    //Ele deve mostrar 9A991
  }
  Serial.println("");
  delay(5000);
}
