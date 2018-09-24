
#include<CS5490.h>

#define rx 11
#define tx 12

/* Choose your board */

/* Arduino UNO and ESP8622 */
CS5490 line(MCLK_default,rx,tx);

/* ESP and MEGA  (Uses Serial2)*/
//CS5490 line(MCLK_default);


void setup() {
  //Initializing communication with CS5490
  //600 is the default baud rate velocity.
  line.begin(600);
  //Initializing communication arduino/PC to show results in Monitor Serial
  Serial.begin(115200);
  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);
  //Set to single conversion
  line.singConv();
}

void loop() {

  /* P.S: if the offset is a positive value, you need to
      write the same value at calibration register, but negative
   */

  /* Option 1: Manual calibration
  1 - Read instant value for 0V input
  2 - Write the value somewere else
  3 - Write the DC offset voltage (at void setup)
  */

  /* Option 2: Automated Manual calibration
  1 - Read instant value for 0V input
  2 - Store value at Arduino EEPROM
  3 - Read EEPROM and store DC offset voltage (at void setup)
  */

  /* Option 3: Automatic calibration
  1 - Enable the desired calibration
  2 - Execute calibration
  3 - Read the results from calibration Register
  4 - Store at Arduino EEPROM
  5 - Read EEPROM and store DC offset voltage (at void setup)
  */

  /*

  //line.write(page,address,value***)
  line.write(16,34,0xaabbcc);

  *** => value is the concatenated bytes to send

}
