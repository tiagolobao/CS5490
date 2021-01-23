
#include<CS5490.h>

#define rx 11
#define tx 12

/* Choose your board */

/* Arduino UNO and ESP8622 */
#ifndef __AVR_ATmega2560__
CS5490 line(MCLK_default,rx,tx);

/* ESP and MEGA  (Uses Serial2)*/
#else
CS5490 line(MCLK_default);
#endif

void setup() {
  //Initializing communication with CS5490
  //600 is the default baud rate velocity.
  line.begin(600);
  //Initializing communication arduino/PC to show results in Monitor Serial
  Serial.begin(115200);
  //Changing BaudRate of CS5490 to 115200
  /*
    WARNING: Everytime that you reset the software from board you also need to reset
    the CS5490!!! If you don't, it will not work properly
  */
  line.setBaudRate(115200);
  //Set to continous conversion
  line.contConv();
}

void loop() {

  double foo = line.getFreq();
  double bar = line.getTime();

  Serial.print("The Line to Sample Frequency Ratio is: ");
  Serial.println( foo , 5 ); //5 is the number of decimal places

  Serial.print("The System Time is: ");
  Serial.println( bar );

  Serial.println("");
  delay(1000);
}
