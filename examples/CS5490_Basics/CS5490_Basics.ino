
#include<CS5490.h>

#define rx 14
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
}

void loop() {

  double foo = line.getFreq();
  double bar = line.getTime();

  Serial.print("The Line to Sample Frequency Ratio is: ");
  Serial.println( foo , 10 ); //10 is the number of decimal places

  Serial.print("The System Time is: ");
  Serial.println( bar );

  Serial.println("");
  delay(1000);
}
