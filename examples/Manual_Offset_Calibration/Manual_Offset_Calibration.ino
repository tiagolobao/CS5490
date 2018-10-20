/*
  !!! This example is too simple. It is better to make multiple
  readings and calculate a more precise valie for offset calibration
*/


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
  line.begin(baudRate_default);
  //Initializing communication arduino/PC to show results in Monitor Serial
  Serial.begin(115200);
  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);
  //Set to continous conversion
  line.contConv();
  delay(100);
  line.setGainI(1.0);
  delay(100);
}

void loop() {

  Serial.println("\nWithout calibration");
  line.setDcOffsetI(0); //Reset previous calibration
  double foo = line.getInstI();
  Serial.println(foo, 5);
  line.setDcOffsetI(-foo); //Calibrate by the last read value

  Serial.println("\nCalibrated");
  foo = line.getInstI();
  Serial.println(foo, 5);

  Serial.println("\nReset arduino to see it again... ");
  while(1);

}
