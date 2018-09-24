
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
}

void loop() {

  double foo;

  Serial.println("\n\nWithout calibration");
  line.setDcOffsetI(0); //Reset previous calibration
  foo = line.getInstI();
  Serial.print("DC current value: ");
  Serial.println(foo, 5);
  foo = line.getDcOffsetI();
  Serial.print("DC offset current value: ");
  Serial.println(foo, 5);

  /*
  -------->Types
  DCoffset
  ACoffset
  Gain
  -------->Channels
  Current
  Voltage
  CurrentAndVoltage
  -------->How to use?
  line.calibrate(type,channel)
  */
  line.calibrate(DCoffset,Current);

  Serial.println("\n\nCalibrated");
  foo = line.getInstI();
  Serial.print("DC current value: ");
  Serial.println(foo, 5);
  foo = line.getDcOffsetI();
  Serial.print("DC offset current value: ");
  Serial.println(foo, 5);

  Serial.println("\nReset arduino to see it again... ");
  while(1);

}
