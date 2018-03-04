
#include<CS5490.h>


CS5490 line(MCLK_default,14,12);


void setup() {
  //Initializing communication with CS5490
  //600 is the default baud rate velocity.
  line.begin(600);
  //Initializing communication arduino/PC to show results in Monitor Serial
  Serial.begin(115200);


  //Turns Continous conversion ON. Necessary for some measurments
  line.CC();
}

void loop() {

  //Get instant voltage number from CS5490
  line.getInstV();

  //Prints last "get" method in Serial Monitor
  Serial.println("The instant voltage value is: ");
  Serial.println(line.data[0],HEX);
  Serial.println(line.data[1],HEX);
  Serial.println(line.data[2],HEX);
  Serial.println("");

  //Get the current peak information from CS5490
  line.getPeakI();

  //Prints last "get" method in Serial Monitor
  Serial.println("The peak current value is: ");
  Serial.println(line.data[0],HEX);
  Serial.println(line.data[1],HEX);
  Serial.println(line.data[2],HEX);
  Serial.println("");



  delay(1000);
}
