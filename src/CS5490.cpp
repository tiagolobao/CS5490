/******************************************

	Author: Tiago Britto Lobão
	tiago.blobao@gmail.com
	Year: 2017
	Version: 0.1

*/


/*
	Purpose: Control an integrated circuit
	Cirrus Logic - CS5490

	Used to measure electrical quantities

	This is a FREE SOFTWARE. You can change
	it or distribute. If you notice any issues
	or suggestions, contact me



******************************************/


#include "CS5490.h"
#include "Arduino.h"

#include <SoftwareSerial.h> //Software Serial Library


/******* Init CS5490 *******/

CS5490::CS5490(float mclk, int rx, int tx){
	this->MCLK = mclk;

	//Arduino Like
	//this->cSerial = new SoftwareSerial(rx,tx);

	//ESP Like
	this->cSerial = new SoftwareSerial(rx,tx);
	//SoftwareSerial swSer(14, 12, false, 256);
	//SoftwareSerial swSer(int receivePin, int transmitPin, bool inverted_logic, int buffer);
}

void CS5490::begin(int baudRate){
	cSerial->begin(baudRate);
}

/**************************************************************/
/*                     PRIVATE METHODS                        */
/**************************************************************/


/******* Write a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::write(int page, int address){

	cSerial->flush();

	uint8_t buffer = (pageByte | (uint8_t)page);
	cSerial->write(buffer);
	buffer = (writeByte | (uint8_t)address);
	cSerial->write(buffer);

	delay(10); //Wait for data
	for(int i; i<3 ; i++)
		cSerial->write(this->data[i]);
}

/******* Read a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::read(int page, int address){

	cSerial->flush();

	uint8_t buffer = (pageByte | (uint8_t)page);
	cSerial->write(buffer);
	buffer = (readByte | (uint8_t)address);
	cSerial->write(buffer);

	//Wait for 3 bytes to arrive
	while(cSerial->available() < 3);
	for(int i=0; i<3; i++){
		data[i] = cSerial->read();
	}
}

/******* Give an instruction by the serial communication *******/

void CS5490::instruct(int value){

	cSerial->flush();

	uint8_t buffer = (instructionByte | (uint8_t)value);
	cSerial->write(buffer);
}


/*
  Function: toDouble
  Transforms a 24 bit number to a double number for easy processing data

  Param:
  data[] => Array with size 3. Each uint8_t is an 8 byte number received from CS5490
  LSBpow => Expoent specified from datasheet of the less significant bit
  MSBoption => Information of most significant bit case. It can be only three values:
    MSBnull (1)  The MSB is a Don't Care bit
    MSBsigned (2) the MSB is a negative value, requiring a 2 complement conversion
    MSBunsigned (3) The MSB is a positive value, the default case.

*/
double CS5490::toDouble(int LSBpow, int MSBoption){

	uint32_t buffer = 0;
	double output = 0.0;
	bool MSB;

	//Concat bytes in a 32 bit word
	buffer += this->data[0];
	buffer += this->data[1] << 8;
	buffer += this->data[2] << 16;

  switch(MSBoption){

    case MSBnull:
      this->data[2] &= ~(1 << 7); //Clear MSB
      buffer += this->data[2] << 16;
      output = (double)buffer;
      output /= pow(2,LSBpow);
    break;

    case MSBsigned:
      MSB = data[2] & 0x80;
  		if(MSB){  //- (2 complement conversion)
  			buffer = ~buffer;
  			//Clearing the first 8 bits
  			for(int i=24; i<32; i++)
  			  buffer &= ~(1 << i);
  			output = (double)buffer + 1.0;
  			output /= -pow(2,LSBpow);
  		}
  		else{     //+
  		  output = (double)buffer;
  			output /= (pow(2,LSBpow)-1.0);
  		}
    break;

    default:
    case MSBunsigned:
      output = (double)buffer;
  		output /= pow(2,LSBpow);
    break;

  }

	return output;
}

/**************************************************************/
/*              PUBLIC METHODS - Read Register                */
/**************************************************************/

void CS5490::readRegister(int page, int address){
	this->read(page, address);
}

/**************************************************************/
/*              PUBLIC METHODS - Instructions                 */
/**************************************************************/
void CS5490::reset(){
	this->instruct(1);
}

void CS5490::standby(){
	this->instruct(2);
}

void CS5490::wakeUp(){
	this->instruct(3);
}

void CS5490::CC(){
	this->instruct(21);
}




/**************************************************************/
/*              PUBLIC METHODS - Calibration                  */
/**************************************************************/


void CS5490::setOffsetV(int value){
	//Page 16, Address 34
	this->read(16,34);
	return;
}

int CS5490::getGainI(){
	//Page 16, Address 33
	this->read(16,33);
	return 0;
}


/**************************************************************/
/*              PUBLIC METHODS - Measurements                 */
/**************************************************************/

int CS5490::getInstantV(){
	//Page 16, Address 3
	this->read(16,3);
	return 0;
}

int CS5490::getRmsV(){
	//Page 16, Address 7
	this->read(16,7);
	return 0;
}

int CS5490::getPeakV(){
	this->read(0,36);
	return 0;
}

int CS5490::getPeakI(){
	this->read(0,37);
	return 0;
}
