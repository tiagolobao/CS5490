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

	Rational Class from:
	http://courses.washington.edu/css342/zander/css332/ratcpp.html


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
	this->cSerial = new SoftwareSerial(rx,tx,false,256);
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

	//uint8_t data[3];
	delay(10); //Wait for data
	for(int i; i<3 ; i++)
		this->data[i] = cSerial->read();
}

/******* Give an instruction by the serial communication *******/

void CS5490::instruct(int value){

	cSerial->flush();

	uint8_t buffer = (instructionByte | (uint8_t)value);
	cSerial->write(buffer);
}


/***** Return float based on data attribute of this class *****/

uint32_t CS5490::numberfy(int dotPosition, bool unsign){
	uint32_t buffer = 0;

	//Contat bytes in a 32 bit word
	buffer += this->data[0] << 16;
	buffer += this->data[1] << 8;
	buffer += this->data[2];

	return buffer;
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