/******************************************

	Author: Tiago Britto Lobão
	tiago.blobao@gmail.com
*/


/*
	Purpose: Control an integrated circuit
	Cirrus Logic - CS5490

	Used to measure electrical quantities

	MIT License

******************************************/


#include "CS5490.h"



/******* Init CS5490 *******/

//For Arduino & ESP8622
#if !(defined ARDUINO_NodeMCU_32S ) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_Node32s)
	CS5490::CS5490(float mclk, int rx, int tx){
		this->MCLK = mclk;
		this->cSerial = new SoftwareSerial(rx,tx);
	}
//For ESP32 AND MEGA
#else
	CS5490::CS5490(float mclk){
		this->MCLK = mclk;
		this->cSerial = &Serial2;
	}
#endif

void CS5490::begin(int baudRate){
	cSerial->begin(baudRate);
	delay(10); //Avoid Bugs on Arduino UNO
}

/**************************************************************/
/*                     PRIVATE METHODS                        */
/**************************************************************/


/******* Write a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::write(int page, int address, long value){

	uint8_t checksum = 0;
	for(int i=0; i<3; i++)
		checksum += 0xFF - checksum;

	//Select page and address
	uint8_t buffer = (pageByte | (uint8_t)page);
	cSerial->write(buffer);
	buffer = (writeByte | (uint8_t)address);
	cSerial->write(buffer);

	//Send information
	for(int i=0; i<3 ; i++){
		data[i] = value & 0x000000FF;
		cSerial->write(this->data[i]);
		value >>= 8;
	}
	//Calculate and send checksum
	buffer = 0xFF - data[0] - data[1] - data[2];
	cSerial->write(buffer);
}

/******* Read a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::read(int page, int address){

	this->clearSerialBuffer();

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
	uint8_t buffer = (instructionByte | (uint8_t)value);
	cSerial->write(buffer);
}

/******* Clears cSerial Buffer *******/
void CS5490::clearSerialBuffer(){
	while (cSerial->available()) cSerial->read();
}

/*
  Function: toDouble
  Transforms a 24 bit number to a double number for easy processing data

  Param:
  data[] => Array with size 3. Each uint8_t is an 8 byte number received from CS5490
  LSBpow => Exponent specified from datasheet of the less significant bit
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
  			buffer = buffer & 0x00FFFFFF; //Clearing the first 8 bits
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

void CS5490::singConv(){
	this->instruct(20);
}

void CS5490::contConv(){
	this->instruct(21);
}

void CS5490::haltConv(){
	this->instruct(24);
}




/**************************************************************/
/*       PUBLIC METHODS - Calibration and Configuration       */
/**************************************************************/

/* SET */
void CS5490::setBaudRate(long value){

	//Calculate the correct binary value
	uint32_t hexBR = ceil(value*0.5242880/MCLK);
	if (hexBR > 65535) hexBR = 65535;
	hexBR += 0x020000;

	this->write(0x80,0x07,hexBR);
	delay(100); //To avoid bugs from ESP32

	//Reset Serial communication from controller
	cSerial->end();
	cSerial->begin(value);
	delay(50); //Avoid bugs from Arduino MEGA
	return;
}

/* GET */
int CS5490::getGainI(){
	//Page 16, Address 33
	this->read(16,33);
	return this->toDouble(22,MSBunsigned);
}

long CS5490::getBaudRate(){
	this->read(0,7);
	uint32_t buffer = this->data[0];
	buffer += this->data[1] << 8;
	buffer += this->data[2] << 16;
	buffer -= 0x020000;
	return ( (buffer/0.5242880)*MCLK );
}

/**************************************************************/
/*              PUBLIC METHODS - Measurements                 */
/**************************************************************/

double CS5490::getPeakV(){
	//Page 0, Address 36
	this->read(0,36);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getPeakI(){
	//Page 0, Address 37
	this->read(0,37);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getInstI(){
	//Page 16, Address 2
	this->read(16,2);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getInstV(){
	//Page 16, Address 3
	this->read(16,3);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getInstP(){
	//Page 16, Address 4
	this->read(16,4);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getRmsI(){
	//Page 16, Address 6
	this->read(16,6);
	return this->toDouble(24, MSBunsigned);
}

double CS5490::getRmsV(){
	//Page 16, Address 7
	this->read(16,7);
	return this->toDouble(24, MSBunsigned);
}

double CS5490::getAvgP(){
	//Page 16, Address 5
	this->read(16,5);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getAvgQ(){
	//Page 16, Address 14
	this->read(16,14);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getAvgS(){
	//Page 16, Address 20
	this->read(16,20);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getInstQ(){
	//Page 16, Address 15
	this->read(16,15);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getPF(){
	//Page 16, Address 21
	this->read(16,21);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getTotalP(){
	//Page 16, Address 29
	this->read(16,29);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getTotalS(){
	//Page 16, Address 30
	this->read(16,30);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getTotalQ(){
	//Page 16, Address 31
	this->read(16,31);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getFreq(){
	//Page 16, Address 49
	this->read(16,49);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getTime(){
	//Page 16, Address 61
	this->read(16,61);
	return this->toDouble(0, MSBunsigned);
}

/**************************************************************/
/*              PUBLIC METHODS - Read Register                */
/**************************************************************/

long CS5490::readReg(int page, int address){
	uint32_t value = 0;
	this->read(page, address);
	value = value + data[2] << 8;
	value = value + data[1] << 8;
	value = value + data[0];
	return value;
}
