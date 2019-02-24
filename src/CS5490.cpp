/**

	@author Tiago Britto Lobão
	tiago.blobao@gmail.com

	Purpose: Control an integrated circuit
	Cirrus Logic - CS5490

	Used to measure electrical quantities

	MIT License

*/


#include "CS5490.h"



/******* Init CS5490 *******/

//For Arduino & ESP8622
#if !(defined ARDUINO_NodeMCU_32S ) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_Node32s)
	CS5490::CS5490(float mclk, int rx, int tx){
		this->selectedPage = -1;
		this->MCLK = mclk;
		this->cSerial = new SoftwareSerial(rx,tx);
	}
//For ESP32 AND MEGA
#else
	CS5490::CS5490(float mclk){
		this->selectedPage = -1;
		this->MCLK = mclk;
		this->cSerial = &Serial2;
	}
#endif

void CS5490::begin(int baudRate){
	cSerial->begin(baudRate);
	delay(10); //Avoid Bugs on Arduino UNO
}

/**************************************************************/
/*                 METHODS FOR BASIC OPERATIONS               */
/**************************************************************/


/******* Write a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::write(int page, int address, uint32_t value){

	uint8_t buffer;
	//Select page and address
	if(this->selectedPage != page){
		buffer = (pageByte | (uint8_t)page);
		cSerial->write(buffer);
		this->selectedPage = page;
	}
	buffer = (writeByte | (uint8_t)address);
	cSerial->write(buffer);

	//Send information
	for(int i=0; i<3 ; i++){
		data[i] = value & 0x000000FF;
		cSerial->write(this->data[i]);
		value >>= 8;
	}
}

/******* Read a register by the serial communication *******/
/* data bytes pass by data variable from this class */

void CS5490::read(int page, int address){

	this->clearSerialBuffer();

	uint8_t buffer;
	//Select page and address
	if(this->selectedPage != page){
		buffer = (pageByte | (uint8_t)page);
		cSerial->write(buffer);
		this->selectedPage = page;
	}
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

  @param
  data[] => Array with size 3. Each uint8_t is an 8 byte number received from CS5490
  LSBpow => Exponent specified from datasheet of the less significant bit
  MSBoption => Information of most significant bit case. It can be only three values:
    MSBnull (1)  The MSB is a Don't Care bit
    MSBsigned (2) the MSB is a negative value, requiring a 2 complement conversion
    MSBunsigned (3) The MSB is a positive value, the default case.
	@return value from last data received from CS55490

	https://repl.it/@tiagolobao/toDoubletoBinary-CS5490
*/
double CS5490::toDouble(int LSBpow, int MSBoption){

	double output = 0.0;
	bool MSB;

	uint32_t buffer = this->concatData();

  switch(MSBoption){

    case MSBnull:
			buffer &= 0x7FFFFF; //Clear MSB
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
  		} else {  //+
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

/*
  Function: toBinary
  Transforms a double number to a 24 bit number for writing registers

  @param
  LSBpow => Expoent specified from datasheet of the less significant bit
  MSBoption => Information of most significant bit case. It can be only three values:
    MSBnull (1)  The MSB is a Don't Care bit
    MSBsigned (2) the MSB is a negative value, requiring a 2 complement conversion
    MSBunsigned (3) The MSB is a positive value, the default case.
  input => (double) value to be sent to CS5490
 @return binary value equivalent do (double) input

	https://repl.it/@tiagolobao/toDoubletoBinary-CS5490
*/
uint32_t CS5490::toBinary(int LSBpow, int MSBoption, double input){

	uint32_t output;

  switch(MSBoption){
    case MSBnull:
      input *= pow(2,LSBpow);
      output = (uint32_t)input;
      output &= 0x7FFFFF; //Clear Don't care bits
    break;
    case MSBsigned:
      if(input <= 0){ //- (2 complement conversion)
        input *= -pow(2,LSBpow);
        output = (uint32_t)input;
        output = ~output;
        output = (output+1) & 0xFFFFFF; //Clearing the first 8 bits
      } else {       //+
        input *= (pow(2,LSBpow)-1.0);
        output = (uint32_t)input;
      }
    break;
    default:
    case MSBunsigned:
  		input *= pow(2,LSBpow);
      output = (uint32_t)input;
    break;
  }

  return output;
}

/******* Concatenation of the incomming data from CS5490 *******/
uint32_t CS5490::concatData(){
	uint32_t output;
	output = output + data[2] << 8;
	output = output + data[1] << 8;
	output = output + data[0];
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

void CS5490::calibrate(uint8_t type, uint8_t channel){
	int settleTime = 2000; //Wait 2 seconds before and after
	delay(settleTime);
	uint8_t calibrationByte = 0b00100000;
	calibrationByte &= (type&channel);
	this->instruct(calibrationByte);
	delay(settleTime);
}

/**************************************************************/
/*       PUBLIC METHODS - Configuration                       */
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

long CS5490::getBaudRate(){
	this->read(0,7);
	uint32_t buffer = this->concatData();
	buffer -= 0x020000;
	return ( (buffer/0.5242880)*MCLK );
}


/**************************************************************/
/*       PUBLIC METHODS - Calibration                         */
/**************************************************************/

/* GAIN */

double CS5490::getGainSys(){
	this->read(16,60);
	return this->toDouble(22,MSBsigned);
}

double CS5490::getGainV(){
	this->read(16,35);
	return this->toDouble(22,MSBunsigned);
}

double CS5490::getGainI(){
	this->read(16,33);
	return this->toDouble(22,MSBunsigned);
}

double CS5490::getGainT(){
	this->read(16,54);
	return this->toDouble(16,MSBunsigned);
}

void CS5490::setGainSys(double value){
	uint32_t binValue = this->toBinary(22,MSBsigned,value);
  this->write(16,60,binValue);
}

void CS5490::setGainV(double value){
	uint32_t binValue = this->toBinary(22,MSBunsigned,value);
  this->write(16,35,binValue);
}

void CS5490::setGainI(double value){
	uint32_t binValue = this->toBinary(22,MSBunsigned,value);
  this->write(16,33,binValue);
}

void CS5490::setGainT(double value){
	uint32_t binValue = this->toBinary(16,MSBunsigned,value);
  this->write(16,54,binValue);
}

/* OFFSET */

double CS5490::getDcOffsetV(){
  this->read(16,34);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getDcOffsetI(){
  this->read(16,32);
	return this->toDouble(23, MSBsigned);
}

double CS5490::getAcOffsetI(){
  this->read(16,37);
	return this->toDouble(24, MSBunsigned);
}

double CS5490::getOffsetT(){
  this->read(16,55);
	return this->toDouble(23, MSBsigned);
}

void CS5490::setDcOffsetV(double value){
	uint32_t binValue = this->toBinary(23,MSBsigned,value);
  this->write(16,34,binValue);
}

void CS5490::setDcOffsetI(double value){
	uint32_t binValue = this->toBinary(23,MSBsigned,value);
  this->write(16,32,binValue);
}

void CS5490::setAcOffsetI(double value){
	uint32_t binValue = this->toBinary(24,MSBunsigned,value);
  this->write(16,37,binValue);
}

void CS5490::setOffsetT(double value){
	uint32_t binValue = this->toBinary(23,MSBsigned,value);
  this->write(16,55,binValue);
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

uint32_t CS5490::readReg(int page, int address){
	this->read(page, address);
	return this->concatData();
}
