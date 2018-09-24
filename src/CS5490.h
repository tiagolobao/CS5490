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


/*

	Hardware important topics

	VIN Max voltage: 250mV (input impedance: 2  MOhm)
	IIN Max current: 250mV (input impedance: 30 KOhm)

*/


#ifndef CS5490_h
#define CS5490_h

// Used .h files
#include "Arduino.h" //Arduino Library

#if !(defined ARDUINO_NodeMCU_32S ) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_Node32s)
	#ifndef SoftwareSerial
		#include <SoftwareSerial.h>
	#endif
#endif

/* For toDouble method */
#define MSBnull 1
#define MSBsigned 2
#define MSBunsigned 3

/* Default values */
#define MCLK_default 4.096
#define baudRate_default 600

// all comands templates
#define readByte        0b00000000
#define writeByte       0b01000000
#define pageByte        0b10000000
#define instructionByte 0b11000000
/*
Used to combine with the 6 last bits
Ex: Select page number 3 -> 000011

 10 000000    ==\  OR    ==\
 00 000011    ==/ GATE   ==/    10 000011

*/



class CS5490{

public:

	#if !(defined ARDUINO_NodeMCU_32S ) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_Node32s)
		SoftwareSerial *cSerial;
		CS5490(float mclk, int rx, int tx);
	#else
		HardwareSerial *cSerial;
		CS5490(float mclk);
	#endif

	uint32_t data[3]; //data buffer for read and write
	int selectedPage;
	float MCLK;

	void write(int page, int address, long value);
	void read(int page, int address);
	void instruct(int instruction);
	void begin(int baudRate);
	void clearSerialBuffer();
	double toDouble(int LBSpow, int MSBoption);
	uint32_t toBinary(int LSBpow, int MSBoption, double input);

	/* Not implemented functions
	void setData(double input);
	void setData(uint8_t input[]);
	uint8_t* toByteArray(int LBSpow, int MSBoption);
	---------------------- */

	/*** Instructions ***/
	void reset();
	void standby();
	void wakeUp();
	void singConv();
	void contConv();
	void haltConv();

	/*** Calibration ***/

	int getGainI();

	/* Not implemented functions
	void setGainSys(int value);
	void setGainV(int value);
	void setGainI(int value);
	void setGainT(int value);
	void setPhaseCompensation(int mode, int phase);
	void setOffsetV(int value);
	void setOffsetI(int value);
	void setOffsetT(int value);
	void setCalibrationScale(int value);
	------------------------*/

 	/*** Measurements ***/

	double getPeakV();
	double getPeakI();

	double getInstI();
	double getInstV();
	double getInstP();

	double getRmsI();
	double getRmsV();

	double getAvgP();
	double getAvgQ();
	double getAvgS();

	double getInstQ();
	double getPF();

	double getTotalP();
	double getTotalS();
	double getTotalQ();

	double getFreq();

	double getTime();

	/*** Configuration ***/
	long getBaudRate();
	void setBaudRate(long value);
	/* Not implemented functions
	void setDO(int mode);
	-------------------------*/

	/* EXTRA METHOD */
	long readReg(int page, int address);
};

#endif
