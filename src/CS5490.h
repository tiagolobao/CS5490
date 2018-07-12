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
	or suggestions, contact me.



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
//Software Serial Library
#ifndef ARDUINO_NodeMCU_32S //For Arduino & Others
	#ifndef  SoftwareSerial.h
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
	float MCLK;

	#ifndef ARDUINO_NodeMCU_32S //Arduino & ESP8622
		SoftwareSerial *cSerial;
		CS5490(float mclk, int rx, int tx);
	#endif

	#ifdef ARDUINO_NodeMCU_32S //ESP32
		HardwareSerial *cSerial;
			CS5490(float mclk);
	#endif

	void read(int page, int address);
	void instruct(int instruction);
	double toDouble(int LBSpow, int MSBoption);

	//Some temporary public methods and atributes
	void write(int page, int address, long value);
	uint8_t data[3]; //data buffer for read and write

	void begin(int baudRate);

	/* Not implemented functions
	void setData(double input);
	void setData(uint8_t input[]);
	uint8_t* toByteArray(int LBSpow, int MSBoption);
	---------------------- */

	/*** Instructions ***/
	void reset();
	void standby();
	void wakeUp();
	void CC(); //Continous Convertion

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

	/*** Configuration ***/

	void setBaudRate(long value);
	/* Not implemented functions
	void setDO(int mode);
	-------------------------*/


};

#endif
