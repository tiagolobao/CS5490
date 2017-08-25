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


/* 

	Hardware important topics

	VIN Max voltage: 250mV (input impedance: 2  MOhm)
	IIN Max current: 250mV (input impedance: 30 KOhm)

*/


#ifndef CS5490_h
#define CS5490_h


// Used .h files
#include <stdio.h>
#include <string.h>
#include <inttypes.h> 
#include "Arduino.h" //Arduino Library
#include <SoftwareSerial.h> //Software Serial Library

#define MCLK_default 4.096


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


private:
	float MCLK;
	//uint32_t data[3];
	SoftwareSerial *cSerial;

	void write(int page, int address);
	void read(int page, int address);
	void instruct(int instruction);

	uint32_t numberfy(int dotPosition, bool unsign);

public:

	uint32_t data[3]; //Colocar novamente como private antes de concluir

	CS5490(float mclk, int rx, int tx);

	void begin(int baudRate);

 	/*** Measurements ***/
	int getInstantV();
	int getInstantI();
	int getRmsI();
	int getRmsV();

	int getPowerFactor();

	int getActivePower();
	int getReactivePower();
	int getApparentPower();

	int getInstantReactive();
	int getInstantActive();

	int getTotalActive();
	int getTotalReactive();
	int getTotalAApparent();
  
	int getPeakV();
	int getPeakI();

	int getTemperature();

	/*** Configuration ***/
	void setDO(int mode);
	void setBaudRate(int value);
	
	

	/*** Calibration ***/

	void setGainSys(int value);
	void setGainV(int value);
	void setGainI(int value);
	void setGainT(int value);

	int getGainI();


	void setPhaseCompensation(int mode, int phase);
	
	void setOffsetV(int value);
	void setOffsetI(int value);
	void setOffsetT(int value);	

	void setCalibrationScale(int value);

	/* Instructions */
	void reset();
	void standby();
	void wakeUp();

};

#endif
