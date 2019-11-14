/**

	@author Tiago Britto Lobão
	tiago.blobao@gmail.com

	Purpose: Control an integrated circuit
	Cirrus Logic - CS5490

	Used to measure electrical quantities

	MIT License

	Modified by Maurizio Malaspina maurizio.malaspina@gmail.com 
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

/* For toDouble and toBinary method */
#define MSBnull 1
#define MSBsigned 2
#define MSBunsigned 3

/* Constants for Auto Calibration */
/*      contant           0bxxxaabbb
     aa  = Type select
		 bbb = Channel select
*/
//Type
#define DCoffset          0b00000000
#define ACoffset          0b00010000
#define Gain              0b00011000
//Channel
#define Current           0b00000001
#define Voltage           0b00000010
#define CurrentAndVoltage 0b00000110

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

typedef enum AfeGainI_e
{
	I_GAIN_10x = 0x00,
	I_GAIN_50x = 0x01

} AfeGainI_t;

typedef enum DO_Function_e
{
	DO_EPG    = 0,
	DO_P_SIGN = 4,
	DO_P_SUM_SIGN = 6,
	DO_Q_SIGN = 7,
	DO_Q_SUM_SIGN = 9,
	DO_V_ZERO_CROSSING = 11,
	DO_I_ZERO_CROSSING = 12,
	DO_HI_Z_DEFAULT = 14
}
DO_Function_t;

class CS5490{

private:
	bool _readOperationResult;

public:

	#if !(defined ARDUINO_NodeMCU_32S ) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_Node32s)
		SoftwareSerial *cSerial;
		CS5490(float mclk, int rx, int tx);
		CS5490(float mclk, int rx, int tx, int reset);
	#else
		HardwareSerial *cSerial;
		CS5490(float mclk);
		CS5490(float mclk, int reset);
	#endif

	uint32_t data[3]; //data buffer for read and write
	int selectedPage;
	float MCLK;
	int resetPin;

	void write(int page, int address, uint32_t value);
	void read(int page, int address);
	void instruct(int instruction);
	void begin(int baudRate);
	void clearSerialBuffer();
	double toDouble(int LBSpow, int MSBoption);
	uint32_t toBinary(int LSBpow, int MSBoption, double input);
	uint32_t concatData();
	bool areLastReadingOperationsSucceeded(void);
	uint32_t getRegChk(void);


	/*** Instructions ***/
	void reset();
	void standby();
	void wakeUp();
	void singConv();
	void contConv();
	void haltConv();
	void calibrate(uint8_t type, uint8_t channel);
	void sendCalibrationCommand(uint8_t type, uint8_t channel);
	void hardwareReset(void);
	bool checkInternalVoltageReference(void);
	void resolve(void);
	/*** Calibration ***/

	//Gain
	double getGainSys();
	double getGainV();
	double getGainI();
	double getGainT();
	void setGainSys(double value);
	void setGainV(double value);
	void setGainI(double value);
	void setGainT(double value);
	void setAfeGainI(AfeGainI_t AfeCurrentGain);

	//Offset
	double getDcOffsetV();
	double getDcOffsetI();
	double getAcOffsetI();
	double getOffsetT();
	void setDcOffsetV(double value);
	void setAcOffsetI(double value);
	void setDcOffsetI(double value);
	void setOffsetT(double value);

	//Others
	//double getPhaseCompensation(int mode, int phase);
	//double getCalibrationScale(int value);
	//void setPhaseCompensation(int mode, int phase);
	//void setCalibrationScale(int value);

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
	void setDOpinFunction(DO_Function_t DO_fnct, bool openDrain);
	/* Not implemented functions
	void setDO(int mode);
	-------------------------*/

	/* EXTRA METHOD */
	uint32_t readReg(int page, int address);
};

#endif
