#include <CS5490.h>
#include <EEPROM.h>

#define rx 11 // Referred to Arduino, must be cross-connected to tx pin of the CS5490
#define tx 12
#define Reset 10

/* Choose your board */

/* Arduino UNO and ESP8622 */
CS5490 line(MCLK_default,rx,tx,Reset);

/* ESP and MEGA  (Uses Serial2)*/
//CS5490 line(MCLK_default,Reset);

// Comment out the following in order to activate tuning mode for AC Igain (shunt method is used in this example)
//#define TUNING_MODE_ACTIVE

// The following parameters are application dependent, therefore must be properly set according your external hardware
#define R_SHUNT_OHM   0.069  // Application dependent
#define V_ALFA        (1000.0/1689000.0) // Resistor partition ratio on V input: on CDB5490U 4 * 422kOhm + 1kOhm 
#define SYS_GAIN      1.25 
#define V_FS          0.6
#define V_FS_RMS_V    0.17676 // 250mVpp
#define V_MAX_RMS     (V_FS_RMS_V/V_ALFA)
#define I_FS          0.6
#define I_FS_RMS_V    0.17676 // 250mVpp with GAIN 10x, otherwise with GAIN 50x 50mVpp --> 0.3535 Vrms
#define I_MAX_RMS_A   (I_FS_RMS_V/R_SHUNT_OHM)
#define P_FS          0.36
#define P_COEFF       ((V_MAX_RMS * I_MAX_RMS_A) / (P_FS * SYS_GAIN * SYS_GAIN))
#define SAMPLE_COUNT_DEFAULT  4000 // Number of sample used to compute the output low rate computation (with quartz 4.096MHz --> 1 measure/s).

bool resetRequired;
uint32_t CriticalRegisterChecksum;

/* TUNING OPERATIONS */
bool CS5490_testReset(void)
{
  // Activate HPF on I current (to remove the DC component: it's an AC application)
  uint32_t reg = line.readReg(16,0);
  reg |= 0x0000000A;
  line.write(16,0,reg);
  
  // Set to single conversion in order to force CRC computing
  line.singConv();

	uint32_t regChk = line.getRegChk();
  if (!line.areLastReadingOperationsSucceeded()) return false;

  if(regChk != 0x00DD8BD6) return false;

  return true; // Reset OK
}


/* NORMAL OPERATIONS */

void CS5490_SendConfiguration(void)
{
  uint32_t IgainCalibrated;
  uint8_t calibrationData[7];

  // Look for calibration data stored in EEPROM
  uint8_t CRC = 0;
  for(uint8_t i=0; i<7;i++)
  {
    calibrationData[i] = EEPROM.read(i);
    //Serial.println(calibrationData[i], HEX);
    CRC ^=calibrationData[i];
  }

  if(!CRC) // EEPROM CRC OK
  {
    IgainCalibrated = ((uint32_t)calibrationData[0]) * 65536 + ((uint32_t)calibrationData[1]) * 256 + calibrationData[2];
    CriticalRegisterChecksum = ((uint32_t)calibrationData[3]) * 65536 + ((uint32_t)calibrationData[4]) * 256 + calibrationData[5];
    
    Serial.println("___CALIBRATION DATA READ FROM EEPROM___ ");
    Serial.print("   IgainCalibrated: ");
    Serial.println(IgainCalibrated, HEX);

    Serial.print("   CriticalRegisterChecksum: ");
    Serial.println(CriticalRegisterChecksum, HEX);

    // Send calibration data
    line.write(16,33,IgainCalibrated);
    uint32_t IgainCalibratedReadBack = line.readReg(16, 33);
    if (line.areLastReadingOperationsSucceeded() && IgainCalibratedReadBack == IgainCalibrated)
    {
      Serial.println("Igain written and verified");
    }
    else
    {
      Serial.println("Igain error!");
    }
  }
  else // No valid calibration parameters found in EEPROM --> APPLY DEFAULTS 0xDD8BD6
  {
    Serial.println("CALIBRATION DATA NOT FOUND: apply default CriticalRegisterChecksum: 0xDD8BD6");
    CriticalRegisterChecksum = 0xDD8BD6;
  }
  
  // Activate HPF on I current (Don't alterate critical registers checksum)
  uint32_t reg = line.readReg(16,0);
  reg |= 0x0000000A;
  line.write(16,0,reg);
  
  //line.setDOpinFunction(DO_V_ZERO_CROSSING, false); // Test the zero crossing feature

  // Set to continous conversion
  line.contConv();
}

void setup() {

  //Initializing communication with CS5490
  //600 is the default baud rate velocity.
   line.begin(baudRate_default);

  line.hardwareReset();
  resetRequired = false;
  
  //Initializing communication arduino/PC to show results in Monitor Serial
  Serial.begin(115200);
  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);

#ifndef TUNING_MODE_ACTIVE
  CS5490_SendConfiguration();
#endif
}

void loop() {

#ifdef TUNING_MODE_ACTIVE

  // The application is AC (mains voltage) with shunt resistor used to measure the current.
  // Perform the Igain calibration with a reference instruments (a calibrated digital power meter) in the condition of more less HALF LOAD 
  // Implementation based on "CIRRUS LOGIC AN366REV2" pag. 15 "Main Calibration Flow" https://statics.cirrus.com/pubs/appNote/AN366REV2.pdf
  // Calibration should be performed with PF=1 --> pure resistive load
  
  #define I_CAL_RMS_A  1.3126 // Moreless 1/2 max load
  #define SCALE_REGISTER_FRACTION  (0.6 * SYS_GAIN * (I_CAL_RMS_A / I_MAX_RMS_A)) // For not full load calibration
  #define SCALE_REGISTER_VALUE ((uint32_t)(SCALE_REGISTER_FRACTION * 0x800000)) 

  // Verify that reset is ok: HW RESET --> START SINGLE CONVERSION --> CHECK RESET CHECKSUM
  while(!CS5490_testReset())
  {
    Serial.println( "RESET issue... retry" );
    line.resolve();
  }

  Serial.println( "RESET OK! Default parameters applied." );

  line.write(18, 63, SCALE_REGISTER_VALUE); // Set scale register for not full load AC current gain calibration --> THIS MODIFY THE critical registers checksum!!!

  Serial.println( "Scale register set --> Start continuos conversion..." );

  line.contConv();

  delay(2000); // Wait 2 seconds

  line.haltConv();

  double rmsV = line.getRmsV();
  double freq = line.getFreq();
  double rmsI = line.getRmsI();
  double PF = line.getPF();
  double P = line.getAvgP();
  double Q = line.getAvgQ();
  double S = line.getAvgS();
   
  if(line.areLastReadingOperationsSucceeded())
  {
        Serial.print("Vac RMS: ");
        rmsV = (((rmsV/SYS_GAIN)/V_FS)*V_FS_RMS_V)/V_ALFA;
        Serial.print( rmsV, 2 );
        Serial.print(" [V] @ ");
        Serial.print(freq*SAMPLE_COUNT_DEFAULT, 2);
        Serial.println(" Hz");
        Serial.print("Iac RMS: ");
        rmsI  = (((rmsI/SYS_GAIN)/I_FS)*I_FS_RMS_V)/R_SHUNT_OHM;
        Serial.print( rmsI, 4 );
        Serial.println(" [A]");
        Serial.print("PF: ");
        Serial.println( PF, 4 );
        Serial.print("Active power: ");
        Serial.print( P * P_COEFF , 4 ); 
        Serial.println(" W");
        Serial.print("Reactive power: ");
        Serial.print( Q * P_COEFF, 4 );
        Serial.println(" VAR");
        Serial.print("Apparent power: ");
        Serial.print( S * P_COEFF, 4 );
        Serial.println(" VA");
        
        Serial.println();
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("Please check if measures are in the expected range, otherwise the calibration is invalid.");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println();

        bool tuningSubSequenceIsOK = true;

        line.write(16, 57, 0x1F40); // Set Tsettle to 2000ms
        uint32_t reg = line.readReg(16, 57);
        if (!line.areLastReadingOperationsSucceeded() || reg != 0x1F40) tuningSubSequenceIsOK = false;

        line.write(16, 51, 0x3E80); // Set sample count register to 16000
        reg = line.readReg(16, 51);
        if (!line.areLastReadingOperationsSucceeded() || reg != 0x3E80) tuningSubSequenceIsOK = false;

        reg = line.readReg(0, 23); // Status0 register (in order to manage the DRDY bit)
        if (!line.areLastReadingOperationsSucceeded()) tuningSubSequenceIsOK = false;
        line.write(0, 23, 0x00800000); // Clear DRDY by setting it
        uint32_t status0_reg_verify = line.readReg(0, 23);
        if (!line.areLastReadingOperationsSucceeded() || reg & 0x7FFFFF != status0_reg_verify) tuningSubSequenceIsOK = false;

        if(tuningSubSequenceIsOK)
        {
          line.sendCalibrationCommand(Gain, Current);  // Start current gain calibration
          
          // Wait for DRDY set, with 3000ms time-out
          unsigned long startMillis = millis();
          bool DRDY;
          do
          {
            reg = line.readReg(0, 23); // Status0 register (in order to manage the DRDY bit)
            if(line.areLastReadingOperationsSucceeded() && reg & 0x800000)
              DRDY = true; 
            else 
              DRDY = false;
          }
          while( !DRDY && ( (millis()-startMillis) < 3000));

          if(DRDY)
          {
            line.contConv();

            delay(2000);

            double rmsV = line.getRmsV();
            double freq = line.getFreq();
            double rmsI = line.getRmsI();
            double PF = line.getPF();
            uint32_t Igain = line.readReg(16, 33);
  
            // Power read always 0 in this calibration phase (experimental result)
            // double P = line.getAvgP();
            // double Q = line.getAvgQ();
            // double S = line.getAvgS();
              
            if(line.areLastReadingOperationsSucceeded())
            {
                  Serial.print("Vac RMS: ");
                  rmsV = (((rmsV/SYS_GAIN)/V_FS)*V_FS_RMS_V)/V_ALFA;
                  Serial.print( rmsV, 2 );
                  Serial.print(" [V] @ ");
                  Serial.print(freq*SAMPLE_COUNT_DEFAULT, 2);
                  Serial.println(" Hz");
                  Serial.print("Iac RMS: ");
                  rmsI  = (((rmsI/SYS_GAIN)/I_FS)*I_FS_RMS_V)/R_SHUNT_OHM;
                  Serial.print( rmsI, 4 );
                  Serial.println(" [A]");
                  Serial.print("PF: ");
                  Serial.println( PF, 4 );
  
                  // Power read always 0 in this calibration phase (experimental result)
                  // Serial.print("Active power: ");
                  // Serial.print( P * P_COEFF , 4 ); 
                  // Serial.println(" W");
                  // Serial.print("Reactive power: ");
                  // Serial.print( Q * P_COEFF, 4 );
                  // Serial.println(" VAR");
                  // Serial.print("Apparent power: ");
                  // Serial.print( S * P_COEFF, 4 );
                  // Serial.println(" VA");
                  
                  // Verify that reset is ok: HW RESET --> START SINGLE CONVERSION --> CHECK RESET CHECKSUM
                  while(!CS5490_testReset())
                  {
                    Serial.println( "RESETTING CS5490..." );
                    line.hardwareReset();
                  }

                  line.write(16, 33, Igain);
                  uint32_t IgainReadBack = line.readReg(16, 33);
                  if ( line.areLastReadingOperationsSucceeded() && IgainReadBack == Igain)
                  {
                    line.singConv(); // In order to compute the critical register checksum
                    delay(1000);

                    uint32_t regChk = line.getRegChk();
                    if(line.areLastReadingOperationsSucceeded())
                    {
                        Serial.print("I gain: "); 
                        Serial.println(Igain, HEX);
                        Serial.print("Critical registers checksum: "); // Must be compared with the one expected for the current configuration (stored in the eeprom). In case of mismatch --> reset CS5490
                        Serial.println(regChk, HEX);
                        Serial.println( "RESET OK! Tuned parameters applied." );

                        uint8_t calibrationData[6];
                        calibrationData[0] = (uint8_t)(Igain>>16); // MSB 
                        calibrationData[1] = (uint8_t)(Igain>>8); 
                        calibrationData[2] = (uint8_t)Igain;       // LSB
                        calibrationData[3] = (uint8_t)(regChk>>16); // MSB 
                        calibrationData[4] = (uint8_t)(regChk>>8); 
                        calibrationData[5] = (uint8_t)regChk;       // LSB

                        uint8_t CRC = 0;
                        for(uint8_t i=0; i<6;i++)
                        {
                          CRC ^= calibrationData[i];
                          EEPROM.update(i, calibrationData[i]);
                          delay(100);
                        }
                        EEPROM.update(6, CRC);
                        delay(100);

                        // EEPROM saved data verification
                        CRC = 0;
                        for(uint8_t i=0; i<7;i++)
                          CRC ^= EEPROM.read(i);
                        
                        if(!CRC)
                        {
                          Serial.println();
                          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                          Serial.println("Calibration data Igain has been saved in the EEPROM and will be restored at the normal startup");
                          Serial.println("Please check if measures are in the expected range, otherwise the calibration is invalid.");
                          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                          Serial.println();
                        }
                        else
                        {
                          Serial.println("Calibration data storage issue!\n Please try restart the calibration or replace the logic board\n");
                        }
                    }
                    else
                    {
                      Serial.print("Calibration aborted due to serial communication errors.\n Please try restart the calibration\n");
                    }  
                  }
                  else
                  {
                    Serial.print("Calibration aborted due to serial communication errors.\n Please try restart the calibration\n");
                  }
            }
            else
            {
              Serial.print("Calibration aborted due to serial communication errors.\n Please try restart the calibration\n");
            }
            

          }
          else
          {
            Serial.println("Calibration measuring time-out error\n Please try restart the calibration\n");
          }
        
        }
        else
        {
          Serial.print("Calibration aborted due to serial communication errors.\n Please try restart the calibration\n"); 
        }
  }
  else
  {
     Serial.print("Calibration aborted due to serial communication errors.\n Please try restart the calibration\n");  
  }
  


  for(;;);

#else 

   //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
   // Demo application: 
   // the evaluation board CDB5490 measures electrical parameters directly from the mains (max 300Vrms). 
   // The current is sensed using a shunt resistor between IN+ and IN- (J32) - J7 on IN- , J8 on IN+, J44 and J46 closed, J53 and J54 open
   // The voltage is connected between LINE and GND (J4) using the builtin attenuation network - J6 in line and J11 on GND, J45 open
   // UART connection with Arduino on J19 (TX, RX, RESET and GND1) - J20 in TX-DIGITAL position - R37, R38 and R43 must be removed to avoid collision with onboard uC
   // J23 on the RESET line must be closed.
   //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  static uint8_t communicationMaxRetries = 5;
  static uint8_t measureErrorDetectionCounter = 10;
  
  // Acquire data and control that read operations are all OK
  double rmsV = line.getRmsV();
  double freq = line.getFreq();
  double rmsI = line.getRmsI();
  double PF = line.getPF();
  double P = line.getAvgP();
  double Q = line.getAvgQ();
  double S = line.getAvgS();
	uint32_t regChk = line.getRegChk();
  bool internalVoltageReferenceOK = line.checkInternalVoltageReference();

  // If all data are valid and the meter configuration is not corrupted, print results
  if(line.areLastReadingOperationsSucceeded())
  {
    if(regChk == CriticalRegisterChecksum)
    {
      Serial.print("Vac RMS: ");
      rmsV = (((rmsV/SYS_GAIN)/V_FS)*V_FS_RMS_V)/V_ALFA;
      Serial.print( rmsV, 2 );
      Serial.print(" [V] @ ");
      Serial.print(freq*SAMPLE_COUNT_DEFAULT, 2);
      Serial.println(" Hz");
      Serial.print("Iac RMS: ");
      
      rmsI  = (((rmsI/SYS_GAIN)/I_FS)*I_FS_RMS_V)/R_SHUNT_OHM;
      
      Serial.print( rmsI, 4 );
      Serial.println(" [A]");
      Serial.print("PF: ");
      Serial.println( PF, 4 );
      Serial.print("Active power: ");
      Serial.print( P * P_COEFF , 4 ); 
      Serial.println(" W");
      Serial.print("Reactive power: ");
      Serial.print( Q * P_COEFF, 4 );
      Serial.println(" VAR");
      Serial.print("Apparent power: ");
      Serial.print( S * P_COEFF, 4 );
      Serial.println(" VA");
      Serial.print("Critical registers checksum: "); // Must be compared with the one expected for the current configuration (stored in the eeprom). In case of mismatch --> reset CS5490
      Serial.println(regChk, HEX);

      if(internalVoltageReferenceOK)
        Serial.println("Voltage reference OK");
      else
      {
        Serial.println("Voltage reference drift ERROR - CS5490 hardware reset required!");
        resetRequired = true;
      }

      if(rmsV==0 && rmsI==0) // Sometimes the meter could enter some meta-state, reporting all 0s
      {
        Serial.println("Potential measuring error - CS5490 hardware reset required!");
        measureErrorDetectionCounter--;
        if(!measureErrorDetectionCounter)
          resetRequired = true;  
      }
    }
    else
    {
      // Critical registes checksum error
      Serial.println("Critical registes checksum error - CS5490 hardware reset required!");
      Serial.print("regChk: ");
      Serial.println(regChk, HEX);
      Serial.print("CriticalRegisterChecksum: ");
      Serial.println(CriticalRegisterChecksum, HEX);
      resetRequired = true;  
    }
    
  }
  else
  {
   Serial.println("Communication error - no reply!"); 
   communicationMaxRetries--;
   if(!communicationMaxRetries)
          resetRequired = true; 
  }

  if(resetRequired)
  {
      resetRequired = false;
      measureErrorDetectionCounter = 10;
      communicationMaxRetries = 5;

      line.resolve();   
      CS5490_SendConfiguration();
      
      Serial.println("RESET PERFORMED");
  }

  Serial.println("");
  delay(1000);
  #endif
}
