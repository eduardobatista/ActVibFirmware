#include <Wire.h>         // I2C
#include <SPI.h>
#include <Ticker.h>       // Timing
#include <Preferences.h>  // Allows recording preferences in non-volatile memory
#include <math.h>         // Math

#include "DSPfuncs.h"
#include "Algorithms.h"
#include "Queue.h"

#include <MCP4725.h>
#include <ADS1X15.h>
#include "MPU6050A.h"
#include "SparkFunLSM6DS3.h"

#define BUF_SIZE 64
#define LED_BUILTIN 2

// SPI Settings
#define VSPI_MISO 19 
#define VSPI_MOSI 23 
#define VSPI_SCLK 18
#define VSPI_SS 5
static const int spiClk = 1000000; // 1 MHz

TwoWire WireA = TwoWire(0); 
TwoWire WireB = TwoWire(1); // Test: GPIO14 as SCL and GPIO13 as SDA 

Preferences prefs;      // Preferences!

// Defining loop tasks
TaskHandle_t reading1;
TaskHandle_t writing1;

// Struct for sensor's data
// struct sensorsData{
//   float xAcceLSM=0, yAcceLSM=0, zAcceLSM=0;
//   float xGyroLSM=0, yGyroLSM=0, zGyroLSM=0;
//   uint8_t MPUallData[17];
//   int16_t valadc;
// };
struct transmitData{
  uint8_t data[60];
  uint8_t nbytes;
};

int num = 20;  
//Queue used to exchange data between reading and writing tasks
// Queue<sensorsData> queue = Queue<sensorsData>(num); // Queue of max num 'sensorsData', where 'sensorsData' is a struct
Queue<transmitData> queue = Queue<transmitData>(num);

// sensorsData input;
// sensorsData output;
transmitData tdata;
transmitData wdata;

// General Settings -----------------------------------------------------------------------
const int mpu_sda_pin = 21; //D5; // definição do pino I2C SDA
const int mpu_scl_pin = 22; //D6; // definição do pino I2C SCL
// --------------------------------------------------------------------------------------

const float F_SAMPLE = 250.0;  // Sampling frequency (Hz)
const float T_SAMPLE = 0.004;  // Sampling period in seconds
const int T_SAMPLE_us = 4000;  // As integer in microseconds
// const float F_SAMPLE = 500.0;  // Sampling frequency (Hz)
// const float T_SAMPLE = 0.002;  // Sampling period in seconds
// const int T_SAMPLE_us = 2000;  // As integer in microseconds

// Declaring and array with two MPU instances at the I2C bus:
MPU6050A mpus[3] = { MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA)};
LSM6DS3 lsms[3] = { LSM6DS3(I2C_MODE,0x6B), LSM6DS3(I2C_MODE,0x6B), LSM6DS3(I2C_MODE,0x6B) };
int8_t imuenable[3] = {0,0,0};
int8_t imutype[3] = {0,0,0};
int8_t imubus[3] = {0,0,0};
int8_t imuaddress[3] = {0,0,0};

// Declaring array of MCPs:
MCP4725 mcps[2] = { MCP4725(0x61,&WireA), MCP4725(0x60,&WireA) };

// Declaring ADC:
uint8_t adcconfig[3] = {0,0,0};
bool adcenablemap[4] = {false,false,false,false}; 
uint8_t nextadc,lastadc = 0;
int16_t lastadcreadings[4] = {0,0,0,0};
ADS1115 adc(0x4B,&WireA);

/* 
   In the following, we have definitions for FIR filters used as
   secondary-path and feedback filters. 
   The lengths of the vector are a maximun values for the sake of memory allocation.
   The actual length of theses filters will be defined by software.
*/
int Nsec = 0; // filter length
float wsec[3000]; // Coef. vector
float xsec[3000]; // Input vector
FIRFilter filtsec = FIRFilter(1,&wsec[0],&xsec[0]); // FIR filter declaration
int Nfbk = 0;
float wfbk[3000];
float xfbk[3000];
FIRFilter filtfbk = FIRFilter(1,&wfbk[0],&xfbk[0]);
float xsec2[3000];
FIRFilter filtsec2 = FIRFilter(1,&wsec[0],&xsec2[0]); // Same coef. vector as filtsec, since filtsec and filtsec are never used together.

/*
   In the following, we have definitions of the active vibration control algorithms: FxNLMS and the propose TAFxNLMS 
*/
float wa[1000];
float xa[1000];
float xasec[1000];
FxNLMS fxnlms = FxNLMS(100,&wa[0],&xa[0],&xasec[0],&filtsec);
float wa2[1000];
float xa2[1000];
float xasec2[1000];
TAFxNLMS tafxnlms = TAFxNLMS(100, &wa[0], &xa[0], &wa2[0], &xa2[0], &xasec[0], &xasec2[0], &filtsec, &filtsec2);

// Two signal generators defined below, since we have two output channels.
// 1668 is chosen to have outputs between 0 and 10V, avoiding saturation.
SignalGenerator siggen[4] = {SignalGenerator(F_SAMPLE,T_SAMPLE,2048),SignalGenerator(F_SAMPLE,T_SAMPLE,2048),
                             SignalGenerator(F_SAMPLE,T_SAMPLE,128),SignalGenerator(F_SAMPLE,T_SAMPLE,128)};

// DC removal is needed when working with adaptive control algorithms.
// Definitions of simple IIR-based DC removers for the two inputs from accelerometers:
DCRemover dcr[2] = {DCRemover(0.95f),DCRemover(0.95f)};



bool reading = false; // Flag for continous reading mode on or off
bool controlling = false; // Flag for control mode on or off
bool algOn = false; // Flag indicating wheter the control algorithm is on or off

// Control parameters:
uint8_t algchoice = 0;  // 0 = FxNLMS is used for control, 2 = TAFxNLMS is used.
uint8_t idRefIMU = 0;
uint8_t idRefIMUSensor = 0; 
uint8_t idErrIMU = 1;
uint8_t idErrIMUSensor = 0; 
uint8_t canalperturb = 0;  // Definition of the actuator output used for generating the perturbation.
uint8_t canalcontrole = 1;  // Definition of the actuator output used for injecting the control signal in the beam.
// Control variables:
float xerr = 0.0;  // Error signal for the control algorithm (corresponds to the reading of the error accelerometer).
float xref = 0.0;  // Reference signal for the control algorithm (corresponds to the reading of the ref. accelerometer).
float xreff = 0.0;  // xref - (feedback filter output). This signal is the input for the control algorithm.
int dclevel = 0;
int satlevel = 0;
int outputaux = 0;  // Stores the corresponding integer value of the control signal before sending it to the DAC output. 
float lastout = 0;  // The last (float) value of the control signal, which feeds the feedback filter.
unsigned char ctrlflags = 0;  // Used for indicating that saturation of the control signal has occurred.

unsigned char cbuf[BUF_SIZE];  // Buffer for storing the byte values before sending them to the host computer.  
uint8_t cbuf2[20]; 

uint32_t Tsamplecycles = 1000;  // Sampling period in CPU cycles.
uint32_t nextperiod = 0;  // Stores the cycle counter value that corresponding to the next system event. Events happen after each sampling period. 


/* Write Outputs
    id = 0 -> mcps[0] (12 bits)
    id = 1 -> mcps[1] (12 bits)
    id = 2 -> GPIO25 (Eletroimã 3)
    id = 3 -> GPIO26 (Eletroimã 4)
*/
void writeOutput(int id,int val) {
    if (id < 2) {
      mcps[id].setValue(val & 0x0FFF);
    } else {
      // Output pins are 25 and 26, for ids equals to 2 and 3 respetively:
      dacWrite(23+id, val & 0xFF);
    }    
}

/*
    Reading configuration data stored in the flash memory. 
    Basically, the length and coefficients of both the secondary and feedback filters
    are stored in the memory. These values are obtained in a preliminary experiment and 
    are sent to the ESP32 from the host computer.
*/
void loadFlashData() {
  prefs.begin("AlgData");
  Nsec = prefs.getInt("AlgData");
  Nfbk = prefs.getInt("AlgData");
  prefs.getBytes("AlgData",&wsec[0],Nsec*sizeof(float));
  prefs.getBytes("AlgData",&wfbk[0],Nfbk*sizeof(float));
  prefs.end();
  filtsec.setMem(Nsec);
  filtsec2.setMem(Nsec);
  filtfbk.setMem(Nfbk);
  Serial.println("Flash data loaded...");
}

int8_t flaginitIMU = -1;
int8_t flagconfigIMU = -1;
int8_t flagadcconfig = -1;
int ctt = 0;
// Reading Task
void Reading(void * parameter){

  int timecounter;
  int auxxxx = 1;

    for (;;) {        

        if (flaginitIMU >= 0) {
          int IMUid = flaginitIMU;
            if (imutype[IMUid] == 0 ) {
              mpus[IMUid].initMPU();
              mpus[IMUid].readData(cbuf2);
              mpus[IMUid].checkMPU();
              // delayMicroseconds(50);
              if (mpus[IMUid].responseOk) { flaginitIMU = -1; } 
              else { flaginitIMU = -2;  }
            } else {
              // Serial.write(IMUid);
              status_t aux = lsms[IMUid].begin();
              if (aux != IMU_SUCCESS) { delay(1); aux = lsms[IMUid].begin(); } 
              if (aux != IMU_SUCCESS) { delay(1); aux = lsms[IMUid].begin(); }         
              if (aux == IMU_SUCCESS) {
                // Some readings seems to be need to kick start the automatic readings:
                lsms[IMUid].readRawAccelX();
                delay(1);
                lsms[IMUid].readRawTemp(); 
                delay(1);
                flaginitIMU = -1; 
              } else { flaginitIMU = -2; }            
            }
        } 
        
        if (flagconfigIMU >= 0) {
          int IMUid = flagconfigIMU;
          if (IMUid < 3) {            
            imuenable[IMUid] = cbuf[0] & 0x01;
            imutype[IMUid] = (cbuf[0]>>1) & 0x01;
            imubus[IMUid] = (cbuf[0]>>2) & 0x03;
            imuaddress[IMUid] = cbuf[1];
            if (imutype[IMUid] == 0) { // MPU6050
              if (imubus[IMUid] == 0) { mpus[IMUid].setI2C(&WireA); }
              else if (imubus[IMUid] == 1) { mpus[IMUid].setI2C(&WireB); }              
              mpus[IMUid].setAddress(imuaddress[IMUid]);
              mpus[IMUid].setAccelScale( cbuf[2] & 0x03 );
              mpus[IMUid].setGyroScale( ((cbuf[2]>>2) & 0x07) - 1 );
              mpus[IMUid].setFilter( (cbuf[2]>>5) & 0x07 );
            } else if (imutype[IMUid] == 1) { // LSM6DS3
              if (imubus[IMUid] < 2) { // I2C buses
                if (imubus[IMUid] == 0) { lsms[IMUid].setI2CBus(&WireA); }
                else if (imubus[IMUid] == 1) { lsms[IMUid].setI2CBus(&WireB); }
                lsms[IMUid].changeI2CAddress(imuaddress[IMUid]);
              } else { // SPI
                lsms[IMUid].setSPIMode(imuaddress[IMUid]);
                
              }
              lsms[IMUid].config((cbuf[2]>>2) & 0x07, cbuf[2] & 0x03, (cbuf[2]>>5) & 0x07);
              lsms[IMUid].readRawAccelX();
            }
          }          
          flagconfigIMU = -1;          
        }

        if (flagadcconfig >= 0) {   
          adc.setGain((1 << adcconfig[1]) >> 1);
          adc.setDataRate(adcconfig[2]);              
          if (adcconfig[0] > 0) {            
            adc.setMode(0);   
            nextadc = 0;
            do {
                nextadc = (nextadc+1) & 0x03;
            } while ( ((adcconfig[0] >> nextadc) & 0x01) == 0 );
            // adc.requestADC(nextadc); 
            adc.readADC(nextadc);
          } else {
            adc.setMode(1);
          }
          for (int iii = 0; iii < 4; iii++) {
            adcenablemap[iii] = (((adcconfig[0] >> iii) & 0x01) == 1);
          }
          if (adc.isConnected()) {
            flagadcconfig = -1;
          } else {
            flagadcconfig = -2;
          }
        }


        if (reading) {  // If Reading Mode is on:

            // Blocks until reaching the time (clock-cycle count) for the next sampling period: ------------ 
            while ( ((nextperiod-ESP.getCycleCount()) >> 31) == 0 ) { auxxxx = auxxxx + 2; }
            nextperiod = nextperiod + Tsamplecycles;
            // --------------------------------------------------------------------------------------------
        
            timecounter = ESP.getCycleCount();

            ctt = 0;
            // First 3 bytes for sync:
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            // Serial.write(0xF);
            // Serial.write(0xF);
            // Serial.write(0xF); 

            // IMU Readings:
            for (int id = 0; id < 3; id++) {
                if (imuenable[id] == 1) { 
                if (imutype[id] == 0) {
                    mpus[id].readData(&tdata.data[ctt]);
                    ctt = ctt + 14;
                    // Serial.write(mpus[id].buf,14); 
                } else {
                    lsms[id].readRegisterRegion(&tdata.data[ctt],0x22,12);
                    ctt = ctt + 12;
                    // Serial.write(cbuf2,12);
                }        
                }
            }

            // Signal Generators:
            for (int id = 0; id < 4; id++) {
                if (siggen[id].enabled) { writeOutput(id,siggen[id].next()); }
                else { writeOutput(id,0); }
                if (id < 2) {
                  tdata.data[ctt++] = (siggen[id].last >> 8) & 0x0F;
                  tdata.data[ctt++] = siggen[id].last & 0xFF;
                  // Serial.write((siggen[id].last >> 8) & 0x0F);
                  // Serial.write(siggen[id].last & 0xFF);
                } else {
                  tdata.data[ctt++] = siggen[id].last;
                  // Serial.write(siggen[id].last);  
                }        
            }

            // ADC Readings:
            if (adcconfig[0] > 0) {

                lastadcreadings[nextadc] = adc.getValue();
                lastadc = nextadc;
                do {
                  nextadc = (nextadc+1) & 0x03;
                } while ( ((adcconfig[0] >> nextadc) & 0x01) == 0 );
                if (lastadc != nextadc) {
                  adc.requestADC(nextadc);
                }
                for (int iii = 0; iii < 4; iii++) {
                  if ( adcenablemap[iii] ) {
                    tdata.data[ctt++] = lastadcreadings[iii] >> 8;
                    tdata.data[ctt++] = lastadcreadings[iii] & 0xFF;
                  }                  
                }

            } 
            //else {
                // tdata.data[ctt++] = 0;
                // tdata.data[ctt++] = 0;
                // Serial.write(0);
                // Serial.write(0);
            // }

            timecounter = ESP.getCycleCount() - timecounter;
            // Sample Time:
            timecounter = (timecounter >> 4) & 0xFFFF;
            tdata.data[ctt++] = (timecounter >> 8) & 0xFF;
            tdata.data[ctt++] = timecounter & 0xFF;
            // Serial.write((timecounter >> 8) & 0xFF);
            // Serial.write(timecounter & 0xFF);
            tdata.nbytes = ctt;
            // Serial.write(tdata.data,tdata.nbytes);
            queue.push(tdata);
        

        } else if (controlling) {  // If Control Mode is on:

            // Blocks until reaching the time (clock-cycle count) for the next sampling period: ------------ 
            while ( ((nextperiod-ESP.getCycleCount()) >> 31) == 0 ) { auxxxx = auxxxx + 2; }
            nextperiod = nextperiod + Tsamplecycles;
            // --------------------------------------------------------------------------------------------

            timecounter = ESP.getCycleCount(); 
            
            writeOutput(canalperturb,siggen[canalperturb].next());
            writeOutput(canalcontrole,outputaux);

            if (imutype[idRefIMU] == 0) {
                xref = dcr[0].filter(mpus[idRefIMU].readSensor(idRefIMUSensor));
            } else {
                xref = dcr[0].filter(lsms[idRefIMU].readSensor(idRefIMUSensor));
            }
            if (imutype[idErrIMU] == 0) {
                xerr = dcr[1].filter(mpus[idErrIMU].readSensor(idErrIMUSensor));
            } else {
                xerr = dcr[1].filter(lsms[idErrIMU].readSensor(idErrIMUSensor));
            }

            if (algOn) { 
                if (algchoice == 0) {
                fxnlms.update(xerr);        
                } else if (algchoice == 2) {
                tafxnlms.update(xerr);
                }
            }
            
            xreff = xref - filtfbk.filter(lastout);
            
            // if (algchoice == 0) {
            //   fxnlms.filter(xreff);
            //   lastout = fxnlms.y;
            // } else if (algchoice == 2) {
            //   tafxnlms.filter(xreff,filtfbk.y);
            //   lastout = tafxnlms.y;
            // } 
            if (algOn) { 
                if (algchoice == 0) {
                fxnlms.filter(xreff);
                lastout = fxnlms.y;
                } else if (algchoice == 2) {
                tafxnlms.filter(xreff,filtfbk.y);
                lastout = tafxnlms.y;
                } 
                outputaux = dclevel - ((int)round(lastout));
                if (outputaux > satlevel) { outputaux = satlevel; ctrlflags = 1; }
                else if (outputaux < 0) { outputaux = 0; ctrlflags = 1; }
                else { ctrlflags = 0; } 
            } else {
                outputaux = dclevel;
                ctrlflags = 0;
            }

            // writeOutput(canalperturb,siggen[canalperturb].next());
            // writeOutput(canalcontrole,outputaux);
            
            // Sending data to the host computer: --------------------------
            // The first three bytes are used for synchronization. 
            // Syncronization needs to be improved, but it is working fine.
            // Serial.write(0xF);
            // Serial.write(0xF);
            // Serial.write(0xF);
            // //Serial.write((*siggenperturb).last);
            // Serial.write(siggen[canalperturb].last >> 8);
            // Serial.write(siggen[canalperturb].last & 0xFF);
            // Serial.write(outputaux >> 8);
            // Serial.write(outputaux & 0xFF);
            // *(float *) &cbuf[0] = xref;
            // *(float *) &cbuf[4] = xerr;
            // Serial.write(&cbuf[0],4);
            // Serial.write(&cbuf[4],4);    
            // Serial.write(ctrlflags);
            // timecounter = (ESP.getCycleCount()-timecounter) >> 4;
            // Serial.write((timecounter >> 8 & 0xFF));
            // Serial.write(timecounter & 0xFF);
            ctt = 0;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = siggen[canalperturb].last >> 8;
            tdata.data[ctt++] = siggen[canalperturb].last & 0xFF;
            tdata.data[ctt++] = outputaux >> 8;
            tdata.data[ctt++] = outputaux & 0xFF;
            *(float *) &tdata.data[ctt] = xref;
            ctt = ctt + 4;
            *(float *) &tdata.data[ctt] = xerr;
            ctt = ctt + 4;
            tdata.data[ctt++] = ctrlflags;
            timecounter = (ESP.getCycleCount()-timecounter) >> 4;
            tdata.data[ctt++] = (timecounter >> 8 & 0xFF);
            tdata.data[ctt++] = timecounter & 0xFF;
            tdata.nbytes = ctt;

            queue.push(tdata);
            // --------------------------------------------------------------

        } else {

          // If not controlling nor reading, delay for a while.
          delay(1);

        }

    }

}

void Writing(void * parameter){

  int sr = 0;  // Stores commands read from the computer host.
  char hascmd = 0;  // Used for indicating if command from the computer host needs to be treated before accepting new commands.
  int cthascmd = 0;  // Indicates if the cmd has been treated. TODO: check if it is important or not, maybe could be changed to a flag. 

  int auxxxx = 1;

    for (;;) {

      while ( (Serial.available() == 0) && (queue.count() == 0) ) {
        delayMicroseconds(50);
      }

      if (queue.count() > 0) {
        wdata = queue.pop();
        Serial.write(wdata.data,wdata.nbytes);
      }

      /*
        Now, dealing with commmands from the computer host:
      */
      if ((hascmd == 0) && (Serial.available() > 0) ) {
        sr = Serial.read();
        if (sr == 'h') { // Handshake
          if (!reading && !controlling) { Serial.write('k'); }
        } 
        else if ((sr == 's') && !controlling) { // Starts reading mode.
          nextperiod = ESP.getCycleCount();
          reading = true;
          controlling = false;
        } 
        else if ((sr == 'S') && !reading) { // Starts control mode.    
          dcr[0].reset();
          dcr[1].reset();
          if (algchoice == 0) { fxnlms.reset(); }
          else if (algchoice == 2) { tafxnlms.reset(); }
          filtsec.reset();
          filtsec2.reset();
          filtfbk.reset();
          lastout = 0;
          // TODO: check the following
          dclevel = siggen[canalcontrole].Z_LEVEL;
          satlevel = dclevel * 2 - 1;
          outputaux = dclevel;
          nextperiod = ESP.getCycleCount();
          controlling = true;
          reading = false;
        }
        else if (sr == 't') { // Stops either reading or control mode.
          reading = false; 
          controlling = false;
        } 
        else if (sr == '!') { // Reads configuration data (treated afterwards).
          hascmd = '!';
        }
        else if (sr == 'a') { // AlgOn (treated afterwards).
          hascmd = 'a';
        }
        else if (sr == 'd') { // AlgOn (treated afterwards).
          hascmd = 'd';
        } 
        else if (sr == 'I') { // NEW! IMU config (treated afterwards).
          hascmd = 'I';
        }
        else if (sr == 'G') { // Generator settings.
          hascmd = 'G';
        }
        else if (sr == 'W') { // Grava caminho secundário ou de feedback
          if (!reading && !controlling) { hascmd = 'W'; }
        }
        else if (sr == 'P') { // Grava na memória não volátil os caminhos secundário e de feedback
          if (!reading && !controlling) {
            prefs.begin("AlgData");
            prefs.putInt("AlgData",Nsec);
            prefs.putInt("AlgData",Nfbk);
            prefs.putBytes("AlgData",&wsec[0],Nsec*sizeof(float));
            prefs.putBytes("AlgData",&wfbk[0],Nfbk*sizeof(float));
            prefs.end();
            Serial.print("ok!");
          }     
        } else if (sr == 'i') { // inicializa sensor escolhido
          if ((!reading) && (!controlling)){ 
            flaginitIMU = Serial.read();
            while (flaginitIMU >= 0) {
              delay(1);
            }
            if (flaginitIMU == -1) { Serial.write("ok!"); } 
            else { Serial.write("err"); }               
          }      
        }
        else if (sr == 'X') { // usado para teste do algoritmo adaptativo
          unsigned char aux[4];
          aux[0] = Serial.read();
          aux[1] = Serial.read();
          aux[2] = Serial.read();
          aux[3] = Serial.read();
          float xn = *(float *) aux;
          aux[0] = Serial.read();
          aux[1] = Serial.read();
          aux[2] = Serial.read();
          aux[3] = Serial.read();
          float dn = *(float *) aux;
          float en = 0;
          //en = dn - fxnlms.filter(xn);
          //fxnlms.update(en);
          if (!isnan(xn) && !isnan(dn)) {
            Serial.write('k');
            en = dn - filtsec.filter(fxnlms.filter(xn));
            fxnlms.update(en);   
            *(float *) aux = en; 
            Serial.write(aux,4);     
          } else {
            //Serial.write('!');
            //en = dn - fxnlms.filter(0);
            //fxnlms.update(0);
          }      
        }
        else if (sr == 'r') {
          lsms[0].setI2CBus(&WireA);
          lsms[0].changeI2CAddress(0x6B);
          for (int tt = 0; tt < 3; tt++) {
            lsms[0].begin();
            delay(2);
          }
          uint8_t readCheck;
          lsms[0].readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
          Serial.println(readCheck);
          Serial.println(lsms[0].commInterface);
          Serial.println(lsms[0].readFloatAccelX());
          Serial.println(lsms[0].readRawTemp());
          Serial.println(lsms[1].commInterface);
          Serial.println(lsms[2].commInterface);
        }
        
      }


      if (hascmd != 0) { // Late treatment of several commands.
        cthascmd++;
        delayMicroseconds(20);
        if (cthascmd > 100) {
          cthascmd = 0;
          hascmd = 0;
        } 
        else if (hascmd == 'I') { // NEW! IMU configuration.
          if (Serial.available() >= 4) {
            unsigned int IMUid = Serial.read();
            Serial.readBytes(cbuf,3);
            flagconfigIMU = IMUid;
            while (flagconfigIMU >= 0) {
              delay(1);
            }
            Serial.write("ok");            
            hascmd = 0;
            cthascmd = 0;
          }
        }       
        else if (hascmd == 'G') {
          if (Serial.available() >= 8) { // If command is G, check if five bytes are available. If they are not, try again at the next cycle.
            unsigned int idgerador = Serial.read();  
            unsigned int tipo = Serial.read();
            float amp = (float)((Serial.read() << 8) + Serial.read());
            float freq = (float)Serial.read();          
            freq = freq + ((float)Serial.read())/100.0;
            int dclevel = (Serial.read() << 8) + Serial.read();
            if (idgerador < 4) {
              siggen[idgerador].setType(tipo,amp,freq,dclevel);
              if (tipo == 2) {
                siggen[idgerador].setChirpParams(Serial.read(),Serial.read(),Serial.read(),
                                                Serial.read(),(Serial.read() << 8) + Serial.read());
              }
            }
            hascmd = 0;
            cthascmd = 0;
          }            
        }
        else if (hascmd == 'd') {
          if (Serial.available() >= 3) {  // If command is d, check if one byte is avaliable from the host computer. If it is not, try again at the next cycle.
            adcconfig[0] = Serial.read();
            adcconfig[1] = Serial.read();
            adcconfig[2] = Serial.read();  
            flagadcconfig = 1;
            while (flagadcconfig >= 0) { 
              // TODO: prevent blocking forever.
              delay(1);
            }
            if (flagadcconfig == -1) {
              Serial.write("ok");
            } else {
              Serial.write("er");
            }
            flagadcconfig = -1;            
            hascmd = 0;
            cthascmd = 0;  
          }
        }
        else if (hascmd == 'a') {
          if (Serial.available() >= 1) {  // If command is A, check if one byte is avaliable from the host computer. If it is not, try again at the next cycle.
            hascmd = 0;
            cthascmd = 0;
            unsigned char aux = Serial.read();
            if (aux == 0) { // Desativa
              algOn = false;
            } else if (aux == 1) { // Ativa
              algOn = true;
            } else if (aux == 2) { // Ativa com mudança de parâmetros
              hascmd = 'A';
            }
          }
        }
        else if (hascmd == 'A') {
          if (Serial.available() >= 8) {  // Eight bytes are available? If not, does not block and try again in the next cycle.
            hascmd = 0;
            cthascmd = 0;
            Serial.readBytes(cbuf,8);
            if (algchoice == 0) { // FxNLMS
              fxnlms.mu = *(float *) &cbuf[0];
              fxnlms.fi = *(float *) &cbuf[4];
            } else if (algchoice == 2) { // TAFxNLMS
              tafxnlms.mu = *(float *) &cbuf[0];
              tafxnlms.fi = *(float *) &cbuf[4];
            }
            algOn = true;
          }
        }
        else if (hascmd == 'W') {
          if (Serial.available() > 0) {        
            unsigned char * tptr;
            int nbytes,ct,natual;
            unsigned char type = Serial.read(); // Tipo de informação a gravar.
            Serial.readBytes(cbuf,2); //número de bytes a ser lido.
            nbytes = ((int)cbuf[0] << 8) + (int)cbuf[1];
            if ((type == 's') || (type == 'f')) {
              if (type == 's') { 
                tptr = (unsigned char *) wsec; 
                Nsec = nbytes >> 2;
              } else if (type == 'f') {
                tptr = (unsigned char *) wfbk; 
                Nfbk = nbytes >> 2;
              }
              Serial.write('k');
              ct = 0;
              while (ct < nbytes) {
                if (nbytes-ct-BUF_SIZE >= 0) { natual = BUF_SIZE; } 
                else { natual = nbytes-ct; }
                Serial.readBytes((tptr+ct),natual);
                Serial.write(natual >> 8);
                Serial.write(natual & 0x00FF);
                ct = ct + natual;
              }
            }       
            hascmd = 0;
            cthascmd = 0;
          }
        }
        else if (hascmd == '!') {
          if (Serial.available() >= 14) {
            Serial.readBytes(cbuf,14);          
            canalcontrole = cbuf[0] & 0x0F;
            canalperturb = (cbuf[0] >> 4) & 0x0F;
            idRefIMU = (cbuf[1] >> 4) & 0x0F;
            idRefIMUSensor = (cbuf[1] & 0x0F);
            idErrIMU = (cbuf[2] >> 4) & 0x0F;
            idErrIMUSensor = (cbuf[2] & 0x0F);
            algchoice = cbuf[3];
            if (algchoice == 0) { // FxNLMS
              fxnlms.setParameters(
                (((int)cbuf[4]) << 8) + (int)cbuf[5],
                *(float *) &cbuf[6],
                *(float *) &cbuf[10]
              );
            } else if (algchoice == 2) { // TAFxNLMS
              tafxnlms.setParameters(
                (((int)cbuf[4]) << 8) + (int)cbuf[5],
                *(float *) &cbuf[6],
                *(float *) &cbuf[10]
              );
            }
            delayMicroseconds(100);
            Serial.print("ok!");
            algOn = false;
            hascmd = 0;
            cthascmd = 0;
          }
        }
      }

    }

}


// Startup configuration:
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // Serial.begin(115200);
  // Serial.begin(230400);
  Serial.begin(500000);

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  pinMode(VSPI_SS, OUTPUT);

  adc.begin();

  WireA.begin();
  WireA.setClock(1000000L);
  WireB.begin(13,14,1000000L); // Test: GPIO14 as SCL and GPIO13 as SDA 
 
  mpus[0].setI2C(&WireA);
  mpus[0].initMPU();
  mpus[0].checkMPU();

  mpus[1].setI2C(&WireB);
  mpus[1].initMPU();
  mpus[1].checkMPU();

  mcps[0].begin();
  mcps[1].begin();


  siggen[0].setType(0,0,10,2048);
  siggen[1].setType(0,0,10,2048);
  siggen[2].setType(0,0,10,128);
  siggen[3].setType(0,0,10,128);

  loadFlashData();

  Tsamplecycles = T_SAMPLE_us * ESP.getCpuFreqMHz(); // Sampling period in clock cycles 
  nextperiod = ESP.getCycleCount();

    // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        Reading,            /* Task function          */
        "Reading Task",     /* Name of the task       */ 
        2048,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        8,                  /* Priority of the task   */
        &reading1,          /* Task handle to keep track of the task */
        0);                 /* Core 0 */
    delay(500); 

    disableCore0WDT();          // needed to start-up Reading task  

     // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        Writing,            /* Task function          */
        "Writing Task",     /* Name of the task       */ 
        2048,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        4,                  /* Priority of the task   */
        &writing1,          /* Task handle to keep track of the task */
        1);                 /* Core 0 */
    delay(500); 
     

    nextperiod = ESP.getCycleCount();
  
}

void loop() {

    // delay(100);
    vTaskDelay(100);
    // vTaskSuspend(NULL);

}