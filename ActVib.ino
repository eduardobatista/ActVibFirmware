#include <Wire.h>         // I2C
#include <SPI.h>
#include <math.h>         // Math

#include <SPIFFS.h>

#include "DSPfuncs.h"
#include "Algorithms.h"
#include "Queue.h"

#include <MCP4725.h>
#include <ADS1X15.h>
#include "MPU6050A.h"
#include "LSM6DS3ESP32.h"

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

hw_timer_t *timer0cfg = NULL;

uint32_t baudrate = 500000;

// Defining loop tasks
TaskHandle_t reading1;
TaskHandle_t writing1;

struct transmitData{
  uint8_t data[60];
  uint8_t nbytes;
};

// Queues:
Queue<float> controlqueue = Queue<float>(2);
Queue<uint64_t> tcount2queue = Queue<uint64_t>(3);
Queue<uint8_t> lastimureadings[3] = {Queue<uint8_t>(28),Queue<uint8_t>(28),Queue<uint8_t>(28)};
Queue<uint16_t> adcreadings = Queue<uint16_t>(8);


// Timing variables: -----------------------------------------------------------------------------
float F_SAMPLE = 250.0;  // Sampling frequency (Hz)
float T_SAMPLE = 0.004;  // Sampling period in seconds
uint16_t T_SAMPLE_us = 4000;  // Sampling period in microseconds
uint32_t T_SAMPLE_cycles = 960000;  // Sampling period in cycles
TickType_t xFrequency0 = 4;   // Main process will run at 4 ticks = 4 ms period
TickType_t xLastWakeTime0; 
TickType_t xFrequency1 = 1;  // Second process may be sped up by changing this value to 1 or 2; Default is 4.
TickType_t xLastWakeTime1;
uint8_t CTTRatio = 4;
// ------------------------------------------------------------------------------------------------

EventGroupHandle_t xEventGroup;

// Declaring and array with two MPU instances at the I2C bus:
MPU6050A mpus[3] = { MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA) };
LSM6DS3ESP32 lsms[3] = { LSM6DS3ESP32(0x6B), LSM6DS3ESP32(0x6B), LSM6DS3ESP32(0x6B)};
int8_t imuenable[3] = {0,0,0};
int8_t imutype[3] = {0,0,0};
int8_t imubus[3] = {0,0,0};
int8_t imuaddress[3] = {0,0,0};
uint8_t imuextra[3] = {0,0,0};

// Declaring array of MCPs:
MCP4725 mcps[2] = { MCP4725(0x61,&WireA), MCP4725(0x60,&WireA) };

// Declaring ADC:
uint8_t adcconfig[3] = {0,0,0};
bool adcenablemap[4] = {false,false,false,false}; 
uint8_t adcsel = 0;
uint8_t adctype = 0; // 0 for ADS1115 or 1 for ADS1015
ADS1115 adc11(0x4B,&WireA);
ADS1015 adc10(0x4B,&WireA);
ADS1X15* adc;
int8_t flaginitADC = 0;

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
   In the following, we have definitions of the active vibration control algorithms: FxNLMS and the proposed CVAFxNLMS 
*/
float wa[1000];
float xa[1000];
float xasec[1000];
float deltawa[1000];
FxNLMS fxnlms = FxNLMS(100,&wa[0],&xa[0],&xasec[0],&filtsec,&deltawa[0]);
float wa2[1000];
float xa2[1000];
float xasec2[1000];
float deltawa2[1000];
CVAFxNLMS cvafxnlms = CVAFxNLMS(100, &wa[0], &xa[0], &wa2[0], &xa2[0], &xasec[0], &xasec2[0], &filtsec, &filtsec2, &deltawa[0], &deltawa2[0]);

// Four signal generators defined below, since we have four output channels.
SignalGenerator siggen[4] = {SignalGenerator(F_SAMPLE,T_SAMPLE),SignalGenerator(F_SAMPLE,T_SAMPLE),
                             SignalGenerator(F_SAMPLE,T_SAMPLE),SignalGenerator(F_SAMPLE,T_SAMPLE)};
OutputScaler outscaler[4] = {OutputScaler(2048),OutputScaler(2048),OutputScaler(128),OutputScaler(128)};
bool flagpwm[2] = {false,false}; // Only channels 3 and 4 are allowed.
uint8_t pwmduty[2] = {0,0};


// DC removal is needed when working with adaptive control algorithms.
// Definitions of simple IIR-based DC removers for the two inputs from accelerometers:
DCRemover dcr[2] = {DCRemover(0.95f),DCRemover(0.95f)};

bool predistenable[4] = {false,false,false,false};
uint8_t predistorders[4] = {1,1,1,1};
float predistcoefs[40] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                          0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fusionweights[2] = {0.5,0.5};

bool reading = false; // Flag for continous reading mode on or off
bool controlling = false; // Flag for control mode on or off
bool algOn = false; // Flag indicating wheter the control algorithm is on or off

// Control parameters:
uint8_t ctrltask = 0;  // 0 for Control, 1 for Path Modeling
bool debugmode = false; // true for enabling the Debug Mode (mechanical system is simulated at the software).
bool nextdebugstep = false;  // helps controlling the debug mode
uint8_t algchoice = 0;  // 0 = FxNLMS is used for control, 1 = FxNLMS with full buffers, 2 = CVA-FxNLMS is used, 3 = CVA-FxNLMS with full buffers.
uint32_t memsize = 100;
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
int lastsenddataaux = 0;
float lastout = 0;  // The last (float) value of the control signal, which feeds the feedback filter.7
unsigned char ctrlflags = 0;  // Used for indicating that saturation of the control signal has occurred.

unsigned char cbuf[BUF_SIZE];  // Buffer for storing the byte values before sending them to the host computer.  
uint8_t cbuf2[20]; // Stores temporary data from commmunication buses.


/* Write Outputs
    id = 0 -> mcps[0] (12 bits)
    id = 1 -> mcps[1] (12 bits)
    id = 2 -> GPIO25 (Eletroimã 3 - 8 bits)
    id = 3 -> GPIO26 (Eletroimã 4  - 8 bits)
*/
void writeOutput(int id, float valf) {
    if (valf > 1.0) { valf = 1.0; }
    else if (valf < -1.0) { valf = -1.0; }  
    int val;
    if (predistenable[id]) { 
      outscaler[id].evalOut(valf);  // This one registers the converted value (without predistortion).
      valf = valf * evalPoly(fabsf(valf),predistorders[id],&predistcoefs[10*id]);  // Adjust sample amplitude.
      val = outscaler[id].dclevel - outscaler[id].evalOutNoReg(valf);
    } else {
      val = outscaler[id].dclevel - outscaler[id].evalOut(valf);
    }
    if (id < 2) {      
      mcps[id].setValue(val & 0x0FFF);
    } else {      
      dacWrite(23+id, val & 0xFF);
    }
}

void zeroOutput(int id) {
    outscaler[id].lastwrittenout = 0;
    if (id < 2) {      
      mcps[id].setValue(0);
    } else {      
      dacWrite(23+id, 0);
    }
}

void writeOutputDebug(int id, float valf) {    
    if (valf > 1.0) { valf = 1.0; }
    else if (valf < -1.0) { valf = -1.0; }  
    int val = outscaler[id].dclevel - outscaler[id].evalOut(valf);
    Serial.write((val >> 8) & 0xFF);
    Serial.write(val & 0xFF);
}
void zeroOutputDebug(int id) {
    outscaler[id].lastwrittenout = 0;
    int val = 0;
    Serial.write((val >> 8) & 0xFF);
    Serial.write(val & 0xFF);
}

/*
    Initialization of IMU, involving definitions and connection check.
     - Parameter IMUid is de number of the IMU sensor (from 0 to 2).
     - Returns -1 in case of success and -2 in case of error.
*/
int8_t initIMU(uint8_t IMUid) {  
  
  // General IMU configuration:
  if (imutype[IMUid] == 0) { // MPU6050
    if (imubus[IMUid] == 0) { mpus[IMUid].setI2C(&WireA); }
    else if (imubus[IMUid] == 1) { mpus[IMUid].setI2C(&WireB); }              
    mpus[IMUid].setAddress(imuaddress[IMUid]);
    mpus[IMUid].setAccelScale( imuextra[IMUid] & 0x03 );
    mpus[IMUid].setGyroScale( ((imuextra[IMUid]>>2) & 0x07) - 1 );
    mpus[IMUid].setFilter( (imuextra[IMUid]>>5) & 0x07 );
  } else if (imutype[IMUid] == 1) { // LSM6DS3
    if (imubus[IMUid] < 2) { // I2C buses
      if (imubus[IMUid] == 0) { lsms[IMUid].setI2CBus(&WireA); }
      else if (imubus[IMUid] == 1) { lsms[IMUid].setI2CBus(&WireB); }
      lsms[IMUid].changeI2CAddress(imuaddress[IMUid]);
    } else { // SPI
      lsms[IMUid].setSPIMode(imuaddress[IMUid]);      
    }
    if (CTTRatio == 255) {
      uint8_t ODR = 0x00;
      if (T_SAMPLE_us < 1300) { ODR = 0x07; }
      else if (T_SAMPLE_us < 2500) { ODR = 0x06; }
      else if (T_SAMPLE_us < 5000) { ODR = 0x05; } 
      else { ODR = 0x04; }
      lsms[IMUid].config((imuextra[IMUid]>>2) & 0x07, imuextra[IMUid] & 0x03,
                                              (imuextra[IMUid]>>5) & 0x07, ODR);
    } else {
      lsms[IMUid].config((imuextra[IMUid]>>2) & 0x07, imuextra[IMUid] & 0x03,
                                              (imuextra[IMUid]>>5) & 0x07, 0x07);
    }    
    // lsms[IMUid].readSensor(0);
  }

  // IMU connection check:
  if (imutype[IMUid] == 0 ) { // MPU6050
    mpus[IMUid].initMPU();
    mpus[IMUid].readData(cbuf2);
    mpus[IMUid].checkMPU();
    if (mpus[IMUid].responseOk) { return -1; } 
    else { return -2;  }
  } else { // LSM6DS3
    status_t aux = lsms[IMUid].begin();
    if (aux != IMU_SUCCESS) { delay(1); aux = lsms[IMUid].begin(); } 
    if (aux != IMU_SUCCESS) { delay(1); aux = lsms[IMUid].begin(); }         
    if (aux == IMU_SUCCESS) {
      // Some readings seem to be need to kick start the automatic readings:
      lsms[IMUid].readSensor(0);
      delay(1);
      lsms[IMUid].readRawTemp(); 
      delay(1);
      return -1; 
    } else { return aux; }            
  }

}


/*
    Initialization of the ADC, involving definitions and connection check.
     - Returns 0 in case of success and -1 in case of error (sensor not found).
*/
int8_t initADC(void) {
    adc->setGain((1 << adcconfig[1]) >> 1);
    adc->setDataRate(adcconfig[2]); 
    adcsel = 0;
    for (int iii = 0; iii < 4; iii++) {
      adcenablemap[iii] = (((adcconfig[0] >> iii) & 0x01) == 1);
      if (adcenablemap[iii] == 1) { adcsel = iii; }
    }            
    if ((adcconfig[0] & 0x0F) > 0) {            
      adc->setMode(0);
      adc->readADC(adcsel);
    } else {
      adc->setMode(1);
    }             
    if (adc->isConnected()) {
      return 0;
    } else {
      return -1;
    }   
}


/* 
  Receive data regarding the paths via serial port.
*/
void getPaths() {     
  // TODO: Implement TimeOuts to avoid locking     
  unsigned char * tptr;
  int nbytes,ct,natual;
  unsigned char type = Serial.read(); // Tipo de informação a gravar.
  Serial.readBytes(cbuf,2); //número de bytes a ser lido.
  nbytes = ((int)cbuf[0] << 8) + (int)cbuf[1];
  if ((type == 's') || (type == 'f')) {
    if (type == 's') { 
      tptr = (unsigned char *) wsec; 
      Nsec = nbytes >> 2;
      filtsec.setMem(Nsec);
      filtsec2.setMem(Nsec);
    } else if (type == 'f') {
      tptr = (unsigned char *) wfbk; 
      Nfbk = nbytes >> 2;
      filtfbk.setMem(Nfbk);
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
}

/*
    Reading configuration data stored in the flash memory. 
    Basically, the length and coefficients of both the secondary and feedback filters
    are stored in the memory. These values are obtained in a preliminary experiment and 
    are sent to the ESP32 from the host computer.
*/
void loadFlashData() {
  File file = SPIFFS.open("/wsec.dat",FILE_READ);
  if (!file) {
    Serial.println("Sec path not found.");
  } else {
    Nsec = (int)(file.size() / sizeof(float));
    file.read((uint8_t *)&wsec[0],file.size());
    Serial.println("Success reading wsec.");
    Serial.println(Nsec);
    file.close();
    filtsec.setMem(Nsec);
    filtsec2.setMem(Nsec);  
  }
  file = SPIFFS.open("/wfbk.dat",FILE_READ);
  if (!file) {
    Serial.println("Feedback not found in mem.");
  } else {
    Nfbk = (int)(file.size() / sizeof(float));
    file.read((uint8_t *)&wfbk[0],file.size());
    Serial.println("Success reading wfbk.");
    Serial.println(Nfbk);
    file.close();
    filtfbk.setMem(Nfbk);
  }
  file = SPIFFS.open("/predist.dat",FILE_READ);
  if (!file) {
    Serial.println("Predist not found in mem.");
  } else {
    file.read(&predistorders[0],4);
    for (int i = 0; i < 4; i++) {
      if (predistorders[i] == 0) { predistenable[i] = false; }
      else { predistenable[i] = true; }
    }
    file.read((uint8_t *)&predistcoefs[0],160);
    file.read((uint8_t *)&fusionweights[0],8);
    file.close();
    Serial.println("Success reading predist.");
  }
}

/*
  Definition of variables required for task management, communication, etc.
*/ 
int8_t flaginitIMU = -1; 
uint8_t cttcycle = 0; 
bool flagzeroed = false;

uint64_t nextsampletime = 0;
uint32_t cnextsampletime = 0;


/* ===== Auxiliary Task (Core 1 - Former Reading Task) ===================================

- In **standy mode**, this task checks if IMU in bus I2C-1 needs to be initialized
  or if ADC needs to be initialized. If needed, initialize these elements. Otherwise,
  delays for 1 ms.

- In **reading mode**, this task runs at 1 kHz executing the following steps:
    -- At each four cycles (250 Hz), IMUs from bus I2C-1 are read.
    -- Generator outputs are written (at every cycle - 1 kHz)
    -- ADC is read (at every cycle - 1 kHz)
    -- Elapsed time is evaluated (at each four cycles - 250 Hz), which results in
       worst case elapsed time for task.
    -- A notification to MAINTASK is sent (at each four cycles - 250 Hz - since TASK 1
       runs at 250 Hz). WARNING: AUXTASK and MAINTASK need to be synchronized, meaning that
       AUXTASK must have cttcycle = 0 when MAINTASK runs.

- In **controlling mode**, this task runs at 250 Hz, executing the following steps:
    -- Writes perturbation and control outputs.
    -- Read IMUs in I2C-1 bus.
    -- Notifies MAINTASK so it can continue.
    -- Measures time of execution (timecounter2).

=====================================================================================*/ 
void AuxTask(void * parameter){

  uint64_t timecounter2;
  uint8_t imuaux[14];
  EventBits_t uxBits;
  uint64_t timeraux = 0;
  uint64_t timeleft = 0;
  uint32_t ccountaux = 0;
  uint32_t ctimeraux = 0;
  uint32_t ctimeleft = 0;

    for (;;) {        

        if (reading) {  // If Reading Mode is on:

          if (CTTRatio == 255) {

            // New Version:

            ctimeleft = cnextsampletime - ESP.getCycleCount(); // Time left for next sample
            do {              
              // If time left is "positive" and it is worth to use a delay in microseconds:
              if ( ((ctimeleft & 0x80000000) == 0) && ((ctimeleft >> 8) > 0) ) {              
                delayMicroseconds( (ctimeleft >> 8) ); // ctimeleft/256 is always smaller than timeleft/240. Thus, this delay will always be smaller than time left.
              }
              ctimeleft = cnextsampletime - ESP.getCycleCount();
            } while ((ctimeleft & 0x80000000) == 0);
            cnextsampletime = cnextsampletime + T_SAMPLE_cycles;

            uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask

            // Generator outputs:
            for (int id = 0; id < 4; id++) {
                if (siggen[id].enabled) { 
                  siggen[id].next();
                  writeOutput(id,siggen[id].lastf); 
                }
                else { zeroOutput(id); }
            }

            // ADC readings:
            if ((adcconfig[0] & 0x0F) > 0) {
                adcreadings.clear();
                adcreadings.push(adc->getValue());
                adc->requestADC(adcsel); 
            }    
             
            // Reading IMUS in the main I2C Bus:
            for (int id = 0; id < 3; id++) {
                if (imuenable[id] == 1) {  
                  if (imubus[id] == 0) {
                    if (imutype[id] == 0) {
                      mpus[id].readData(&imuaux[0]);
                    } else {
                      lsms[id].readRegisterRegion(&imuaux[0],0x22,12);                      
                    }
                    lastimureadings[id].clear();
                    for (int idd = 0; idd < 14; idd++) {
                      lastimureadings[id].push(imuaux[idd]);
                    }                    
                  }                        
                }
            }
            uxBits = xEventGroupSetBits(xEventGroup, 0x02);
            uxBits = xEventGroupWaitBits(xEventGroup, 0x04, pdTRUE, pdFALSE, 4); // Wait notification from MainTask


          } else {

            // Old version, kept for compatibility purposes:

            do {
              timeraux = timerReadMicros(timer0cfg);
              if (nextsampletime > timeraux) { 
                timeleft = nextsampletime - timeraux;
                delayMicroseconds(timeleft);
              } else { timeleft = 0; }
              timeraux = timerReadMicros(timer0cfg);
            } while (timeraux < nextsampletime);

            nextsampletime = nextsampletime + 1000;

            if ( cttcycle == 0 ) {    
              tcount2queue.clear();
              timeraux = timerReadMicros(timer0cfg);
              tcount2queue.push(timeraux-timecounter2);
              timecounter2 = timeraux;
              tcount2queue.push(timecounter2);
              ccountaux = ESP.getCycleCount();
              uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask
            }

            // ADC readings:
            if ((adcconfig[0] & 0x0F) > 0) {
              if (adcreadings.count() < CTTRatio) {  // TODO: change the 4 here to accomodate other sampling rates.
                adcreadings.push(adc->getValue());
                adc->requestADC(adcsel); 
              }
            }  

            // Generator outputs:
            for (int id = 0; id < 4; id++) {
                if (siggen[id].enabled) { 
                  siggen[id].next();
                  writeOutput(id,siggen[id].lastf); 
                }
                else { zeroOutput(id); }
            }     

            if ( cttcycle == 0 ) {    
              // Reading IMUS in the main I2C Bus:
              for (int id = 0; id < 3; id++) {
                  if (imuenable[id] == 1) {  
                    if (imubus[id] == 0) {
                      if (imutype[id] == 0) {
                        mpus[id].readData(&imuaux[0]);
                      } else {
                        lsms[id].readRegisterRegion(&imuaux[0],0x22,12);                      
                      }
                      lastimureadings[id].clear();
                      for (int idd = 0; idd < 14; idd++) {
                        lastimureadings[id].push(imuaux[idd]);
                      }                    
                    }                        
                  }
              }
              // tcount2queue.push(timerReadMicros(timer0cfg) - timecounter2);
              tcount2queue.push( (uint64_t)((ESP.getCycleCount() - ccountaux)>>8) );
              uxBits = xEventGroupSetBits(xEventGroup, 0x02);
              // uxBits = xEventGroupWaitBits(xEventGroup, 0x04, pdTRUE, pdFALSE, 20); // Wait notification from MainTask
            }

            cttcycle++;
            if (cttcycle >= CTTRatio) { cttcycle = 0; }

          }
            
        } else if (controlling && !debugmode) {  // If Control Mode is on:

          if (CTTRatio == 255) {

            // New version:
            ctimeleft = cnextsampletime - ESP.getCycleCount(); // Time left for next sample
            do {              
              // If time left is "positive" and it is worth to use a delay in microseconds:
              if ( ((ctimeleft & 0x80000000) == 0) && ((ctimeleft >> 8) > 0) ) {              
                delayMicroseconds( (ctimeleft >> 8) ); // ctimeleft/256 is always smaller than timeleft/240. Thus, this delay will always be smaller than time left.
              }
              ctimeleft = cnextsampletime - ESP.getCycleCount();
            } while ((ctimeleft & 0x80000000) == 0);
            cnextsampletime = cnextsampletime + T_SAMPLE_cycles;

            controlqueue.clear();

            // Writing outputs:
            if (ctrltask == 0) {  // If controlling:       
              writeOutput(canalperturb,siggen[canalperturb].lastf);
              writeOutput(canalcontrole,lastout); // TODO: WARNING!!!
            } else {  // If path modeling:
              zeroOutput(canalperturb);
              writeOutput(canalcontrole,siggen[canalcontrole].lastf);
            }

            uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask  

            // Read sensors from bus I2C-1, filter readings for DC removal, 
            // put readings at controlqueue.
            if (imubus[idRefIMU] == 0) {
              if (imutype[idRefIMU] == 0) {
                controlqueue.push(dcr[0].filter(mpus[idRefIMU].readSensor(idRefIMUSensor)));
              } else {
                controlqueue.push(dcr[0].filter(lsms[idRefIMU].readSensor(idRefIMUSensor)));
              }
            }
            if (imubus[idErrIMU] == 0) {
              if (imutype[idErrIMU] == 0) {
                controlqueue.push(dcr[1].filter(mpus[idErrIMU].readSensor(idErrIMUSensor)));
              } else {
                controlqueue.push(dcr[1].filter(lsms[idErrIMU].readSensor(idErrIMUSensor)));
              }
            }
            
            // Notifies TASK 1:
            uxBits = xEventGroupSetBits(xEventGroup, 0x02); // Set Bit 0 to unlock MainTask
            uxBits = xEventGroupWaitBits(xEventGroup, 0x04, pdTRUE, pdFALSE, 4); // Wait notification from MainTask

          
          } else {

            // Blocks until reaching the time for next sample: --------------------------------------------
            vTaskDelayUntil(&xLastWakeTime1, xFrequency0);
            // --------------------------------------------------------------------------------------------

            tcount2queue.clear();
            tcount2queue.push(timerReadMicros(timer0cfg)-timecounter2);
            timecounter2 = timerReadMicros(timer0cfg);
            tcount2queue.push(timecounter2);
            
            controlqueue.clear();

            // Writing outputs:
            if (ctrltask == 0) {  // If controlling:       
              writeOutput(canalperturb,siggen[canalperturb].lastf);
              writeOutput(canalcontrole,lastout); // TODO: WARNING!!!
            } else {  // If path modeling:
              zeroOutput(canalperturb);
              writeOutput(canalcontrole,siggen[canalcontrole].lastf);
            }

            uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask            

            // Read sensors from bus I2C-1, filter readings for DC removal, 
            // put readings at controlqueue.
            if (imubus[idRefIMU] == 0) {
              if (imutype[idRefIMU] == 0) {
                controlqueue.push(dcr[0].filter(mpus[idRefIMU].readSensor(idRefIMUSensor)));
              } else {
                controlqueue.push(dcr[0].filter(lsms[idRefIMU].readSensor(idRefIMUSensor)));
              }
            }
            if (imubus[idErrIMU] == 0) {
              if (imutype[idErrIMU] == 0) {
                controlqueue.push(dcr[1].filter(mpus[idErrIMU].readSensor(idErrIMUSensor)));
              } else {
                controlqueue.push(dcr[1].filter(lsms[idErrIMU].readSensor(idErrIMUSensor)));
              }
            }
            
            // Notifies TASK 1:
            tcount2queue.push(timerReadMicros(timer0cfg) - timecounter2);
            uxBits = xEventGroupSetBits(xEventGroup, 0x02); // Set Bit 0 to unlock MainTask
          
          }

        } else if (controlling && debugmode) {

          vTaskDelay((TickType_t)4);

        } else {

          // Some tasks that are done only when not controlling nor reading:

          // IMU Initialization:
          if (flaginitIMU >= 0) { 
            flaginitIMU = initIMU(flaginitIMU);
          }           

          if (flaginitADC == 1) {
            flaginitADC = initADC();
          }

          if (!flagzeroed) {
            for (int id = 0; id < 4; id++) { writeOutput(id,0); }
            flagzeroed = true;
          }

          // If not controlling nor reading, one can delay for a while:
          delay(1);
          cnextsampletime = ESP.getCycleCount();

        }

    }

}



/* ===== Main Task (Core 0 - Former Writing Task) ==========================================

=> ALWAYS after dealing with de different modes: 
   -- Check if there is serial data to send and sends it if needed.
   -- Deal with commands coming from the serial port (USART).

- In **standy mode**, sleeps for 1 ms if no data has been received via USART (Serial)
  and there is no data to transmit.

- In **reading mode**, this task runs at 250 Hz executing the following steps:
    -- Clears all registered notifications that hapened before.
    -- IMUs from I2C-2 and SPI buses are read.
    -- Measures intermediate elapsed time value (timecounter1a).
    -- Stops, waiting for a notification from AUXTASK.
    -- Set up data package to be transmit via serial port:
       -- Get readings from IMUs read in AUXTASK (lastimureadings queues).
       -- Get values for signal generators.
       -- Get ADC readings obtained in AUXTASK (adcreadings queue).
    -- Measure elapsed time again (from the beginning of this task, including the 
       time stopped waiting for a notification from AUXTASK).

- In **controlling mode**, this task runs at 250 Hz executing the following steps:
    -- Clears all registered notifications that hapened before.
    -- Sensors at IMUs from I2C-2 and SPI buses are read.
    -- Measures intermediate elapsed time value (timecounter1a).
    -- Stops, waiting for a notification from AUXTASK.
    -- Gets readings that happened in AUXTASK (controlqueue)
    -- If control algorithm is on (AlgOn), updates filter coefficients.
    -- Performs feedback filter reading. 
    -- If AlgOn, filters with the adaptive filter (i.e., control effort calculation).
    -- Prepares data package to be sent via serial port.

        
=====================================================================================*/ 
void MainTask(void * parameter){

  uint8_t sr = 0;  // Stores commands read from the computer host.
  char hascmd = 0;  // Used for indicating if command from the computer host needs to be treated before accepting new commands.
  int cthascmd = 0;  // Indicates if the cmd has been treated. TODO: check if it is important or not, maybe could be changed to a flag. 
  uint8_t imuidd;

  transmitData tdata;
  int ctt = 0;
  uint64_t timesample,timetask1,timetask2;
  uint32_t clastsampletime,ctimeaux,ccountaux = 0;

  uint8_t errorflags;  // Errors: None | None | None | None | None | IncompleteADCRead | TaskNotifyTimeout2  | TaskNotifyTimeout1  

  EventBits_t uxBits;

    for (;;) {

      if (reading) {


            if (CTTRatio == 255) {

              // New version:

              errorflags = 0;   
              // Clear bits from EventGroup to wait for notifications/flags from AuxTask
              uxBits = xEventGroupClearBits(xEventGroup,0x07); // Clear bits 1 and 0
              // Wait for notification (flag, bit 0) from AuxTask (expires in 20 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdFALSE, 20);
              if ( (uxBits & 0x01) == 0 ) { errorflags = errorflags | 0x01; };
              clastsampletime = ESP.getCycleCount();
              timesample = (clastsampletime - ctimeaux) / 240;
              ctimeaux = clastsampletime;

              tdata.nbytes = 0;                      
              
              ctt = 0;
              // First 3 bytes for sync:
              tdata.data[ctt++] = 0xF;
              tdata.data[ctt++] = 0xF;
              tdata.data[ctt++] = 0xF;

              // IMU Readings:
              for (int id = 0; id < 3; id++) {
                  if (imuenable[id] == 1) {  
                    if (imubus[id] != 0) {
                      if (imutype[id] == 0) {
                        mpus[id].readData(&tdata.data[ctt]);                    
                      } else {
                        lsms[id].readRegisterRegion(&tdata.data[ctt],0x22,12);
                      }
                    }
                    if (imutype[id] == 0) { ctt = ctt + 14; }
                    else { ctt = ctt + 12; }
                  }
              }
        
              // Wait for notification (flag, bit 1) from AuxTask (expires in 2 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x02, pdTRUE, pdFALSE, 2); 
              if ( (uxBits & 0x02) == 0 ) { errorflags = errorflags | 0x02; }
              timetask2 = (ESP.getCycleCount() - clastsampletime) >> 8;  

              ctt = 3;
              for (int id = 0; id < 3; id++) {
                if (imuenable[id] == 1) {
                  if (imubus[id] != 0) { // Just realign:
                    if (imutype[id] == 0) { ctt = ctt + 14; }
                    else { ctt = ctt + 12; }
                  } else { // Copy readings from the other Task:                 
                    int nbytes = 14;
                    if (imutype[id] == 1) { nbytes = 12; }
                    for (int iii = 0; iii < nbytes; iii++) {
                      tdata.data[ctt++] = lastimureadings[id].pop();
                    }
                  }
                }                                      
              }

              // Signal Generators:
              for (int id = 0; id < 4; id++) {
                  if (id < 2) {
                    tdata.data[ctt++] = (outscaler[id].lastwrittenout >> 8) & 0xFF;
                    tdata.data[ctt++] = outscaler[id].lastwrittenout & 0xFF;
                  } else {
                    tdata.data[ctt++] = outscaler[id].lastwrittenout;
                  }        
              }
              
              // Send ADC Readings:
              if ((adcconfig[0] & 0x0F) > 0) {
                int16_t adcaux = adcreadings.pop();
                tdata.data[ctt++] = adcaux >> 8;
                tdata.data[ctt++] = adcaux & 0xFF;   
                for (int iii = 0; iii < 3; iii++) {
                  tdata.data[ctt++] = 0;
                  tdata.data[ctt++] = 0;  
                }                                         
              }
              
              // timesample = tcount2queue.pop(); // Tempo amostragem
              // timerReadMicros(timer0cfg);
              timetask1 = (ESP.getCycleCount() - clastsampletime) >> 8;  // Tempo na task 1
              // timetask2 = tcount2queue.pop(); // Tempo na task 2

              
              tdata.data[ctt++] = (timesample >> 8) & 0xFF;
              tdata.data[ctt++] = timesample & 0xFF;
              tdata.data[ctt++] = (timetask1 >> 8) & 0xFF;
              tdata.data[ctt++] = timetask1 & 0xFF;
              tdata.data[ctt++] = (timetask2 >> 8) & 0xFF;
              tdata.data[ctt++] = timetask2 & 0xFF;
              tdata.data[ctt++] = errorflags;
              tdata.nbytes = ctt;

              uxBits = xEventGroupSetBits(xEventGroup, 0x04);


            } else {


              errorflags = 0;   
              // Clear bits from EventGroup to wait for notifications/flags from AuxTask
              uxBits = xEventGroupClearBits(xEventGroup,0x03); // Clear bits 1 and 0
              // Wait for notification (flag, bit 0) from AuxTask (expires in 20 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdFALSE, 20);
              if ( (uxBits & 0x01) == 0 ) { errorflags = errorflags | 0x01; } 

              tdata.nbytes = 0;                      
              
              ctt = 0;
              // First 3 bytes for sync:
              tdata.data[ctt++] = 0xF;
              tdata.data[ctt++] = 0xF;
              tdata.data[ctt++] = 0xF;

              // IMU Readings:
              for (int id = 0; id < 3; id++) {
                  if (imuenable[id] == 1) {  
                    if (imubus[id] != 0) {
                      if (imutype[id] == 0) {
                        mpus[id].readData(&tdata.data[ctt]);                    
                      } else {
                        lsms[id].readRegisterRegion(&tdata.data[ctt],0x22,12);
                      }
                    }
                    if (imutype[id] == 0) { ctt = ctt + 14; }
                    else { ctt = ctt + 12; }
                  }
              }
        
              // Wait for notification (flag, bit 1) from AuxTask (expires in 2 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x02, pdTRUE, pdFALSE, 2); 
              if ( (uxBits & 0x02) == 0 ) { errorflags = errorflags | 0x02; } 

              ctt = 3;
              for (int id = 0; id < 3; id++) {
                if (imuenable[id] == 1) {
                  if (imubus[id] != 0) { // Just realign:
                    if (imutype[id] == 0) { ctt = ctt + 14; }
                    else { ctt = ctt + 12; }
                  } else { // Copy readings from the other Task:                 
                    int nbytes = 14;
                    if (imutype[id] == 1) { nbytes = 12; }
                    for (int iii = 0; iii < nbytes; iii++) {
                      tdata.data[ctt++] = lastimureadings[id].pop();
                    }
                  }
                }                                      
              }

              // Signal Generators:
              for (int id = 0; id < 4; id++) {
                  if (id < 2) {
                    tdata.data[ctt++] = (outscaler[id].lastwrittenout >> 8) & 0xFF;
                    tdata.data[ctt++] = outscaler[id].lastwrittenout & 0xFF;
                  } else {
                    tdata.data[ctt++] = outscaler[id].lastwrittenout;
                  }        
              }
              
              // Send ADC Readings:
              if ((adcconfig[0] & 0x0F) > 0) {
                if (adcreadings.count() != CTTRatio) {
                  errorflags = errorflags | 0x02; // Set IncompleteADCRead error
                  adcreadings.clear();
                  for (int iii = 0; iii < 4; iii++) {  // TODO: change limit to 5 to deal with 5 ms.
                    tdata.data[ctt++] = 0;
                    tdata.data[ctt++] = 0;
                  }
                } else {
                  for (int iii = 0; iii < 4; iii++) {  // TODO: change limit to 5 to deal with 5 ms.
                    if (iii < CTTRatio) {
                      int16_t adcaux = adcreadings.pop();
                      tdata.data[ctt++] = adcaux >> 8;
                      tdata.data[ctt++] = adcaux & 0xFF;
                    } else {
                      tdata.data[ctt++] = 0;
                      tdata.data[ctt++] = 0; 
                    }                  
                  }                
                }                            
              }
              
              timesample = tcount2queue.pop(); // Tempo amostragem
              timerReadMicros(timer0cfg);
              timetask1 = timerReadMicros(timer0cfg) - tcount2queue.pop();  // Tempo na task 1
              timetask2 = tcount2queue.pop(); // Tempo na task 2

              
              tdata.data[ctt++] = (timesample >> 8) & 0xFF;
              tdata.data[ctt++] = timesample & 0xFF;
              tdata.data[ctt++] = (timetask1 >> 8) & 0xFF;
              tdata.data[ctt++] = timetask1 & 0xFF;
              tdata.data[ctt++] = (timetask2 >> 8) & 0xFF;
              tdata.data[ctt++] = timetask2 & 0xFF;
              tdata.data[ctt++] = errorflags;
              tdata.nbytes = ctt;

              uxBits = xEventGroupSetBits(xEventGroup, 0x04);


            }

            


      } else if (controlling) {

          // Blocks until reaching the time for next sample: --------------------------------------------
          // vTaskDelayUntil(&xLastWakeTime0, xFrequency0);
          // --------------------------------------------------------------------------------------------
          // xTaskNotifyWait(0xffffffffUL,0xffffffffUL,&ulNotifiedValue,(TickType_t)0);

          if (!debugmode) {

              if (ctrltask == 1) {
                siggen[canalcontrole].next();
              } else {
                siggen[canalperturb].next();
              } 

              errorflags = 0;
              // Clear bits from EventGroup to wait for notifications/flags from AuxTask
              uxBits = xEventGroupClearBits(xEventGroup,0x03); // Clear bits 1 and 0
              // Wait for notification (flag, bit 0) from AuxTask (expires in 20 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdFALSE, 20);
              if ( (uxBits & 0x01) == 0 ) { errorflags = errorflags | 0x01; }
              if (CTTRatio == 255) {
                clastsampletime = ESP.getCycleCount();
                timesample = (clastsampletime - ctimeaux) / 240;
                ctimeaux = clastsampletime;
              }          

              if (imubus[idRefIMU] != 0) {
                if (imutype[idRefIMU] == 0) {
                  xref = dcr[0].filter(mpus[idRefIMU].readSensor(idRefIMUSensor));
                } else {
                  xref = dcr[0].filter(lsms[idRefIMU].readSensor(idRefIMUSensor));
                }
              }
              if (imubus[idErrIMU] != 0) {
                if (imutype[idErrIMU] == 0) {
                  xerr = dcr[1].filter(mpus[idErrIMU].readSensor(idErrIMUSensor));
                } else {
                  xerr = dcr[1].filter(lsms[idErrIMU].readSensor(idErrIMUSensor));
                }            
              }

              // Wait for notification (flag, bit 1) from AuxTask (expires in 2 ms):
              uxBits = xEventGroupWaitBits(xEventGroup, 0x02, pdTRUE, pdFALSE, 2 ); 
              if ( (uxBits & 0x02) == 0 ) { errorflags = errorflags | 0x02; }
              if (CTTRatio == 255) {
                timetask2 = (ESP.getCycleCount() - clastsampletime) >> 8;  
              }

              uxBits = xEventGroupSetBits(xEventGroup, 0x04);
              

              if (imubus[idRefIMU] == 0) {
                if (controlqueue.count() == 0) { 
                  errorflags = errorflags | 0x08;
                  xref = 0;
                } else {
                  xref = controlqueue.pop();
                }            
              } 
              if (imubus[idErrIMU] == 0) {
                if (controlqueue.count() == 0) { 
                  errorflags = errorflags | 0x10;
                  xerr = 0;
                } else {
                  xerr = controlqueue.pop();
                }          
              } 


            

          }
          
          if (nextdebugstep) {

            errorflags = 0;

            if (ctrltask == 1) {
              siggen[canalcontrole].next();
            } else {
              siggen[canalperturb].next();
            } 

            // delayMicroseconds(1000);

            // Writing outputs:
            if (ctrltask == 0) {  // If controlling:       
              writeOutputDebug(canalperturb,siggen[canalperturb].lastf);
              writeOutputDebug(canalcontrole,lastout);
            } else {  // If path modeling:
              zeroOutputDebug(canalperturb);
              writeOutputDebug(canalcontrole,siggen[canalcontrole].lastf);
            }
            
          }
        

          if (!debugmode || nextdebugstep) {

                        
            if (ctrltask == 0) {           
              

              switch (algchoice) {

                case 0:
                  if (algOn) {
                    xreff = xref - filtfbk.filter(lastout);
                    fxnlms.updateStep1();
                    fxnlms.filter(xreff);
                    fxnlms.updateStep2(xerr);
                    lastout = fxnlms.y;
                  }
                  break;
              
                case 1:
                  if (algOn) { 
                    xreff = xref - filtfbk.filter(lastout);
                    fxnlms.filter(xreff);
                    lastout = fxnlms.y;
                    fxnlms.update(xerr);
                  }
                  break;

                case 2:
                  if (algOn) {
                    xreff = xref - filtfbk.filter(lastout);
                    cvafxnlms.updateStep1();
                    cvafxnlms.filter(xreff,filtfbk.y);
                    cvafxnlms.updateStep2(xerr);
                    lastout = cvafxnlms.y;
                  }
                  break;
              
                case 3:
                  if (algOn) {
                    xreff = xref - filtfbk.filter(lastout);
                    cvafxnlms.filter(xreff,filtfbk.y);
                    lastout = cvafxnlms.y;
                    cvafxnlms.update(xerr);                    
                  }
                  break;
              
                default:
                  lastout = 0;
                  break;
                  
              }

              lastsenddataaux = outscaler[canalcontrole].lastwrittenout;
              // senddataaux = ((int)roundf( maxamplevel * lastout ));
              // outputaux = (dclevel - senddataaux);
              // if (outputaux > satlevel) { outputaux = satlevel; errorflags = errorflags | 0x20; }
              // else if (outputaux < 0) { outputaux = 0; errorflags = errorflags | 0x40; }
              if (lastout > 1.0) { lastout = 1.0; errorflags = errorflags | 0x20; }
              else if (lastout < -1.0) { lastout = -1.0; errorflags = errorflags | 0x40; }

            }

            
            
          
            // Sending data to the host computer: --------------------------
            // The first three bytes are used for synchronization. 
            // Syncronization needs to be improved, but it is working fine.            
            ctt = 0;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            tdata.data[ctt++] = 0xF;
            if (ctrltask == 0) {
              tdata.data[ctt++] = (outscaler[canalperturb].lastwrittenout >> 8) & 0xFF;
              tdata.data[ctt++] = outscaler[canalperturb].lastwrittenout & 0xFF;
              // tdata.data[ctt++] = (senddataaux >> 8) & 0xFF;
              // tdata.data[ctt++] = senddataaux & 0xFF;
              tdata.data[ctt++] = (lastsenddataaux >> 8) & 0xFF;
              tdata.data[ctt++] = lastsenddataaux & 0xFF;
            } else {
              tdata.data[ctt++] = 0;
              tdata.data[ctt++] = 0;
              tdata.data[ctt++] = (outscaler[canalcontrole].lastwrittenout >> 8) & 0xFF;
              tdata.data[ctt++] = outscaler[canalcontrole].lastwrittenout & 0xFF;
            }
            *(float *) &tdata.data[ctt] = xref;
            ctt = ctt + 4;
            *(float *) &tdata.data[ctt] = xerr;
            ctt = ctt + 4;
            tdata.data[ctt++] = ctrlflags;

            if (CTTRatio == 255) {

              timetask1 = (ESP.getCycleCount() - clastsampletime) >> 8;

            } else {

              timetask1 = timerReadMicros(timer0cfg);

              timesample = tcount2queue.pop(); // Tempo amostragem

              timetask1 = timetask1 - tcount2queue.pop();  // Tempo na task 1

              timetask2 = tcount2queue.pop(); // Tempo na task 2

            }

            

            tdata.data[ctt++] = (timesample >> 8 & 0xFF);
            tdata.data[ctt++] = timesample & 0xFF;
            tdata.data[ctt++] = (timetask1 >> 8) & 0xFF;
            tdata.data[ctt++] = timetask1 & 0xFF;
            // if (!debugmode) {
            //   timetask2 = tcount2queue.pop();
            //   timetask2 = tcount2queue.pop() - timetask2;
            //   timetask2 = (timetask2 >> 4) & 0xFFFF;
            // } else {
            //   timetask2 = 0;
            // }          
            tdata.data[ctt++] = (timetask2 >> 8) & 0xFF;
            tdata.data[ctt++] = timetask2 & 0xFF;
            tdata.data[ctt++] = errorflags;
            tdata.nbytes = ctt;

            nextdebugstep = false;         

          }

      } else { 

        // If not reading nor controlling nor have data to treat, sleep for a while:
        while ( (Serial.available() == 0) && (tdata.nbytes == 0) ) {
          // vTaskDelay((TickType_t)1);
          delay(5);
        }

      }

      if (tdata.nbytes > 0) {
        Serial.write(tdata.data,tdata.nbytes);
        tdata.nbytes = 0;
      }

      /*
        Now, dealing with commmands from the computer host:
      */
      if ((hascmd == 0) && (Serial.available() > 0) ) {

        sr = Serial.read();

        switch (sr) {

          case 'h':
            if (!reading && !controlling) { Serial.write('k'); }
            break;

          case 'b':
            // Set baud rate: 0 = 115200, 1 = 500000, 2 = 921600, 3 = 1000000
            if (!reading && !controlling) {
              uint8_t newbaud = Serial.read(); 
              Serial.write('k');
              Serial.write(newbaud);
              Serial.end();
              switch (newbaud) {
                case 0:
                  baudrate = 115200;
                  break;
                case 1:
                  baudrate = 500000;
                  break;
                case 2:
                  baudrate = 921600;
                  break;
                case 3:
                  baudrate = 1000000;
                  break;
                default:
                  baudrate = 500000;
              }
              Serial.end();
              Serial.begin(baudrate);              
            }
            break;

          case 's':
            if (!controlling) {
              timerStop(timer0cfg);
              timerRestart(timer0cfg);
              timerStart(timer0cfg);
              // if (CTTRatio == 255) {
              //   for (int idd = 0; idd < 4; idd++) { siggen[idd].setFreqMult(F_SAMPLE,1.0); }
              // } else {
              //   for (int idd = 0; idd < 4; idd++) { siggen[idd].setFreqMult(1000.0,4.0); }
              // }              
              xEventGroupClearBits(xEventGroup,0x03); 
              // TickType_t actualtcount = xTaskGetTickCount();
              // xLastWakeTime0 = actualtcount - xFrequency0;
              // xLastWakeTime1 = actualtcount - xFrequency1;
              xLastWakeTime0 = xTaskGetTickCount();
              xLastWakeTime1 = xTaskGetTickCount();
              nextsampletime = 1000;
              cttcycle = 0;
              errorflags = 0;
              reading = true;
              controlling = false;
              // Serial.write('k');
            }            
            break;

          case 'S': 
            if (!reading) { 
              for (int idd = 0; idd < 4; idd++) { siggen[idd].setFreqMult(1.0); }
              dcr[0].reset();
              dcr[1].reset();
              if ((algchoice == 0) || (algchoice == 1)) { fxnlms.reset(); }
              else if ((algchoice == 2) || (algchoice == 3)) { cvafxnlms.reset(); }
              filtsec.reset();
              filtsec2.reset();
              filtfbk.reset();
              lastout = 0;
              // TODO: check the following
              // dclevel = outscale[canalcontrole].dclevel;              
              // maxamplevel = (float)(dclevel-1);
              // maxamplevel = 1.0;
              // satlevel = dclevel * 2 - 1;
              // outputaux = dclevel;
              // senddataaux = 0;
              xEventGroupClearBits(xEventGroup,0x03);
              xLastWakeTime0 = xTaskGetTickCount();
              xLastWakeTime1 = xTaskGetTickCount();
              controlling = true;
              reading = false;
              cttcycle = 0;
              Serial.write('K');
            }
            break;

          case 't':                        
            reading = false; 
            controlling = false;
            nextsampletime = 0;
            // timerStop(timer0cfg);
            // for (int id = 0; id < 4; id++) { writeOutput(id,0); } // Set outputs to zero after stopping.
            flagzeroed = false;
            Serial.println("Stopped!");
            break;

          case 'T':
            hascmd = 'T';
            break;

          case 'N':
            nextdebugstep = true;          
            break;

          case 'f':            
            hascmd = 'f';
            break;

          case 'm':            
            hascmd = 'm';
            break;

          case 'p':
            if (!reading && !controlling) {
              uint8_t pidx = Serial.read();
              uint8_t porder = Serial.read();
              predistorders[pidx] = porder;
              if (porder == 0) {
                predistenable[pidx] = false;
                Serial.write("ok");
              } else {
                predistenable[pidx] = true;
                if (porder < 10) {
                  uint32_t nbytes = (porder+1) << 2;
                  if (Serial.readBytes(&cbuf[0], nbytes) != nbytes) {
                    Serial.write("er");
                  } else {
                    Serial.write("ok");
                    for (int ix = 0; ix < (porder+1); ix++) {
                      predistcoefs[10*pidx+ix] = *(float *)(&cbuf[0+(ix<<2)]);                      
                    }
                    Serial.write(&cbuf[0],nbytes);
                  }                  
                }
              }              
            }
            break;

          case 'F':
            if (!reading && !controlling) {              
              if (Serial.readBytes(&cbuf[0], 8) != 8) {
                Serial.write("er");
              } else {
                Serial.write("ok");
                fusionweights[0] = *(float *)(&cbuf[0]);
                fusionweights[1] = *(float *)(&cbuf[4]);
                Serial.write(&cbuf[0],8);
              }             
            }
            break;

          case 'w':
            if (!reading && !controlling) {
              File file = SPIFFS.open("/predist.dat",FILE_WRITE);
              if(!file){ 
                Serial.print("er0"); 
              } else {
                file.write(&predistorders[0],4);
                file.write((uint8_t *)&predistcoefs[0],160);
                file.write((uint8_t *)&fusionweights[0],8);
                file.close();
                Serial.print("ok!"); 
              }
            }
            break;
          

          case 'P':
            if (!reading && !controlling) {
              bool flagok = true;
              File file = SPIFFS.open("/wsec.dat",FILE_WRITE);
              if(!file){ 
                Serial.print("er0"); 
                flagok = false;
              } else {
                file.write((uint8_t *)&wsec[0],Nsec*sizeof(float));
                file.close();
              } 
              if (flagok) {
                file = SPIFFS.open("/wfbk.dat",FILE_WRITE);
                if(!file){ 
                  Serial.print("er1"); 
                  flagok = false;
                } else {
                  file.write((uint8_t *)&wfbk[0],Nfbk*sizeof(float));
                  file.close();
                }              
                Serial.print("ok!");
              }              
            }
            break;

          case 'c':
            if (!reading && !controlling) {
              Serial.println("WSec = ");
              for (int nn = 0; nn < Nsec; nn++) {
                Serial.println(wsec[nn],10);
              } 
              Serial.println("WFbk = ");
              for (int nn = 0; nn < Nsec; nn++) {
                Serial.println(wfbk[nn],10);
              }
              Serial.println("End!");
            }
            break;


          case 'i':
            if ((!reading) && (!controlling)){ 
              sr = Serial.read();
              if (imubus[sr] != 0) {
                flaginitIMU = initIMU(sr);
              } else {
                flaginitIMU = sr;
                while (flaginitIMU >= 0) { delay(1); }
              }              
              if (flaginitIMU == -1) { Serial.write("ok!"); } 
              else { Serial.write("er"); Serial.write(flaginitIMU); }               
            }   
            break;

          
          case '!':
            if (!reading && !controlling) { hascmd = '!'; }
            break;

          
          case 'a':
            hascmd = 'a';
            break;


          case 'd':
            if (!reading && !controlling) { hascmd = 'd'; }
            break;


          case 'I':
            if (!reading && !controlling) { hascmd = 'I'; }
            break;


          case 'G':
            hascmd = 'G';
            break; 


          case 'W':
            if (!reading && !controlling) { getPaths(); }
            break;


          case 'x':
            Serial.println(SPIFFS.format());            
            break;


          case 'X':
            Serial.println("Fusion Weights:");
            Serial.println(fusionweights[0]);
            Serial.println(fusionweights[1]);
            for (int auxX = 0; auxX < 3; auxX++) {
              Serial.println(*(lsms[auxX].fusionweights));
              Serial.println(*(lsms[auxX].fusionweights+1));
            }
            Serial.println("Predist Data:");
            Serial.println(predistenable[0]);
            Serial.println(predistenable[1]);
            Serial.println(predistenable[2]);
            Serial.println(predistenable[3]);
            Serial.println(predistorders[0]);
            Serial.println(predistorders[1]);
            Serial.println(predistorders[2]);
            Serial.println(predistorders[3]);
            for (int auxX = 0; auxX <= predistorders[1]; auxX++) {
              Serial.println(predistcoefs[10+auxX]);
            }
            for (float auxX = -1.0; auxX <= 1.0; auxX = auxX + 0.1) {
              Serial.println(auxX * evalPoly(fabsf(auxX),predistorders[1],&predistcoefs[10]),10);
            }            
            break;

          case 'r':
            for (int tt = 0; tt < 3; tt++) {
              lsms[0].begin();
              delay(2);
            }
            uint8_t readCheck;
            lsms[0].readRegister(&readCheck, WHO_AM_I);
            Serial.println(readCheck);
            // Serial.println(lsms[0].commInterface);
            Serial.println(lsms[0].readFloatAccel(0));
            Serial.println(lsms[0].readRawTemp());
            // Serial.println(lsms[1].commInterface);
            // Serial.println(lsms[2].commInterface);
            break;          

        }
        
      }


      if (hascmd != 0) { // Late treatment of several commands.

        cthascmd++;
        delayMicroseconds(20);
        if (cthascmd > 100) {
          cthascmd = 0;
          hascmd = 0;
        } 

        switch(hascmd) {

          case 'T':
           if (Serial.available() >= 8) {
              Serial.readBytes(cbuf,8);
              xerr = *(float *)&cbuf[0];
              xref = *(float *)&cbuf[4];
              delayMicroseconds(200);
              Serial.write(cbuf,8);
              cthascmd = 0;
              hascmd = 0;
            }
            break;

          case 'f':
            if (Serial.available() > 0) {
              CTTRatio =  Serial.read();
              T_SAMPLE_us = ((uint16_t)CTTRatio) * 1000;
              T_SAMPLE = ((float)CTTRatio) * 1e-3;
              F_SAMPLE = 1.0 / T_SAMPLE;
              xFrequency0 = CTTRatio; 
              Serial.write("f");
              Serial.write(CTTRatio);  
              cthascmd = 0;
              hascmd = 0;
            } 
            break;

          case 'm':
            if (Serial.available() > 0) {
              CTTRatio = 255;
              int16_t raux = (Serial.read() << 8);
              raux = raux + Serial.read();
              T_SAMPLE_us = raux;
              T_SAMPLE_cycles = uint32_t(T_SAMPLE_us) * 240;
              T_SAMPLE = ((float)raux) * 1e-6;
              F_SAMPLE = 1.0 / T_SAMPLE;
              xFrequency0 = CTTRatio; 
              Serial.write("m");
              Serial.write((raux >> 8) & 0xFF);
              Serial.write(raux & 0xFF);
              cthascmd = 0;
              hascmd = 0;
            } 
            break;

          case 'I':
            if (Serial.available() >= 4) {
              imuidd = Serial.read();
              Serial.readBytes(cbuf,3);
              if (imuidd < 3) {            
                imuenable[imuidd] = cbuf[0] & 0x01;
                imutype[imuidd] = (cbuf[0]>>1) & 0x01;
                imubus[imuidd] = (cbuf[0]>>2) & 0x03;
                imuaddress[imuidd] = cbuf[1];
                imuextra[imuidd] = cbuf[2];                
              }
              Serial.write("KI");            
              hascmd = 0;
              cthascmd = 0;
            }
            break;


          case 'G':
            if (Serial.available() >= 8) { // If command is G, check if five bytes are available. If they are not, try again at the next cycle.
              unsigned int idgerador = Serial.read();  
              unsigned int tipo = Serial.read();
              float amp = (float)((Serial.read() << 8) + Serial.read());
              amp = amp / 2047.0;
              float freq = (float)Serial.read();          
              freq = freq + ((float)Serial.read())/100.0;
              int dclevel = (Serial.read() << 8) + Serial.read();
              if (idgerador < 4) {
                if (CTTRatio == 255) {
                  siggen[idgerador].setType(tipo,amp,freq,1.0,F_SAMPLE);
                } else {
                  siggen[idgerador].setType(tipo,amp,freq,4.0,F_SAMPLE);
                }
                outscaler[idgerador].adjust(dclevel);
                if (tipo == 2) {
                  siggen[idgerador].setChirpParams(Serial.read(),Serial.read(),Serial.read(),
                                                  Serial.read(),(Serial.read() << 8) + Serial.read());
                } 
                if ((tipo == 4) && (idgerador > 1)) {
                  pwmduty[idgerador-2] = Serial.read();
                  siggen[idgerador].enabled = false;
                  if (flagpwm[idgerador-2]) {
                    ledcSetup(idgerador-2,50.0,8);
                    ledcAttachPin(23+idgerador,idgerador-2);
                  }
                  flagpwm[idgerador-2] = true;                                    
                } else {
                  flagpwm[idgerador-2] = false;
                  pwmduty[idgerador-2] = 0;
                }
              }
              if (controlling) { siggen[idgerador].setFreqMult(1.0); }
              hascmd = 0;
              cthascmd = 0;
            }
            break;


          case 'd':
            if (Serial.available() >= 3) {  // If command is d, check if one byte is avaliable from the host computer. If it is not, try again at the next cycle.
              adcconfig[0] = Serial.read();
              adcconfig[1] = Serial.read();
              adcconfig[2] = Serial.read();  
              adctype = (adcconfig[0] >> 4);
              if ( adctype == 0 ) { adc = &adc11; }
              else { adc = &adc10; }        
              flaginitADC = 1;
              while (flaginitADC > 0) { delay(1); }
              if (flaginitADC == 0) {
                Serial.write("KA");
                Serial.write(adcsel); // Written four times to maintain compatibility
                Serial.write(adcsel);
                Serial.write(adcsel);
                Serial.write(adcsel);
                // Serial.write(&adcseq[0],4);
              } else {
                Serial.write("e!");
              }
              hascmd = 0;
              cthascmd = 0;  
            }
            break;


          case 'a':
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
            break;


          case 'A':  // Extra tasks from command 'a'
            if (Serial.available() >= 8) {  // Eight bytes are available? If not, does not block and try again in the next cycle.
              hascmd = 0;
              cthascmd = 0;
              Serial.readBytes(cbuf,8);
              if ((algchoice == 0) || (algchoice == 1)) { // FxNLMS
                fxnlms.mu = *(float *) &cbuf[0];
                fxnlms.fi = *(float *) &cbuf[4];
              } else if ((algchoice == 2) || (algchoice == 3)) { // TAFxNLMS
                cvafxnlms.mu = *(float *) &cbuf[0];
                cvafxnlms.fi = *(float *) &cbuf[4];
              }
              algOn = true;
            }
            break;


          case '!':
            if (Serial.available() >= 15) {
              Serial.readBytes(cbuf,15);          
              canalcontrole = cbuf[0] & 0x0F;
              canalperturb = (cbuf[0] >> 4) & 0x0F;
              idRefIMU = (cbuf[1] >> 4) & 0x0F;
              idRefIMUSensor = (cbuf[1] & 0x0F);
              idErrIMU = (cbuf[2] >> 4) & 0x0F;
              idErrIMUSensor = (cbuf[2] & 0x0F);
              algchoice = cbuf[3];
              memsize = (((int)cbuf[4]) << 8) + (int)cbuf[5];              
              if ((algchoice == 0) || (algchoice == 1)) { // FxNLMS
                fxnlms.setParameters(
                  memsize,
                  *(float *) &cbuf[6],
                  *(float *) &cbuf[10]
                );
              } else if ((algchoice == 2) || (algchoice == 3)) { // TAFxNLMS
                cvafxnlms.setParameters(
                  memsize,
                  *(float *) &cbuf[6],
                  *(float *) &cbuf[10]
                );
              }
              ctrltask = cbuf[14] & 0x01;
              if ( ((cbuf[14] >> 1) & 0x01) == 1 ) {
                debugmode = true;
              } else {
                debugmode = false;
              }
              delayMicroseconds(50);
              Serial.print("ok!");
              algOn = false;
              hascmd = 0;
              cthascmd = 0;
            }
            break;

        }
    
      }

    }
  

}

// Startup configuration:
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(500000);
  Serial.setTimeout(10);

  timer0cfg = timerBegin(0, 2, true);

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  pinMode(VSPI_SS, OUTPUT);

  adc11.begin();
  adc10.begin();

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS failed.");
  } else {
    Serial.println("SPIFFS ok!");
    Serial.println(SPIFFS.totalBytes());
  }

  WireA.begin();
  WireA.setClock(1000000L);
  WireB.begin(13,14,1000000L); // Test: GPIO14 as SCL and GPIO13 as SDA 

  mcps[0].begin();
  mcps[1].begin();

  flagzeroed = false;

  siggen[0].setType(0,0,10,1.0);
  siggen[1].setType(0,0,10,1.0);
  siggen[2].setType(0,0,10,1.0);
  siggen[3].setType(0,0,10,1.0);
  outscaler[0].adjust(2048);
  outscaler[1].adjust(2048);
  outscaler[2].adjust(128);
  outscaler[3].adjust(128);

  xEventGroup = xEventGroupCreate();

  loadFlashData();
  Serial.end();
  Serial.begin(baudrate);

  for (int ix = 0; ix < 3; ix++) {
    lsms[ix].setFusionWeights(&fusionweights[0]);
    mpus[ix].setFusionWeights(&fusionweights[0]);
  }

  
    // disableCore1WDT(); 
    // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        AuxTask,            /* Task function          */
        "Auxiliary Task",     /* Name of the task       */ 
        4096,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        2,                  /* Priority of the task   */
        &reading1,          /* Task handle to keep track of the task */
        1);                 /* Core 1 */
    delay(500); 

    // disableCore0WDT();
    //  // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        MainTask,            /* Task function          */
        "Main Task",     /* Name of the task       */ 
        4096,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        2,                  /* Priority of the task   */
        &writing1,          /* Task handle to keep track of the task */
        0);                 /* Core 0 */
    delay(500); 
 
}

void loop() {
  // vTaskDelay(10000);
  vTaskDelete(NULL);
  vTaskSuspend(NULL);  
}