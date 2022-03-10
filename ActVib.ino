#include <Wire.h>         // I2C
#include <SPI.h>
// #include <Ticker.h>       // Timing
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
// Queue<transmitData> queue = Queue<transmitData>(num);

Queue<float> controlqueue = Queue<float>(2);


// Timing variables: -----------------------------------------------------------------------------
float F_SAMPLE = 250.0;  // Sampling frequency (Hz)
float T_SAMPLE = 0.004;  // Sampling period in seconds
TickType_t xFrequency0 = 4;   // Main process will run at 4 ticks = 4 ms period
TickType_t xLastWakeTime0; 
TickType_t xFrequency1 = 1;  // Second process may be sped up by changing this value to 1 or 2; Default is 4.
TickType_t xLastWakeTime1;
uint8_t CTTRatio = 4;
// ------------------------------------------------------------------------------------------------

EventGroupHandle_t xEventGroup;

// Declaring and array with two MPU instances at the I2C bus:
MPU6050A mpus[3] = { MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA), MPU6050A(0x68,&WireA) };
LSM6DS3 lsms[3] = { LSM6DS3(I2C_MODE,0x6B), LSM6DS3(I2C_MODE,0x6B), LSM6DS3(I2C_MODE,0x6B) };
int8_t imuenable[3] = {0,0,0};
int8_t imutype[3] = {0,0,0};
int8_t imubus[3] = {0,0,0};
int8_t imuaddress[3] = {0,0,0};
uint8_t imuextra[3] = {0,0,0};
Queue<uint8_t> lastimureadings[3] = {Queue<uint8_t>(28),Queue<uint8_t>(28),Queue<uint8_t>(28)};

// Declaring array of MCPs:
MCP4725 mcps[2] = { MCP4725(0x61,&WireA), MCP4725(0x60,&WireA) };

// Declaring ADC:
uint8_t adcconfig[3] = {0,0,0};
bool adcenablemap[4] = {false,false,false,false}; 
uint8_t nextadc,lastadc = 0;
uint8_t adcseq[4] = {0,0,0,0};
uint8_t adctype = 0; // 0 for ADS1115 or 1 for ADS1015
ADS1115 adc11(0x4B,&WireA);
ADS1015 adc10(0x4B,&WireA);
ADS1X15* adc;
Queue<uint16_t> adcreadings = Queue<uint16_t>(8);
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

// Four signal generators defined below, since we have four output channels.
SignalGenerator siggen[4] = {SignalGenerator(F_SAMPLE,T_SAMPLE,2048),SignalGenerator(F_SAMPLE,T_SAMPLE,2048),
                             SignalGenerator(F_SAMPLE,T_SAMPLE,128),SignalGenerator(F_SAMPLE,T_SAMPLE,128)};

// DC removal is needed when working with adaptive control algorithms.
// Definitions of simple IIR-based DC removers for the two inputs from accelerometers:
DCRemover dcr[2] = {DCRemover(0.95f),DCRemover(0.95f)};

bool reading = false; // Flag for continous reading mode on or off
bool controlling = false; // Flag for control mode on or off
bool algOn = false; // Flag indicating wheter the control algorithm is on or off

// Control parameters:
uint8_t ctrltask = 0;  // 0 for Control, 1 for Path Modeling
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
float maxamplevel = 0.0;
int satlevel = 0;
int outputaux = 0;  // Stores the corresponding integer value of the control signal before sending it to the DAC output. 
int senddataaux = 0;
float lastout = 0;  // The last (float) value of the control signal, which feeds the feedback filter.
unsigned char ctrlflags = 0;  // Used for indicating that saturation of the control signal has occurred.

unsigned char cbuf[BUF_SIZE];  // Buffer for storing the byte values before sending them to the host computer.  
uint8_t cbuf2[20]; // Stores temporary data from commmunication buses.


/* Write Outputs
    id = 0 -> mcps[0] (12 bits)
    id = 1 -> mcps[1] (12 bits)
    id = 2 -> GPIO25 (Eletroimã 3 - 8 bits)
    id = 3 -> GPIO26 (Eletroimã 4  - 8 bits)
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
    lsms[IMUid].config((imuextra[IMUid]>>2) & 0x07, imuextra[IMUid] & 0x03, (imuextra[IMUid]>>5) & 0x07);
    lsms[IMUid].readRawAccelX();
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
      lsms[IMUid].readRawAccelX();
      delay(1);
      lsms[IMUid].readRawTemp(); 
      delay(1);
      return -1; 
    } else { return -2; }            
  }

}


/*
    Initialization of the ADC, involving definitions and connection check.
     - Returns 0 in case of success and -1 in case of error (sensor not found).
*/
int8_t initADC(void) {
    adc->setGain((1 << adcconfig[1]) >> 1);
    adc->setDataRate(adcconfig[2]); 
    for (int iii = 0; iii < 4; iii++) {
      adcenablemap[iii] = (((adcconfig[0] >> iii) & 0x01) == 1);
    }            
    if ((adcconfig[0] & 0x0F) > 0) {            
      adc->setMode(0);
      nextadc = 0; 
      for (int iii = 0; iii < 4; iii++) {
        while ( adcenablemap[nextadc] == 0  ) {  nextadc = (nextadc+1) & 0x03; }
        adcseq[iii] = nextadc;
        nextadc = (nextadc+1) & 0x03;
      }  
      nextadc = 0;
      adc->readADC(adcseq[0]);
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

/*
  Definition of variables required for task management, communication, etc.
*/ 
int8_t flaginitIMU = -1; 
uint8_t cttcycle = 0; 
Queue<uint32_t> tcount2queue = Queue<uint32_t>(2);
bool flagzeroed = false;


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

  uint32_t auxxxx = 0;  
  transmitData tdata;
  int ctt = 0;
  uint32_t timecounter2;
  uint8_t imuaux[14];
  EventBits_t uxBits;

    for (;;) {        

        if (reading) {  // If Reading Mode is on:

          vTaskDelayUntil(&xLastWakeTime1,xFrequency1);

            if ( cttcycle == 0 ) {    
              tcount2queue.clear();
              tcount2queue.push(ESP.getCycleCount()); 
              uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask
              // Reading IMUS in the main I2C Bus:
              for (int id = 0; id < 3; id++) {
                  if (imuenable[id] == 1) {  
                    if (imubus[id] == 0) {
                      if (imutype[id] == 0) {
                        // mpus[id].readData(&lastimureadings[id][0]);
                        mpus[id].readData(&imuaux[0]);
                      } else {
                        // lsms[id].readRegisterRegion(&lastimureadings[id][0],0x22,12);
                        lsms[id].readRegisterRegion(&imuaux[0],0x22,12);                      
                      }
                      lastimureadings[id].clear();
                      for (int idd = 0; idd < 14; idd++) {
                        lastimureadings[id].push(imuaux[idd]);
                      }                    
                    }                        
                  }
              }
            }
            // cttcycle++;
            // if (cttcycle == 4) { cttcycle = 0; }

            // Generator outputs:
            for (int id = 0; id < 4; id++) {
                if (siggen[id].enabled) { 
                  siggen[id].next();
                  writeOutput(id,siggen[id].lastforout); 
                }
                else { writeOutput(id,0); }
            }

            // ADC readings:
            if ((adcconfig[0] & 0x0F) > 0) {
              
              if (adcreadings.count() == 0) { nextadc = 0; }
              
              if (adcreadings.count() < 4) {  // TODO: change the 4 here to accomodate other sampling rates.
                adcreadings.push(adc->getValue());
                nextadc = (nextadc+1) & 0x03;
                adc->requestADC(adcseq[nextadc]); 
              }

            }

            if (cttcycle == 0) {
              tcount2queue.push(ESP.getCycleCount()); 
              // xTaskNotify(writing1, 0, eNoAction);
              uxBits = xEventGroupSetBits(xEventGroup, 0x02);
            }
            cttcycle++;
            if (cttcycle == CTTRatio) { cttcycle = 0; }

            
        } else if (controlling) {  // If Control Mode is on:

            // Blocks until reaching the time for next sample: --------------------------------------------
            vTaskDelayUntil(&xLastWakeTime1, xFrequency0);
            // --------------------------------------------------------------------------------------------
            uxBits = xEventGroupSetBits(xEventGroup, 0x01); // Set Bit 0 to start MainTask

            tcount2queue.clear();
            tcount2queue.push(ESP.getCycleCount()); 
            
            controlqueue.clear();

            // Writing outputs:
            if (ctrltask == 0) {  // If controlling:       
              // siggen[canalperturb].next();  // Evaluated elsewhere to avoid delays 
              writeOutput(canalperturb,siggen[canalperturb].lastforout);
              writeOutput(canalcontrole,outputaux);
            } else {  // If path modeling:
              // siggen[canalcontrole].next(); // Evaluated elsewhere to avoid delays             
              writeOutput(canalperturb,siggen[canalperturb].Z_LEVEL);
              writeOutput(canalcontrole,siggen[canalcontrole].lastforout);
            }
            

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
            // xTaskNotify(writing1, 0, eNoAction);
            uxBits = xEventGroupSetBits(xEventGroup, 0x02); // Set Bit 0 to unlock MainTask

            // timecounter2 = ESP.getCycleCount() - timecounter2;
            tcount2queue.push(ESP.getCycleCount()); 

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
  uint8_t ctcycle = 0;
  uint8_t imuidd;

  uint32_t auxxxx = 0;
  transmitData tdata;
  int ctt = 0;
  uint32_t timecounter1,timecounter1a,timecounter2;

  uint8_t errorflags;  // Errors: None | None | None | None | None | IncompleteADCRead | TaskNotifyTimeout2  | TaskNotifyTimeout1  

  uint32_t ulNotifiedValue;
  BaseType_t retnotify = pdFALSE;

  EventBits_t uxBits;

    for (;;) {

      if (reading) {

            // Blocks until reaching the next sampling period: --------------------------------------------    
            // vTaskDelayUntil(&xLastWakeTime0, xFrequency0);
            // -------------------------------------------------------------------------------------------- 
            // xTaskNotifyWait(0xffffffffUL,0xffffffffUL,&ulNotifiedValue,(TickType_t)0);

            errorflags = 0;   
            // Clear bits from EventGroup to wait for notifications/flags from AuxTask
            uxBits = xEventGroupClearBits(xEventGroup,0x03); // Clear bits 1 and 0
            // Wait for notification (flag, bit 0) from AuxTask (expires in 20 ms):
            uxBits = xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdFALSE, 20);
            if ( (uxBits & 0x01) == 0 ) { errorflags = errorflags | 0x01; } 

            timecounter1 = ESP.getCycleCount();

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

            timecounter1a = ESP.getCycleCount() - timecounter1;
      
            // retnotify = xTaskNotifyWait(0,0xffffffffUL,&ulNotifiedValue,(TickType_t)2);
            // if (retnotify == pdFALSE) { errorflags = errorflags | 0x01; } // Set TaskNotifyTimeout           
            // Wait for notification (flag, bit 1) from AuxTask (expires in 2 ms):
            uxBits = xEventGroupWaitBits(xEventGroup, 0x02, pdTRUE, pdFALSE, 2 ); 
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
                    // tdata.data[ctt++] = lastimureadings[id][iii];
                    tdata.data[ctt++] = lastimureadings[id].pop();
                  }
                }
              }                                      
            }

            // Signal Generators:
            for (int id = 0; id < 4; id++) {
                if (id < 2) {
                  tdata.data[ctt++] = (siggen[id].last >> 8) & 0xFF;
                  tdata.data[ctt++] = siggen[id].last & 0xFF;
                } else {
                  tdata.data[ctt++] = siggen[id].last;
                }        
            }
            
            // Send ADC Readings:
            if ((adcconfig[0] & 0x0F) > 0) {
              // if ((adcreadings.count() < CTTRatio) || (adcreadings.count() > CTTRatio)) {
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

            timecounter1 = ESP.getCycleCount() - timecounter1;

            timecounter2 = tcount2queue.pop();
            timecounter2 = tcount2queue.pop() - timecounter2;

            timecounter1 = (timecounter1 >> 4) & 0xFFFF;
            tdata.data[ctt++] = (timecounter1 >> 8) & 0xFF;
            tdata.data[ctt++] = timecounter1 & 0xFF;
            timecounter1a = (timecounter1a >> 4) & 0xFFFF;
            tdata.data[ctt++] = (timecounter1a >> 8) & 0xFF;
            tdata.data[ctt++] = timecounter1a & 0xFF;
            timecounter2 = (timecounter2 >> 4) & 0xFFFF;
            tdata.data[ctt++] = (timecounter2 >> 8) & 0xFF;
            tdata.data[ctt++] = timecounter2 & 0xFF;
            tdata.data[ctt++] = errorflags;
            tdata.nbytes = ctt;


      } else if (controlling) {

          // Blocks until reaching the time for next sample: --------------------------------------------
          // vTaskDelayUntil(&xLastWakeTime0, xFrequency0);
          // --------------------------------------------------------------------------------------------
          // xTaskNotifyWait(0xffffffffUL,0xffffffffUL,&ulNotifiedValue,(TickType_t)0);

          errorflags = 0;
          // Clear bits from EventGroup to wait for notifications/flags from AuxTask
          uxBits = xEventGroupClearBits(xEventGroup,0x03); // Clear bits 1 and 0
          // Wait for notification (flag, bit 0) from AuxTask (expires in 20 ms):
          uxBits = xEventGroupWaitBits(xEventGroup, 0x01, pdTRUE, pdFALSE, 20);
          if ( (uxBits & 0x01) == 0 ) { errorflags = errorflags | 0x01; }          

          timecounter1 = ESP.getCycleCount();

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

          timecounter1a = ESP.getCycleCount() - timecounter1;

          // retnotify = xTaskNotifyWait(0,0xffffffffUL,&ulNotifiedValue,(TickType_t)1);
          // if (retnotify == pdFALSE) { errorflags = errorflags | 0x01; } // Set TaskNotifyTimeout

          // Wait for notification (flag, bit 1) from AuxTask (expires in 2 ms):
          uxBits = xEventGroupWaitBits(xEventGroup, 0x02, pdTRUE, pdFALSE, 2 ); 
          if ( (uxBits & 0x02) == 0 ) { errorflags = errorflags | 0x02; } 

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


          if (ctrltask == 1) {

            siggen[canalcontrole].next();

          } else {

            siggen[canalperturb].next();

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
            // if (algOn) { 
                if (algchoice == 0) {
                  fxnlms.filter(xreff);
                  lastout = fxnlms.y;
                } else if (algchoice == 2) {
                  tafxnlms.filter(xreff,filtfbk.y);
                  lastout = tafxnlms.y;
                } 
                senddataaux = ((int)round( maxamplevel * lastout ));
                outputaux = dclevel - senddataaux;
                if (outputaux > satlevel) { outputaux = satlevel; errorflags = errorflags | 0x20; }
                else if (outputaux < 0) { outputaux = 0; errorflags = errorflags | 0x40; }
                // else { ctrlflags = 0; } 
            // } else {
            //     outputaux = dclevel;
            //     // ctrlflags = 0;
            // }

          }

          // writeOutput(canalperturb,siggen[canalperturb].next());
          // writeOutput(canalcontrole,outputaux);
          
          // Sending data to the host computer: --------------------------
          // The first three bytes are used for synchronization. 
          // Syncronization needs to be improved, but it is working fine.            
          ctt = 0;
          tdata.data[ctt++] = 0xF;
          tdata.data[ctt++] = 0xF;
          tdata.data[ctt++] = 0xF;
          if (ctrltask == 0) {
            tdata.data[ctt++] = (siggen[canalperturb].last >> 8) & 0xFF;
            tdata.data[ctt++] = siggen[canalperturb].last & 0xFF;
            tdata.data[ctt++] = (senddataaux >> 8) & 0xFF;
            tdata.data[ctt++] = senddataaux & 0xFF;
          } else {
            tdata.data[ctt++] = 0;
            tdata.data[ctt++] = 0;
            tdata.data[ctt++] = (siggen[canalcontrole].last >> 8) & 0xFF;
            tdata.data[ctt++] = siggen[canalcontrole].last & 0xFF;
          }
          *(float *) &tdata.data[ctt] = xref;
          ctt = ctt + 4;
          *(float *) &tdata.data[ctt] = xerr;
          ctt = ctt + 4;
          tdata.data[ctt++] = ctrlflags;
          timecounter1 = (ESP.getCycleCount()-timecounter1) >> 4;
          tdata.data[ctt++] = (timecounter1 >> 8 & 0xFF);
          tdata.data[ctt++] = timecounter1 & 0xFF;
          timecounter1a = (timecounter1a >> 4) & 0xFFFF;
          tdata.data[ctt++] = (timecounter1a >> 8) & 0xFF;
          tdata.data[ctt++] = timecounter1a & 0xFF;
          timecounter2 = tcount2queue.pop();
          timecounter2 = tcount2queue.pop() - timecounter2;
          timecounter2 = (timecounter2 >> 4) & 0xFFFF;
          tdata.data[ctt++] = (timecounter2 >> 8) & 0xFF;
          tdata.data[ctt++] = timecounter2 & 0xFF;
          tdata.data[ctt++] = errorflags;
          tdata.nbytes = ctt;

      } else { 

        // If not reading nor controlling nor have data to treat, sleep for a while:
        while ( (Serial.available() == 0) && (tdata.nbytes == 0) ) {
          // delayMicroseconds(50);
          vTaskDelay((TickType_t)1);
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

          case 's':
            if (!controlling) {
              for (int idd = 0; idd < 4; idd++) { siggen[idd].setFreqMult(4.0); }
              xEventGroupClearBits(xEventGroup,0x03); 
              TickType_t actualtcount = xTaskGetTickCount();
              xLastWakeTime0 = actualtcount - xFrequency0;
              xLastWakeTime1 = actualtcount - xFrequency1;
              ctcycle = 0;
              cttcycle = 0;
              reading = true;
              controlling = false;
            }            
            break;

          case 'S': 
            if (!reading) {
              for (int idd = 0; idd < 4; idd++) { siggen[idd].setFreqMult(1.0); }
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
              maxamplevel = (float)(dclevel-1);
              satlevel = dclevel * 2 - 1;
              outputaux = dclevel;
              senddataaux = 0;
              xEventGroupClearBits(xEventGroup,0x03);
              xLastWakeTime0 = xTaskGetTickCount();
              xLastWakeTime1 = xTaskGetTickCount();
              controlling = true;
              reading = false;
              cttcycle = 0;
            }
            break;


          case 't':
            reading = false; 
            controlling = false;
            // for (int id = 0; id < 4; id++) { writeOutput(id,0); } // Set outputs to zero after stopping.
            flagzeroed = false;
            break;


          case 'f':            
            hascmd = 'f';
            break;


          case 'P':
            if (!reading && !controlling) {
              prefs.begin("AlgData");
              prefs.putInt("AlgData",Nsec);
              prefs.putInt("AlgData",Nfbk);
              prefs.putBytes("AlgData",&wsec[0],Nsec*sizeof(float));
              prefs.putBytes("AlgData",&wfbk[0],Nfbk*sizeof(float));
              prefs.end();
              Serial.print("ok!");
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
              else { Serial.write("err"); }               
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


          case 'X':
            // unsigned char aux[4];
            // aux[0] = Serial.read();
            // aux[1] = Serial.read();
            // aux[2] = Serial.read();
            // aux[3] = Serial.read();
            // float xn = *(float *) aux;
            // aux[0] = Serial.read();
            // aux[1] = Serial.read();
            // aux[2] = Serial.read();
            // aux[3] = Serial.read();
            // float dn = *(float *) aux;
            // float en = 0;
            // //en = dn - fxnlms.filter(xn);
            // //fxnlms.update(en);
            // if (!isnan(xn) && !isnan(dn)) {
            //   Serial.write('k');
            //   en = dn - filtsec.filter(fxnlms.filter(xn));
            //   fxnlms.update(en);   
            //   *(float *) aux = en; 
            //   Serial.write(aux,4);     
            // } else {
            //   //Serial.write('!');
            //   //en = dn - fxnlms.filter(0);
            //   //fxnlms.update(0);
            // }
            break;

          case 'r':
            Serial.println("--------");
            Serial.println(imubus[0]);
            Serial.println(imutype[0]);
            Serial.println(imuaddress[0]);
            Serial.println(imuenable[0]);
            Serial.println(imuextra[0]);
            Serial.println(initIMU(0));
            // lsms[0].setI2CBus(&WireA);
            // lsms[0].changeI2CAddress(0x6B);
            // for (int tt = 0; tt < 3; tt++) {
            //   lsms[0].begin();
            //   delay(2);
            // }
            // uint8_t readCheck;
            // lsms[0].readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
            // Serial.println(readCheck);
            // Serial.println(lsms[0].commInterface);
            // Serial.println(lsms[0].readFloatAccelX());
            // Serial.println(lsms[0].readRawTemp());
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

          case 'f':
            if (Serial.available() > 0) {
              CTTRatio =  Serial.read();
              T_SAMPLE = ((float)CTTRatio) * 1e-3;
              F_SAMPLE = 1.0 / T_SAMPLE;
              xFrequency0 = CTTRatio; 
              Serial.write("k");
              Serial.write(CTTRatio);  
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
              Serial.write("ok");            
              hascmd = 0;
              cthascmd = 0;
            }
            break;


          case 'G':
            if (Serial.available() >= 8) { // If command is G, check if five bytes are available. If they are not, try again at the next cycle.
              unsigned int idgerador = Serial.read();  
              unsigned int tipo = Serial.read();
              float amp = (float)((Serial.read() << 8) + Serial.read());
              float freq = (float)Serial.read();          
              freq = freq + ((float)Serial.read())/100.0;
              int dclevel = (Serial.read() << 8) + Serial.read();
              if (idgerador < 4) {
                siggen[idgerador].setType(tipo,amp,freq,dclevel,4.0);
                if (tipo == 2) {
                  siggen[idgerador].setChirpParams(Serial.read(),Serial.read(),Serial.read(),
                                                  Serial.read(),(Serial.read() << 8) + Serial.read());
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
              // adc->setGain((1 << adcconfig[1]) >> 1);
              // adc->setDataRate(adcconfig[2]); 
              // for (int iii = 0; iii < 4; iii++) {
              //   adcenablemap[iii] = (((adcconfig[0] >> iii) & 0x01) == 1);
              // }            
              // if ((adcconfig[0] & 0x0F) > 0) {            
              //   adc->setMode(0);
              //   nextadc = 0; 
              //   for (int iii = 0; iii < 4; iii++) {
              //     while ( adcenablemap[nextadc] == 0  ) {  nextadc = (nextadc+1) & 0x03; }
              //     adcseq[iii] = nextadc;
              //     nextadc = (nextadc+1) & 0x03;
              //   }  
              //   nextadc = 0;
              //   adc->readADC(adcseq[0]);
              // } else {
              //   adc->setMode(1);
              // }             
              // if (adc->isConnected()) {
              //   Serial.write("ok");
              //   Serial.write(&adcseq[0],4);
              // } else {
              //   Serial.write("e!");
              // }        
              flaginitADC = 1;
              while (flaginitADC > 0) { delay(1); }
              if (flaginitADC == 0) {
                Serial.write("ok");
                Serial.write(&adcseq[0],4);
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
              if (algchoice == 0) { // FxNLMS
                fxnlms.mu = *(float *) &cbuf[0];
                fxnlms.fi = *(float *) &cbuf[4];
              } else if (algchoice == 2) { // TAFxNLMS
                tafxnlms.mu = *(float *) &cbuf[0];
                tafxnlms.fi = *(float *) &cbuf[4];
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
              ctrltask = cbuf[14];
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
  // Serial.begin(115200);
  // Serial.begin(230400);
  Serial.begin(500000);
  // Serial.begin(500000);

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  pinMode(VSPI_SS, OUTPUT);

  adc11.begin();
  adc10.begin();

  WireA.begin();
  WireA.setClock(1000000L);
  WireB.begin(13,14,1000000L); // Test: GPIO14 as SCL and GPIO13 as SDA 

  mcps[0].begin();
  mcps[1].begin();

  flagzeroed = false;

  siggen[0].setType(0,0,10,2048,1.0);
  siggen[1].setType(0,0,10,2048,1.0);
  siggen[2].setType(0,0,10,128,1.0);
  siggen[3].setType(0,0,10,128,1.0);

  xEventGroup = xEventGroupCreate();

  loadFlashData();

  
    // disableCore1WDT(); 
    // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        AuxTask,            /* Task function          */
        "Auxiliary Task",     /* Name of the task       */ 
        2048,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        2,                  /* Priority of the task   */
        &reading1,          /* Task handle to keep track of the task */
        1);                 /* Core 1 */
    delay(500); 

    // disableCore1WDT();    // needed to start-up Reading task  

    disableCore0WDT();
    //  // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        MainTask,            /* Task function          */
        "Main Task",     /* Name of the task       */ 
        2048,               /* Stack size of the task */
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