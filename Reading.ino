#include <SPI.h>          // SPI
#include <Wire.h>         // I2C
#include <Ticker.h>       // Timing
#include <Preferences.h>  // Allows recording preferences in non-volatile memory
#include <math.h>         // Math

#include "DSPfuncs.h"
#include "MPU6050A.h"
#include "Algorithms.h"
#include "Queue.h"
#include "SparkFunLSM6DS3.h"

#include <MCP4725.h>
#include <ADS1X15.h> 

#define BUF_SIZE 64
#define LED_BUILTIN 2

// SPI Settings
#define VSPI_MISO 19 
#define VSPI_MOSI 23 
#define VSPI_SCLK 18
#define VSPI_SS 5
static const int spiClk = 1000000; // 1 MHz

Preferences prefs;      // Preferences!

// General Settings -----------------------------------------------------------------------
const int mpu_sda_pin = 21; //D21; // definição do pino I2C SDA
const int mpu_scl_pin = 22; //D22; // definição do pino I2C SCL
// ----------------------------------------------------------------------------------------

const float F_SAMPLE = 250.0;  // Sampling frequency (Hz)
const float T_SAMPLE = 0.004;  // Sampling period in seconds
const int T_SAMPLE_us = 4000;  // As integer in microseconds

/* Write do ESP32 DAC outputs
    id = 0 -> GPIO25 (Eletroimã 3)
    id = 1 -> GPIO26 (Eletroimã 4)
*/
void writeOutput(int id,int val) {
    dacWrite(25+id,val & 0xFF);
}

// Declaring and array with two MPU instances at the I2C bus:
MPU6050A mpus[2] = { MPU6050A(0x68), MPU6050A(0x69)};

// Declaring array of MCPs:
MCP4725 mcps[2] = { MCP4725(0x61,&Wire), MCP4725(0x60,&Wire) };

// Declaring ADC:
int8_t adcconfig[4] = {0,0,0,0};
ADS1115 adc(0x4B);

// Dlecaring LSM6DS3
LSM6DS3 SensorOne(SPI_MODE, 5);

// Defining loop tasks
TaskHandle_t reading1;

/* 
   In the following, we have definitions for FIR filters used as
   secondary-path and feedback filters. 
   The lengths of the vector are a maximun values for the sake of memory allocation.
   The actual length of theses filters will be defined by software.
*/
int Nsec = 0;                                         // filter length
int num = 2;                                         // Number of spaces for the queues
float wsec[3000];                                     // Coef. vector
float xsec[3000];                                     // Input vector
FIRFilter filtsec = FIRFilter(1,&wsec[0],&xsec[0]);   // FIR filter declaration
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

// Struct for sensor's data
struct sensorsData{
  float xAcceLSM=0, yAcceLSM=0, zAcceLSM=0;
  float xGyroLSM=0, yGyroLSM=0, zGyroLSM=0;
  uint8_t MPUallData[17];
  int16_t valadc;
};

//Queue used to exchange data between reading and writing tasks
Queue<sensorsData> queue = Queue<sensorsData>(num); // Queue of max num 'sensorsData', where 'sensorsData' is a struct

sensorsData input;
sensorsData output;

bool reading = false;      // Flag for continous reading mode on or off
bool controlling = false;  // Flag for control mode on or off
bool algOn = false;        // Flag indicating wheter the control algorithm is on or off

int sr = 0;        // Stores commands read from the computer host.
char hascmd = 0;   // Used for indicating if command from the computer host needs to be treated before accepting new commands.
int cthascmd = 0;  // Indicates if the cmd has been treated. TODO: check if it is important or not, maybe could be changed to a flag. 

int i;                         // Counter
int sreading = 0;              // Indicates which sensor (0 or 1) is being read in reading mode. In this mode, all variables of only one sensor are read.
int sensorchoice = 0;          // TODO: can be removed, but is used as a temporary storage for defining sreading.
int algchoice = 0;             // 0 = FxNLMS is used for control, 2 = TAFxNLMS is used.
int idsmpus[2] = {0,0};        // MPU ids to define with MPU is channel 1 and which is 2.
int8_t idsensors[2] = {0,0};   // I2C adresses of the MPUs.
float xerr = 0.0;              // Error signal for the control algorithm (corresponds to the reading of the error accelerometer).
float xref = 0.0;              // Reference signal for the control algorithm (corresponds to the reading of the ref. accelerometer).
float xreff = 0.0;             // xref - (feedback filter output). This signal is the input for the control algorithm.
int canalperturb = 0;          // Definition of the actuator output used for generating the perturbation.
int canalcontrole = 1;         // Definition of the actuator output used for injecting the control signal in the beam.
int outputaux = 0;             // Stores the corresponding integer value of the control signal before sending it to the 8-bit DAC. 
float lastout = 0;             // The last (float) value of the control signal, which feeds the feedback filter.
unsigned char ctrlflags = 0;   // Used for indicating that saturation of the control signal has occurred.
unsigned char cbuf[BUF_SIZE];  // Buffer for storing the byte values before sending them to the host computer.  

uint32_t Tsamplecycles = 1000;  // Sampling period in CPU cycles.
uint32_t nextperiod = 0;        // Stores the cycle counter value that corresponding to the next system event. Events happen after each sampling period. 

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

// Reading Task
void Reading(void * parameter){
  for (;;) {
    // Blocks until reaching the time (clock-cycle count) for the next sampling period: ------------ 
      while ( ((nextperiod-ESP.getCycleCount()) >> 31) == 0 ) {
        int auxxxx = 1;      // random instructions
        auxxxx = auxxxx + 2; // random instructions
      }
      nextperiod = nextperiod + Tsamplecycles;
      // ---------------------------------------------------------------------------------------------

    if (reading){
      // LSM6DS3
      SPI.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      digitalWrite(VSPI_SS, LOW);
      //Get all parameters
      input.xAcceLSM = SensorOne.readFloatGyroX();
      input.yAcceLSM = SensorOne.readFloatGyroY();
      input.zAcceLSM = SensorOne.readFloatGyroZ();
      input.xGyroLSM = SensorOne.readFloatAccelX();
      input.yGyroLSM = SensorOne.readFloatAccelY();
      input.zGyroLSM = SensorOne.readFloatAccelZ();
      digitalWrite(VSPI_SS, HIGH);
      SPI.endTransaction();
      
      // MPU6050
      mpus[sreading].readData();
      for (i=0; i<17; i++){
        input.MPUallData[i] = (mpus[sreading].buf[i]);
      }
      
      if (adcconfig[0] == 1) {
        input.valadc = adc.getValue();
      } 
      else {
        input.valadc = 0;
      }

      // Pushes data from both sensors to the queue
      if (queue.count() == num){   // Queue is full
        queue.pop();               // Deletes the first item of the queue
        queue.push(input);         // Adds the most recent data package to back of the queue
      }
      else { // There is free space to be filled on the queue
        queue.push(input); 
      }
    }
  }
}

// Startup configuration:
void setup() {
    Serial.begin(115200);
    
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); 
    pinMode(VSPI_SS, OUTPUT);
  
    // Inicialização LSM6DS3
    if( SensorOne.begin() != 0 ){
        Serial.println("Problem starting the sensor with CS @ Pin 5.");
    }
    else{
        Serial.println("Sensor with CS @ Pin 5 started.");
    }

    Wire.begin();
    Wire.setClock(1000000L);
 
    mpus[0].setI2C(&Wire);
    mpus[0].initMPU();
    mpus[0].checkMPU();

    mpus[1].setI2C(&Wire);
    mpus[1].initMPU();
    mpus[1].checkMPU();
    //if (mpu.responseOk) { Serial.write("\nMPU Response Ok!\n"); }

    mcps[0].begin();
    mcps[1].begin();

    siggen[0].setType(0,0,10,2048);
    siggen[1].setType(0,0,10,2048);
    siggen[2].setType(0,0,10,128);
    siggen[3].setType(0,0,10,128);

    loadFlashData();

    // Inicialização Core 0 (Leitura)
    xTaskCreatePinnedToCore(
        Reading,            /* Task function          */
        "Reading Task",     /* Name of the task       */ 
        1500,               /* Stack size of the task */
        NULL,               /* Parameter of the task  */
        1,                  /* Priority of the task   */
        &reading1,          /* Task handle to keep track of the task */
        0);                 /* Core 0 */
    delay(500);             // needed to start-up Reading task

    disableCore0WDT();
    Tsamplecycles = T_SAMPLE_us * ESP.getCpuFreqMHz(); // Sampling period in clock cycles 
    nextperiod = ESP.getCycleCount();
}
  
void loop() {
    if (reading){
      if (queue.count() != 0){
        // Gets all the struct data from the queue
        output = queue.pop();
  
        /*
        // LSM6DS3 OUTPUT --------------------------------------------------------------------
        OUTxAcceLSM = output.xAcceLSM;
        OUTyAcceLSM = output.yAcceLSM;
        OUTzAcceLSM = output.zAcceLSM;
        OUTxGyroLSM = output.xGyroLSM;
        OUTyGyroLSM = output.yGyroLSM;
        OUTzGyroLSM = output.zGyroLSM;
        /* Adicionar a forma de saída dos dados do LSM6DS3 para visualização no software */
  
        // MPU6050 OUTPUT ---------------------------------------------------------------------
        for (i = 0; i < 17; i++){
          Serial.write(output.MPUallData[i]);
        }
      }
      
      // DAC and ADC related code
      if (siggen[0].enabled) { mcps[0].setValue(siggen[0].next()); }
      else { mcps[0].setValue(0); } 
      Serial.write((siggen[0].last >> 8) & 0x0F);
      Serial.write(siggen[0].last & 0xFF);
  
      if (siggen[1].enabled) { mcps[1].setValue(siggen[1].next()); }
      else { mcps[1].setValue(0); } 
      Serial.write((siggen[1].last >> 8) & 0x0F);
      Serial.write(siggen[1].last & 0xFF);
      
      if (siggen[2].enabled) { dacWrite(25,siggen[2].next()); }
      else { dacWrite(25,0); }  
      Serial.write(siggen[2].last);
  
      if (siggen[3].enabled) { dacWrite(26,siggen[3].next());}
      else { dacWrite(26,0); }
      Serial.write(siggen[3].last);
  
      if (adcconfig[0] == 1) {
        Serial.write(output.valadc >> 8);
        Serial.write(output.valadc & 0xFF);
      } else {
        Serial.write(0);
        Serial.write(0);
      }
    }

    // Commands through Serial Monitor 
    if ((hascmd == 0) && (Serial.available() > 0) ) {
      sr = Serial.read();
      
      if (sr == 'h') { // Handshake
        if (!reading && !controlling) { Serial.write('k'); }
      } 
      else if ((sr == 's') && !controlling) { // Starts reading mode.
        reading = true;
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
      else if ((sr == 'c') && !reading && !controlling) { // Changes sensor range configurations (treated afterwards).
        hascmd = 'c';      
      }
      else if ((sr == 'g') && !reading && !controlling) { // Low pass filter setup for the MPU6050 (afterwards).
        hascmd = 'g';      
      } 
      else if (sr == 'G') { // Configura geração do sinal
        // Primeiro byte seguinte é tipo do sinal que será gerado (0 = ruído, 1 = seno)
        // Segundo byte: amplitude do seno ou valor máximo do ruído
        // Terceiro e quarto bytes: 00 para ruído e parte inteira e fracionário da freq do seno.
        hascmd = 'G';
      }     
      else if (sr == 'C') { // Seta sensor a ser usado.
        hascmd = 'C';
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
          mpus[sreading].initMPU();
          mpus[sreading].readData();
          mpus[sreading].checkMPU();
          if (mpus[sreading].responseOk) {
            Serial.write("ok!");
          } else {
            Serial.write("err");
          }
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
      else if (sr == 'r') { // Usado para testes diversos
        Serial.println(Nsec);
        Serial.println(Nfbk);
        Serial.println(fxnlms.mu);
        Serial.println(fxnlms.fi*100);
        Serial.println(fxnlms.N);
        Serial.println((*(fxnlms.filtsec)).N);
        Serial.println(filtfbk.N);
        Serial.println(algchoice);
        // for (int k = 0; k < 30; k++) {
        //     Serial.println(wsec[k]);
        // }
        // for (int k = 0; k < 30; k++) {
        //     Serial.println(wfbk[k]);
        // }
        //Wire.begin();
        unsigned int a;
        a = ESP.getCycleCount();      
        mpus[0].readData();
        a = ESP.getCycleCount() - a;      
        Serial.println(a);
        a = ESP.getCycleCount();      
        mpus[0].readSensor(0);
        a = ESP.getCycleCount() - a;  
        Serial.println(a);
        a = ESP.getCycleCount();      
        mpus[0].readSensor(1);
        mpus[1].readSensor(5);
        a = ESP.getCycleCount() - a;  
        Serial.println(a);
        a = ESP.getCycleCount();      
        filtsec.filter(3.33);
        a = ESP.getCycleCount() - a;
        Serial.println(a);
        Serial.println(mpus[0].readSensor(1));
        Serial.println(mpus[1].readSensor(5));
        // Serial.println(ESP.getCpuFreqMHz());
        // Serial.write(mpu.buf,17); 
        // float aux2 = 0.9999;
        // a = ESP.getCycleCount();
        // aux2 = firfilt.filter(33.3);  
        // Serial.println(ESP.getCycleCount()-a);
        // Serial.println(aux2);
        // Serial.println(Wire.getErrorText(Wire.lastError()));
      }
    }

    if (hascmd != 0) { // Late treatment of several commands.
      cthascmd++;
      if (cthascmd > 10) {
        cthascmd = 0;
        hascmd = 0;
      } else if (hascmd == 'G') {
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
          // if (idgerador == 0) {
          //   siggen[0].setType(tipo,amp,freq);
          //   if (tipo == 2) {
          //     siggen[0].setChirpParams(Serial.read(),Serial.read(),Serial.read(),Serial.read(),Serial.read());
          //   }
          // } else {
          //   siggen[1].setType(tipo,amp,freq);
          //   if (tipo == 2) {
          //     siggen[1].setChirpParams(Serial.read(),Serial.read(),Serial.read(),Serial.read(),Serial.read());
          //   }
          // }
          hascmd = 0;
          cthascmd = 0;
        }            
      }
      else if (hascmd == 'd') {
        if (Serial.available() >= 4) {  // If command is A, check if one byte is avaliable from the host computer. If it is not, try again at the next cycle.
          hascmd = 0;
          cthascmd = 0;
          adcconfig[0] = Serial.read();
          adcconfig[1] = Serial.read();
          adcconfig[2] = Serial.read();
          adcconfig[3] = Serial.read();
          Serial.write("ok");
          if (adcconfig[0] == 1) {
            adc.setGain(adcconfig[2]);
            adc.setDataRate(adcconfig[3]);
            adc.setMode(0); 
            adc.readADC(adcconfig[1]);
          }          
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
      else if (hascmd == 'c') {
        if (Serial.available() >= 2) { // Two bytes are available? If not, does not block and try again in the next cycle.
          sr = Serial.read()-0x30;  
          mpus[sreading].setAccelScale(sr);
          sr = Serial.read()-0x30;
          mpus[sreading].setGyroScale(sr);
          Serial.write("ok");
          hascmd = 0;
          cthascmd = 0;
        }          
      } 
      else if (hascmd == 'g') {
        if (Serial.available() >= 1) {
          sr = Serial.read()-0x30; 
          mpus[sreading].setFilter(sr);
          Serial.write("ok");
          hascmd = 0;
          cthascmd = 0;
        }
      }
      else if (hascmd == 'C') {
        if (Serial.available() >= 1) {
          sensorchoice = Serial.read();
          if (sensorchoice >= 0x30) { sensorchoice = sensorchoice - 0x30; }
          if (sensorchoice == 0 || sensorchoice == 1) {
            sreading = sensorchoice;
          } 
          Serial.write("ok");
          hascmd = 0;
          cthascmd = 0;
        }
      }
      else if (hascmd == 'W') {
        if (Serial.available() > 0) {        
          unsigned char * tptr;
          int nbytes,ct,natual;
          unsigned char type = Serial.read(); // Tipo de informação a gravar.
          Serial.readBytes(cbuf,2);           // número de bytes a ser lido.
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
      else if (hascmd == '!') { // Pacote a ser lido: [Canal controle (1 byte), Sensores (2), Algoritmo (1), N (2), mu (4), fi (4)]
        if (Serial.available() >= 14) {
          Serial.readBytes(cbuf,14);
          canalcontrole = cbuf[0];
          if (canalcontrole == 0) { canalperturb = 1; }
          else if (canalcontrole == 1) { canalperturb = 0; }
          else { canalcontrole = 1; canalperturb = 0; }
          idsensors[0] = 0x0F & cbuf[1];
          idsensors[1] = 0x0F & cbuf[2];
          idsmpus[0] = cbuf[1] >> 4;
          idsmpus[1] = cbuf[2] >> 4;
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
          Serial.print("ok!");
          algOn = false;
          hascmd = 0;
          cthascmd = 0;
        }
      }
    }
}
