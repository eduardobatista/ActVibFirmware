#include <Arduino.h>
#include <Wire.h>
#include "MPU6050A.h"

#define GtoMS2 9.80665  // To convert g to m/s^2 

MPU6050A::MPU6050A(int addr) {
    MPU_ADDR = addr;
    found = false;
    responseOk = false;
    powermode = 0;
    accscale = GtoMS2 * 2.0 / 32768.0;
    gyroscale =  250.0 / 32768.0;
}

void MPU6050A::writeRegMPU(int reg, int val) {
    TWire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
    TWire.write(reg);                      // envia o registro com o qual se deseja trabalhar
    TWire.write(val);                      // escreve o valor no registro
    TWire.endTransmission(true);           // termina a transmissão
}

uint8_t MPU6050A::readRegMPU(uint8_t reg) {
    uint8_t data;
    TWire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
    TWire.write(reg);                      // envia o registro com o qual se deseja trabalhar
    TWire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
    TWire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
    data = TWire.read();                   // lê o byte e guarda em 'data'
    return data;                          //retorna 'data'
}

void MPU6050A::findMPU(int mpu_addr) {
    TWire.beginTransmission(MPU_ADDR);
    int data = TWire.endTransmission(true);     
    if(data == 0){ found = true; } else { found = false; }
}

void MPU6050A::setI2C(TwoWire *tw) {
    //Wire.begin(sda_pin, scl_pin);
    // Wire.begin();
    // Wire.setClock(400000L);
    //Wire.setClock(100000L);
    TWire = *tw;
}

void MPU6050A::checkMPU() {
      findMPU(MPU_ADDR);     
      int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
      if (data == 104) {
        responseOk = true;
        data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B     
        if(data == 64) powermode = 0; // Sleep Mode
        else powermode = 1; // Active Mode
      } else {
        responseOk = false;
      }
}  

/* função para configurar as escalas do giroscópio
       registro da escala do giroscópio: 0x1B[4:3]
       0 é 250°/s
        FS_SEL  Full Scale Range
          0        ± 250 °/s      0b00000000
          1        ± 500 °/s      0b00001000
          2        ± 1000 °/s     0b00010000
          3        ± 2000 °/s     0b00011000
    */
void MPU6050A::setGyroScale(uint8_t conf) {
    writeRegMPU(GYRO_CONFIG,(conf << 3));
    gyroscale = (float)(250 << int(conf)) / 32768.0;
}

/* função para configurar as escalas do acelerômetro
    registro da escala do acelerômetro: 0x1C[4:3]
    0 é 250°/s     
    AFS_SEL   Full Scale Range
        0           ± 2g            0b00000000
        1           ± 4g            0b00001000
        2           ± 8g            0b00010000
        3           ± 16g           0b00011000
*/
void MPU6050A::setAccelScale(uint8_t conf) {
    writeRegMPU(ACCEL_CONFIG,(conf << 3));
    accscale = (float)(2 << int(conf)) * GtoMS2 / 32768.0;
}

void MPU6050A::initMPU() {
      writeRegMPU(PWR_MGMT_1,0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
      setGyroScale(0);
      setAccelScale(0);
}

void MPU6050A::readCONFIG() {
      uint8_t aux = readRegMPU(CONFIG);
      Serial.write( ((aux & 0x04) >> 2) + 0x30 );
      Serial.write( ((aux & 0x02) >> 1) + 0x30 );
      Serial.write( (aux & 0x01) + 0x30 );
      readRegMPU(PWR_MGMT_1);
}

void MPU6050A::setFilter(uint8_t val) {
    uint8_t aux = readRegMPU(CONFIG);
    aux = (aux & 0xF8) + (val & 0x07);
    writeRegMPU(CONFIG,aux);
}

void MPU6050A::enableBypass(bool val) {
      if (val) {
        uint8_t aux = readRegMPU(USER_CTRL);
        aux = aux & 0xDF; // zera o bit 5
        writeRegMPU(USER_CTRL,aux);
        aux = readRegMPU(INT_PIN_CFG);
        aux = aux | 0x02; // seta o bit 1
        writeRegMPU(INT_PIN_CFG,aux);
      } else {
        uint8_t aux = readRegMPU(INT_PIN_CFG);
        aux = aux & 0xFD; // zera o bit 1
        writeRegMPU(INT_PIN_CFG,aux);
      }
}

/*
    Lê os dados de apenas 1 sensor, sendo: 
    0 para AccX, 1 para AccY, 2 para AccZ, 
    3 para GyroX, 4 para GyroY, 5 para GyroZ
*/
float MPU6050A::readSensor(int id) {
      float scl;
      if (id < 3) {
        id = ACCEL_XOUT + id*2;
        scl = accscale;
      } else {
        id = GYRO_XOUT + (id-3)*2;
        scl = gyroscale;
      }
      TWire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
      TWire.write(id);                         // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
      TWire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
      TWire.requestFrom(MPU_ADDR,2);           // configura para receber 14 bytes começando do registro escolhido acima (0x3B)    
      buf[1] = TWire.read();
      buf[0] = TWire.read();
      return (float)(*(int16_t *)&buf[0]) * scl;
}

void MPU6050A::readData() {
      TWire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
      TWire.write(ACCEL_XOUT);                 // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
      TWire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
      TWire.requestFrom(MPU_ADDR,14);          // configura para receber 14 bytes começando do registro escolhido acima (0x3B)    
      buf[0] = 0xF;
      buf[1] = 0xF;
      buf[2] = 0xF;
      buf[3] = TWire.read();
      buf[4] = TWire.read();
      buf[5] = TWire.read();
      buf[6] = TWire.read();
      buf[7] = TWire.read();
      buf[8] = TWire.read();
      for (int i = 0; i < 8; i++) {
        buf[9+i] = TWire.read();
      }
      /*while (TWire.available()) {
        TWire.read();
      }*/
      TWire.begin();
}  