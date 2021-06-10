#include <Arduino.h>
#include <Wire.h>

#ifndef MPU6050A_h
#define MPU6050A_h

    class MPU6050A {

        private:             
            const int WHO_AM_I = 0x75; // registro de identificação do dispositivo
            const int PWR_MGMT_1 = 0x6B; // registro de configuração do gerenciamento de energia
            const int GYRO_CONFIG = 0x1B; // registro de configuração do giroscópio
            const int ACCEL_CONFIG = 0x1C; // registro de configuração do acelerômetro
            const int ACCEL_XOUT = 0x3B; // registro de leitura do eixo X do acelerômetro
            const int GYRO_XOUT = 0x43; // registro de leitura do eixo X do gyro
            const int CONFIG = 0x1A; // registro que contém configurações de filtro.
            const int INT_PIN_CFG = 0x37; 
            const int USER_CTRL = 0x6A; 
            int MPU_ADDR = 0x68; //0x68; // definição do endereço do sensor MPU6050 (0x68)
            TwoWire* TWire;
            void writeRegMPU(int reg, int val);
            uint8_t readRegMPU(uint8_t reg);
            void findMPU(int mpu_addr);
        
        public:
            uint8_t buf[14];
            bool found;
            bool responseOk;
            int powermode;
            float accscale;
            float gyroscale;
            MPU6050A(int addr, TwoWire *tw);
            void setI2C(TwoWire *tw);
            void checkMPU();
            void setGyroScale(uint8_t conf);
            void setAccelScale(uint8_t conf);
            void initMPU();
            void readCONFIG();
            void setFilter(uint8_t val);
            void enableBypass(bool val);
            float readSensor(int id);
            void readData(uint8_t *outputpointer);
            void setAddress(int addr);
            int getAddress();

    };

#endif