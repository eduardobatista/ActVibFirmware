/*
    Based on SparkfunLSM6DS3
    - Rewritten from the sake of understanding and simplicity
*/

#include "Wire.h"

// Register Definitions:
#define FUNC_CFG_ACCESS 0x01
#define SENSOR_SYNC_TIME_FRAME 0x04
#define FIFO_CTRL1 0x06
#define FIFO_CTRL2 0x07
#define FIFO_CTRL3 0x08
#define FIFO_CTRL4 0x09
#define FIFO_CTRL5 0x0A
#define ORIENT_CFG_G 0x0B
#define INT1_CTRL 0x0D
#define INT2_CTRL 0x0E
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL4_C 0x13
#define CTRL5_C 0x14
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18
#define CTRL10_C 0x19
#define MASTER_CONFIG 0x1A
#define WAKE_UP_SRC 0x1B
#define TAP_SRC 0x1C
#define D6D_SRC 0x1D
#define STATUS_REG 0x1E
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D
#define SENSORHUB1_REG 0x2E
#define SENSORHUB2_REG 0x2F
#define SENSORHUB3_REG 0x30
#define SENSORHUB4_REG 0x31
#define SENSORHUB5_REG 0x32
#define SENSORHUB6_REG 0x33
#define SENSORHUB7_REG 0x34
#define SENSORHUB8_REG 0x35
#define SENSORHUB9_REG 0x36
#define SENSORHUB10_REG 0x37
#define SENSORHUB11_REG 0x38
#define SENSORHUB12_REG 0x39
#define FIFO_STATUS1 0x3A
#define FIFO_STATUS2 0x3B
#define FIFO_STATUS3 0x3C
#define FIFO_STATUS4 0x3D
#define FIFO_DATA_OUT_L 0x3E
#define FIFO_DATA_OUT_H 0x3F
#define TIMESTAMP0_REG 0x40
#define TIMESTAMP1_REG 0x41
#define TIMESTAMP2_REG 0x42
#define STEP_TIMESTAMP_L 0x49 0100 1001
#define STEP_TIMESTAMP_H 0x4A 0100 1010
#define STEP_COUNTER_L 0x4B
#define STEP_COUNTER_H 0x4C
#define SENSORHUB13_REG 0x4D
#define SENSORHUB14_REG 0x4E
#define SENSORHUB15_REG 0x4F
#define SENSORHUB16_REG 0x50
#define SENSORHUB17_REG 0x51
#define SENSORHUB18_REG 0x52
#define FUNC_SRC 0x53
#define TAP_CFG 0x58
#define TAP_THS_6D 0x59
#define INT_DUR2 0x5A
#define WAKE_UP_THS 0x5B
#define WAKE_UP_DUR 0x5C
#define FREE_FALL 0x5D
#define MD1_CFG 0x5E
#define MD2_CFG 0x5F
#define OUT_MAG_RAW_X_L 0x66
#define OUT_MAG_RAW_X_H 0x67
#define OUT_MAG_RAW_Y_L 0x68
#define OUT_MAG_RAW_Y_H 0x69
#define OUT_MAG_RAW_Z_L 0x6A
#define OUT_MAG_RAW_X_H 0x6B

typedef enum {
    IMU_SUCCESS,
    IMU_HW_ERROR,
    IMU_NOT_SUPPORTED,
    IMU_GENERIC_ERROR,
    IMU_OUT_OF_BOUNDS,
    IMU_ALL_ONES_WARNING,
    //...
} status_t;

struct LSM6DS3Settings {
    public:        
        uint8_t gyroEnabled = 1;
        uint8_t gyroRange = 125;
        uint8_t gyroSampleRate = 833;
        uint8_t gyroBandWidth = 400;
        uint8_t gyroFifoEnabled = 0;
        uint8_t gyroFifoDecimation = 1;

        uint8_t accelEnabled = 1;
        uint8_t accelODROff = 1;
        uint8_t accelRange = 2;
        uint8_t accelSampleRate = 0x07; // Corresponds to 833 Hz
        uint8_t accelBandWidth = 400;
        uint8_t accelFifoEnabled = 0;
        uint8_t accelFifoDecimation = 1;

        uint8_t tempEnabled = 1;

        uint8_t commMode = 1;

        uint16_t fifoThreshold = 3000;
        int16_t fifoSampleRate = 10;
        uint8_t fifoModeWord = 0;

        uint8_t tempSensitivity;
};

class LSM6DS3ESP32 {

    public:

        LSM6DS3ESP32(uint8_t); // Paramenter is the address (0x6A or 0x6B)

        void setI2CBus(TwoWire *tw); 
        void changeI2CAddress(uint8_t newaddress);

        void setSPIMode(uint8_t CSPin);

        void config(uint8_t gyroRange, uint8_t accelRange, uint8_t filterBandwidth);

        status_t writeRegister(uint8_t offset, uint8_t dataToWrite);
        status_t readRegister(uint8_t* outputPointer, uint8_t offset);
        status_t readRegisterRegion(uint8_t* outputPointer, uint8_t offset, uint8_t length);
        status_t readRegisterInt16(int16_t* outputPointer, uint8_t offset);

        status_t begin(void);

        float calcGyro(int16_t input);
        float calcAccel(int16_t input);

        float readSensor(int sensorid);
        int16_t readRawAccel(uint8_t axis);
        float readFloatAccel(uint8_t axis);
        int16_t readRawGyro(uint8_t axis);
        float readFloatGyro(uint8_t axis);

        int16_t readRawTemp(void);

        void setFusionWeights(float *fusweights);
        float *fusionweights;

    private:

        LSM6DS3Settings settings;

        uint8_t I2CAddress = 0x6B;
        TwoWire* TWire;

        float gyroscaler = 0.004375;
        float accelscaler = 0.000061;

        // const uint8_t gyromap[5] = {125, 250, 500, 1000, 2000};   
        // Mapping to CTRL1XL, BW_XL:     
	    const uint8_t bwmap[4] = {0x03, 0x02, 0x01, 0x00};
        // Mapping to CTRL1_XL, FS_XL:
        const uint8_t accmap[4] = {0x00, 0x02, 0x03, 0x01};

        //Error checking
        uint16_t allOnesCounter;
        uint16_t nonSuccessCounter;

};