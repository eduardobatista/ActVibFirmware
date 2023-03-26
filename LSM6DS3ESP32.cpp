/*
    Based on SparkfunLSM6DS3
    - Rewritten from the sake of understanding and simplicity
*/

#include "LSM6DS3ESP32.h"

#define GtoMS2 9.80665 

LSM6DS3ESP32::LSM6DS3ESP32(uint8_t address) {
    I2CAddress = address;
}

void LSM6DS3ESP32::setI2CBus(TwoWire *tw) {
    TWire = tw;
}

void LSM6DS3ESP32::changeI2CAddress(uint8_t newaddress) {
    I2CAddress = newaddress;
}

void LSM6DS3ESP32::config(uint8_t gyroRange, uint8_t accelRange, uint8_t filterBandwidth){

    settings.gyroEnabled = 1;  //Can be 0 or 1
    settings.gyroRange = gyroRange;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    settings.gyroSampleRate = 0x07; // Corresponding to 1.66 kHz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    settings.gyroBandWidth = bwmap[filterBandwidth];  //Hz.  Can be: 50, 100, 200, 400;
    settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
    settings.gyroFifoDecimation = 1;  //set 1 for on /1

    gyroscaler = 0.004375 * ((float)(1 >> gyroRange));

    settings.accelEnabled = 1;
    settings.accelODROff = 1;
    settings.accelRange = accmap[accelRange]; //Max G force readable.  Can be: 2, 4, 8, 16
    settings.accelSampleRate = 0x07; // Corresponding to 1.66 kHz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
    settings.accelBandWidth = bwmap[filterBandwidth];  //Hz.  Can be: 50, 100, 200, 400;
    settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
    settings.accelFifoDecimation = 1;  //set 1 for on /1

    accelscaler = 0.000061 * GtoMS2 * ((float)(1 >> accelRange));

    settings.tempEnabled = 1;

    //Select interface mode
    settings.commMode = 1;  //Can be modes 1, 2 or 3

    //FIFO control data
    settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
    settings.fifoSampleRate = 10;  //default 10Hz
    settings.fifoModeWord = 0;  //Default off

}

float LSM6DS3ESP32::calcGyro(int16_t input) {     
    return ((float)input) * gyroscaler;
}

float LSM6DS3ESP32::calcAccel(int16_t input) {     
    return ((float)input) * accelscaler;
}

status_t LSM6DS3ESP32::writeRegister(uint8_t offset, uint8_t dataToWrite) {
    status_t returnError = IMU_SUCCESS;
    //Write the byte
    TWire->beginTransmission(I2CAddress);
    TWire->write(offset);
    TWire->write(dataToWrite);
    if (TWire->endTransmission() != 0) {
        returnError = IMU_HW_ERROR;
    }
    return returnError;
}

status_t LSM6DS3ESP32::readRegisterRegion(uint8_t* outputPointer, uint8_t offset, uint8_t length) {
    status_t returnError = IMU_SUCCESS;

    //define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;
    
    TWire->beginTransmission(I2CAddress);
    TWire->write(offset);
    if (TWire->endTransmission() != 0) {
        returnError = IMU_HW_ERROR;
    } else { //OK, all worked, keep going
        // request 6 bytes from slave device
        TWire->requestFrom(I2CAddress, length);
        // New!!! :
        for (int ctdelay = 0; ctdelay < 25; ctdelay++) {
            if (TWire->available()) {
                while ((TWire->available()) && (i < length)) { // slave may send less than requested
                    c = TWire->read(); // receive a byte as character
                    *outputPointer = c;
                    outputPointer++;
                    i++;
                }
                break;
            } else { delayMicroseconds(1); }
        }
    }

    return returnError;
}

status_t LSM6DS3ESP32::readRegister(uint8_t* outputPointer, uint8_t offset) {
    uint8_t result;
    status_t returnError = IMU_SUCCESS;

    TWire->beginTransmission(I2CAddress);
    TWire->write(offset);
    if (TWire->endTransmission() != 0) {
        returnError = IMU_HW_ERROR;
    }
    TWire->requestFrom(I2CAddress, 1); 
    for (int ctdelay = 0; ctdelay < 25; ctdelay++) {
        if (TWire->available()) {
            while (TWire->available()) { // slave may send less than requested
                result = TWire->read(); // receive a byte as a proper uint8_t
            }
            break;
        } else { delayMicroseconds(1); }
    }

    *outputPointer = result;
    return returnError;
}

status_t LSM6DS3ESP32::readRegisterInt16(int16_t* outputPointer, uint8_t offset) {
    uint8_t myBuffer[2];
    status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
    *outputPointer = (int16_t)(myBuffer[1] << 8 | myBuffer[0]);
    return returnError;
}

status_t LSM6DS3ESP32::begin(void)
{
    status_t returnError = IMU_SUCCESS;    
    uint8_t readCheck = 0;
    readRegister(&readCheck, WHO_AM_I);
    if (!(readCheck == 0x69 || readCheck == 0x6A)) { returnError = IMU_HW_ERROR; }
    if (returnError != IMU_SUCCESS) { return returnError; }

    uint8_t dataToWrite = 0;
    // Accelerometer setup:
    dataToWrite = 0; //Start Fresh!
    if (settings.accelEnabled == 1) {
        dataToWrite |= settings.accelBandWidth;
        dataToWrite |= settings.accelRange << 2;
        dataToWrite |= settings.accelSampleRate << 4;
    }    
    writeRegister(CTRL1_XL, dataToWrite);
    // Set the ODR bit:
    readRegister(&dataToWrite, CTRL4_C);
    dataToWrite |= 0x80;
    writeRegister(CTRL4_C, dataToWrite);

    // Setting BDU to 1:
    readRegister(&dataToWrite, CTRL3_C);
    dataToWrite |= 0x40;
    writeRegister(CTRL3_C, dataToWrite);

    // Gyroscope setup
    dataToWrite = 0; //Start Fresh!
    if (settings.gyroEnabled == 1) {
        // dataToWrite |= settings.gyroBandWidth;        
        dataToWrite |= settings.gyroSampleRate << 4;        
        if (settings.gyroRange == 0) { dataToWrite |= 0x02; }
        else {
            dataToWrite |= (settings.gyroRange-1);
        }
    } 
    writeRegister(CTRL2_G, dataToWrite);

    // uint8_t result;
    // readRegister(&result, WHO_AM_I);
    // //Setup the internal temperature sensor
    // if (settings.tempEnabled == 1) {
    //     if (result == 0x69) { //0x69 LSM6DS3
    //         settings.tempSensitivity = 16;    // Sensitivity to scale 16
    //     } 
    //     // else if (result == LSM6DS3_C_ACC_GYRO_WHO_AM_I) { //0x6A LSM6dS3-C
    //     //    settings.tempSensitivity = 256;    // Sensitivity to scale 256
    //     // }
    // }
    settings.tempSensitivity = 16;

    return returnError;
}

float LSM6DS3ESP32::readSensor(int sensorid) {
	float ret = 0;
    if (sensorid <= 2) {
        ret = readFloatAccel(sensorid);
    } else if (sensorid <= 6) {
        ret = readFloatGyro(sensorid-3);
    } else {
        ret = *(fusionweights) * readFloatAccel(2) + *(fusionweights+1) * readFloatGyro(0);
    }
	return ret;
}

int16_t LSM6DS3ESP32::readRawAccel(uint8_t axis) {
    int16_t output;
    status_t errorLevel = readRegisterInt16(&output, OUTX_L_XL+(axis<<1));
    if (errorLevel != IMU_SUCCESS) {
        if (errorLevel == IMU_ALL_ONES_WARNING) {
            allOnesCounter++;
        } else {
            nonSuccessCounter++;
        }
    }
    return output;
}

float LSM6DS3ESP32::readFloatAccel(uint8_t axis) {
    float output = calcAccel(readRawAccel(axis));
    return output;
}

int16_t LSM6DS3ESP32::readRawGyro(uint8_t axis) {
    int16_t output;
    status_t errorLevel = readRegisterInt16(&output, OUTX_L_G+(axis<<1));
    if (errorLevel != IMU_SUCCESS) {
        if (errorLevel == IMU_ALL_ONES_WARNING) {
            allOnesCounter++;
        } else {
            nonSuccessCounter++;
        }
    }
    return output;
}

float LSM6DS3ESP32::readFloatGyro(uint8_t axis) {
    float output = calcGyro(readRawGyro(axis));
    return output;
}

void LSM6DS3ESP32::setFusionWeights(float *fusweights) {
    fusionweights = fusweights;
}

void LSM6DS3ESP32::setSPIMode(uint8_t CSPin) {
	// FUTURE!
}

int16_t LSM6DS3ESP32::readRawTemp(void) {
    int16_t output;
    readRegisterInt16(&output, OUT_TEMP_L);
    return output;
}