/*   
    QMC5883L Digital Compass Library
*/

// Some part of the code is adapted from Adafruit HMC5883 library

#include "QMC5883L.h"

QMC5883L::QMC5883L(PinName sda, PinName scl): QMC5883L_i2c(sda, scl)
{
}

float QMC5883L::setMagRange(MagScale Mscale)
{
    float mRes; // Varies with gain
    
    switch(Mscale)
    {
        case MagScale_2G:
            mRes = 1.0/12000;  //LSB/G
            break;
        case MagScale_8G:
            mRes = 1.0/3000;
            break;
    } 
    return mRes;
}

void QMC5883L::QMC5883L_WriteByte(uint8_t QMC5883L_reg, uint8_t QMC5883L_data)
{
    char data_out[2];
    data_out[0]=QMC5883L_reg;
    data_out[1]=QMC5883L_data;
    #ifdef DEBUG
        printf("WRITE: %d, %x, %x\r\n", QMC5883L_ADDRESS, data_out[0], data_out[1]);
    #endif
    this->QMC5883L_i2c.write(QMC5883L_ADDRESS, data_out, 2, 0);
}

uint8_t QMC5883L::QMC5883L_ReadByte(uint8_t QMC5883L_reg)
{
    char data_out[1], data_in[1];
    data_out[1] = QMC5883L_reg;
    #ifdef DEBUG
        printf("RWRITE: %d, %x\r\n", QMC5883L_ADDRESS, data_out[0]);
    #endif
    this->QMC5883L_i2c.write(QMC5883L_ADDRESS, data_in, 1, 1);
    this->QMC5883L_i2c.read(QMC5883L_ADDRESS, data_out, 1, 0);
    #ifdef DEBUG
        printf("Read: %d, %x\r\n", QMC5883L_ADDRESS, data_in);
    #endif
    return (data_in[0]);
}

uint8_t QMC5883L::ChipID()
{
    uint8_t ChipID = QMC5883L_ReadByte(CHIP_ID);   // Should return 0x68  
    return ChipID;
}

void QMC5883L::init()
{   
    changeState(osr512, MagScale_8G, odr_200hz, mode_continuous);
    QMC5883L_WriteByte(SET_RESET, 0x01);
    wait_ms(10);
}

void QMC5883L::changeState(OSR osr, MagScale rng, ODR odr, Mode mode)
{
    char tmp = ((osr & 0x3) << 6) | ((rng & 0x3) << 4) | ((odr & 0x3) << 2) | (mode & 0x3);
    QMC5883L_WriteByte(CONTROL_A, temp);
}

void QMC5883L::standby()
{
    // Standby mode
    changeState(osr512, MagScale_2G, odr_1Ohz, mode_standby);
    wait_ms(10);
}

void QMC5883L::reset()
{
    // Reset
    QMC5883L_WriteByte(CONTROL_B, 0x80);
    wait_ms(10);
    
    // Config de base
    init();
}

int16_t QMC5883L::getMagXvalue()
{
    uint8_t LoByte, HiByte;
    LowByte = QMC5883L_ReadByte(OUT_X_LSB); // read Accelerometer X_Low  value
    HiByte = QMC5883L_ReadByte(OUT_X_MSB); // read Accelerometer X_High value
    return((HiByte<<8) | LoByte);
}

int16_t QMC5883L::getMagYvalue()
{
    uint8_t LoByte, HiByte;
    LowByte = QMC5883L_ReadByte(OUT_Y_LSB); // read Accelerometer X_Low  value
    HiByte = QMC5883L_ReadByte(OUT_Y_MSB); // read Accelerometer X_High value
    return ((HiByte<<8) | LowByte);
}

int16_t QMC5883L::getMagZvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = QMC5883L_ReadByte(OUT_Z_LSB); // read Accelerometer X_Low  value
    HiByte = QMC5883L_ReadByte(OUT_Z_MSB); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t QMC5883L::getMagTemp()
{
    uint8_t LoByte, HiByte;
    LoByte = QMC5883L_ReadByte(TEMP_LSB); // read Accelerometer X_Low  value
    HiByte = QMC5883L_ReadByte(TEMP_MSB); // read Accelerometer X_High value
    return ((HiByte<<8) | LowByte);
}