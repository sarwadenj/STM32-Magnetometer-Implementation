/*     
*   Copyright (c) 2015, Baser Kandehir, baser.kandehir@ieee.metu.edu.tr
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:
*
*   The above copyright notice and this permission notice shall be included in
*   all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*   THE SOFTWARE.
*
*/

// Some part of the code is adapted from Adafruit HMC5883 library

#ifndef QMC5883L_H
#define QMC5883L_H

#include "mbed.h"
#include "math.h"

// #define DEBUG 1
#define PI 3.14159265359 
#define GAUSS_TO_MICROTESLA 100
#define QMC5883L_ADDRESS 0x1A//0x18//

/* Register Definitions */
#define OUT_X_LSB    0x00
#define OUT_X_MSB    0x01
#define OUT_Y_LSB    0x02
#define OUT_Y_MSB    0x03
#define OUT_Z_LSB    0x04
#define OUT_Z_MSB    0x05
#define STATUS       0x06
#define TEMP_LSB     0x07
#define TEMP_MSB     0x08
#define CONTROL_A    0x09
#define CONTROL_B    0x0A
#define SET_RESET    0x0B
#define CHIP_ID      0x0D

/* Magnetometer Gain Settings */
enum MagScale
{
    MagScale_2G =  0x0,      // +/- 2 Ga
    MagScale_8G  = 0x1,     // +/- 8 Ga
};

enum OSR{
    osr512 = 0x0,
    osr256 = 0x1,
    osr128 = 0x2,
    osr64 = 0x3
};

enum Mode{
    mode_standby = 0x0,
    mode_continuous = 0x1
};

enum ODR{
    odr_1Ohz = 0x0,
    odr_50hz = 0x1,
    odr_100hz = 0x2,
    odr_200hz = 0x3
};

class QMC5883L
{
    public:
        QMC5883L(PinName sda, PinName scl);
        void    init();
        void changeState(OSR osr, MagScale rng, ODR odr, Mode mode);
        void standby();
        void reset();
        double  getHeading();
        void    readMagData(float* dest);  
        int16_t getMagXvalue();
        int16_t getMagYvalue();
        int16_t getMagZvalue();
        int16_t getMagTemp();
        uint8_t   ChipID();
    private:
        float setMagRange(MagScale Mscale);
        uint8_t QMC5883L_ReadByte(uint8_t QMC5883L_reg);
        void QMC5883L_WriteByte(uint8_t QMC5883L_reg, uint8_t QMC5883L_data);
        I2C QMC5883L_i2c;
};

#endif