/*
 * Barometer.h
 * A library for barometer
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : LG
 * Create Time:
 * Change Log :
 *
 * loovee 9-24-2014
 * Change all int to short, all unsigned int to unsigned short to fit some 32bit system
 *
 * The MIT License (MIT)
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef __BAROMETER_H__
#define __BAROMETER_H__

#include <Arduino.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085 / BMP180 = 1110111 + RW-Bit

//#define DEBUG
// Number of samples for moving average filter
#define SAMPLES 10

// Low Pass filter constant
#define LOW_PASS_FACTOR 0.2

// Oversampling Wert des Barometers 0..3 (0: schnell/ungenau 3:langsam/genau)
//const unsigned char OSS = 0; // 1 Sample, 4,5 ms, Rauschen 0,5m
//const unsigned char OSS = 1; // 2 Samples, 7,5 ms, Rauschen 0,4m 
//const unsigned char OSS = 2; // 4 Samples, 13,5 ms, Rauschen 0,3m
const unsigned char OSS = 3; // 8 Samples, 25,5ms, Rauschen 0,25m

class Barometer
{
    public:
    void init(void);
    float altitude;
    float temperature; 
    float pressure;
    void ReadAltitude();

    private:

    short ac1;              // Kalibrierwerte
    short ac2;
    short ac3;
    unsigned short ac4;
    unsigned short ac5;
    unsigned short ac6;
    short b1;
    short b2;
    short mb;
    short mc;
    short md;
    long PressureCompensate;
    float values[SAMPLES];  // Filtervektor
    uint32_t lasttempread;
    float bmp085GetTemperature(unsigned short ut);
    long  bmp085GetPressure(unsigned long up);
    float calcAltitude(float pressure);
    unsigned short bmp085ReadUT(void);
    unsigned long bmp085ReadUP(void);
    char  bmp085Read(unsigned char address);
    short bmp085ReadInt(unsigned char address);
    void  writeRegister(short deviceAddress, byte address, byte val);
    short readRegister(short deviceAddress, byte address);
    float filter(float altitude);
};

#endif
