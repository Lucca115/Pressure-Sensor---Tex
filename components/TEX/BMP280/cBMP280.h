#ifndef CBMP280_H
#define CBMP280_H

#pragma once

#include <cstdint>
#include "cifI2C.h"

class cBMP280;

typedef int32_t BMP280_S32_t;

class cBMP280 
{
private:
    float temperature;
    float pressure;
    cifI2C *ptI2C;    
    uint32_t sampleID = 0;


    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3; 
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    BMP280_S32_t t_fine;


public:
    cBMP280();
    ~cBMP280();
    void init(cifI2C& I2CDriver);
    float getTemperature();
    float getPressure();
    void poll();

private:    
    double compensate_T_double(BMP280_S32_t adc_T);
    double compensate_P_double(BMP280_S32_t adc_P);
};



#endif