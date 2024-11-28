#include "cSMP3011.h"


//#include <cstdio>

#define SMP3011_PRESSURE_RESOLUTION     (16777216.0f)
#define SMP3011_TEMPERATURE_RESOLUTION  (65536.0f)
#define SMP3011_ADDRESS                 (0x78)
#define SMP3011_MAX_TEMPERATURE         (150.0f)
#define SMP3011_MIN_TEMPERATURE         (-40.0f)

#define SMP3011_MAX_PRESSURE_PERCENTAGE (0.85f)
#define SMP3011_MIN_PRESSURE_PERCENTAGE (0.15f)

#define SMP3011_MAX_RANGE_PA            (500000.0f)
#define SMP3011_MIN_RANGE_PA            (0.0f)

cSMP3011::cSMP3011()
{
    temperature = 0;
    pressure = 0;
}

cSMP3011::~cSMP3011()
{

}

void cSMP3011::init(cifI2C& I2CDriver)
{
    this->ptI2C = &I2CDriver;
     
    ptI2C->lock();
    uint8_t PressSensorCommand = 0xAC;
    ptI2C->masterWrite(0x78, &PressSensorCommand, 1);
    ptI2C->unlock();
}

float cSMP3011::getTemperature()
{    
    float res = temperature;
     return res;
}

float cSMP3011::getPressure()
{
    float res = pressure;
    return res;
}


void cSMP3011::poll()
{
    uint8_t PressSensorBuffer[6]; 
    ptI2C->lock();       
    ptI2C->masterRead(SMP3011_ADDRESS, PressSensorBuffer, sizeof(PressSensorBuffer));        
    ptI2C->unlock();
     
    if((PressSensorBuffer[0]&0x20) == 0)        
    {
        ptI2C->lock();
        uint8_t PressSensorCommand = 0xAC;
        ptI2C->masterWrite(SMP3011_ADDRESS, &PressSensorCommand, 1);
        ptI2C->unlock();

        float pressurePercentage = (((uint32_t)PressSensorBuffer[1]<<16)|((uint32_t)PressSensorBuffer[2]<<8)|((uint32_t)PressSensorBuffer[3]));
        pressurePercentage /= SMP3011_PRESSURE_RESOLUTION;
        

        float temperaturePercentage = ((uint32_t)PressSensorBuffer[4]<<8)|((uint32_t)PressSensorBuffer[5]);
        temperaturePercentage /= SMP3011_TEMPERATURE_RESOLUTION;
        
        pressure    = (((pressurePercentage - SMP3011_MIN_PRESSURE_PERCENTAGE)/(SMP3011_MAX_PRESSURE_PERCENTAGE - SMP3011_MIN_PRESSURE_PERCENTAGE)) * (SMP3011_MAX_RANGE_PA-SMP3011_MIN_RANGE_PA)) + SMP3011_MIN_RANGE_PA;
        temperature = ((SMP3011_MAX_TEMPERATURE - SMP3011_MIN_TEMPERATURE)*temperaturePercentage) + SMP3011_MIN_TEMPERATURE;
    }
}
