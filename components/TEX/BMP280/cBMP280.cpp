#include "cBMP280.h"
#include <cstdio>

#define BMP280_ADDRESS		0xEC

cBMP280::cBMP280()
{
    temperature = 0;
    pressure = 0;
}

cBMP280::~cBMP280()
{

}

void cBMP280::init(cifI2C& I2CDriver)
{
    this->ptI2C = &I2CDriver;   

	uint8_t I2CBuffer[30];	


	//-------------------------------
	// ID
	//-------------------------------
	I2CBuffer[0] = 0xD0;
	ptI2C->masterWrite(BMP280_ADDRESS>>1, I2CBuffer, 1);
	ptI2C->masterRead(BMP280_ADDRESS>>1, I2CBuffer, 1);
	uint8_t ID = I2CBuffer[0];	


	//-------------------------------
	// CALIBRATION
	//-------------------------------
	I2CBuffer[0] = 0x88;
	ptI2C->masterWrite(BMP280_ADDRESS>>1, I2CBuffer, 1);
	ptI2C->masterRead(BMP280_ADDRESS>>1, I2CBuffer, 26);
    ptI2C->unlock();

	dig_T1  = (int16_t) I2CBuffer[1 ]<<8  | (int16_t) I2CBuffer[0 ];	//88 e 89
	dig_T2  = (int16_t) I2CBuffer[3 ]<<8  | (int16_t) I2CBuffer[2 ];	//8A e 8B
	dig_T3  = (int16_t) I2CBuffer[5 ]<<8  | (int16_t) I2CBuffer[4 ];	//8C e 8D
	
	dig_P1  = (int16_t) I2CBuffer[7 ]<<8  | (int16_t) I2CBuffer[6 ];	//8E e 8F
	dig_P2  = (int16_t) I2CBuffer[9 ]<<8  | (int16_t) I2CBuffer[8 ];	//90 e 91
	dig_P3  = (int16_t) I2CBuffer[11]<<8  | (int16_t) I2CBuffer[10];	//92 e 93
	dig_P4  = (int16_t) I2CBuffer[13]<<8  | (int16_t) I2CBuffer[12];	//94 e 95
	dig_P5  = (int16_t) I2CBuffer[15]<<8  | (int16_t) I2CBuffer[14];	//96 e 97
	dig_P6  = (int16_t) I2CBuffer[17]<<8  | (int16_t) I2CBuffer[16];	//98 e 99
	dig_P7  = (int16_t) I2CBuffer[19]<<8  | (int16_t) I2CBuffer[18];	//9A e 9B
	dig_P8  = (int16_t) I2CBuffer[21]<<8  | (int16_t) I2CBuffer[20];	//9C e 9D
	dig_P9  = (int16_t) I2CBuffer[23]<<8  | (int16_t) I2CBuffer[22];	//9E e 9F

	//-------------------------------
	// OVERSAMPLING
	//-------------------------------
	uint8_t osrs_p = 0x05;
	uint8_t osrs_t = 0x05;
	uint8_t mode = 3;
	uint8_t ctrl_meas = (osrs_t << 5)|(osrs_p << 2)|(mode);

	I2CBuffer[0] = 0xF4;
	I2CBuffer[1] = ctrl_meas;
	ptI2C->lock();
	ptI2C->masterWrite(BMP280_ADDRESS>>1, I2CBuffer, 2);		
	ptI2C->unlock();
	//------------------------------


	//-------------------------------
	// CONFIG
	//-------------------------------
	uint8_t t_sb = 0x01;
	uint8_t filter = 4;		
	uint8_t config = (t_sb << 5)|(filter << 2);

	I2CBuffer[0] = 0xF5;
	I2CBuffer[1] = config;
	ptI2C->lock();
	ptI2C->masterWrite(BMP280_ADDRESS>>1, I2CBuffer, 2);		
	ptI2C->unlock();
	//------------------------------	 
}

float cBMP280::getTemperature()
{    
    float res = temperature;
    return res;
}

float cBMP280::getPressure()
{
    float res = pressure;
    return res;
}

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double cBMP280::compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
		(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double cBMP280::compensate_P_double(BMP280_S32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}

void cBMP280::poll()
{
	uint8_t I2CBuffer[30];	

	I2CBuffer[0] = 0xF7;
	ptI2C->lock();
	ptI2C->masterWrite(BMP280_ADDRESS>>1, I2CBuffer, 1);
	ptI2C->masterRead(BMP280_ADDRESS>>1, I2CBuffer, 6);
	ptI2C->unlock();

	int32_t press = ((((uint32_t)I2CBuffer[0 ]<<16) | ((uint32_t)I2CBuffer[1 ]<<8)  | ((uint32_t)I2CBuffer[2])) >> 4 );
	int32_t temp = ((((uint32_t)I2CBuffer[3 ]<<16) | ((uint32_t)I2CBuffer[4 ]<<8)  | ((uint32_t)I2CBuffer[5])) >> 4);
			
	pressure = compensate_P_double(press);
	temperature = compensate_T_double(temp);        
}
