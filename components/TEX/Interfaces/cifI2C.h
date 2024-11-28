/* 
* cifI2C.h
*
* Created: 23/11/2020 14:49:40
* Author: henrique.coser
*/

#ifndef __CIFI2C_H__
#define __CIFI2C_H__

#include <stdint.h>

class cifI2C
{
//variables
public:
protected:
private:

//functions
public:
	cifI2C() = default;
	virtual ~cifI2C() {}
	virtual void lock() = 0;
	virtual void unlock() = 0;
	virtual bool openAsMaster(uint32_t baudrate) = 0;
	virtual bool masterRead(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len ) = 0;
	virtual bool masterRead(uint8_t address, uint8_t *ptBuffer, uint32_t len ) = 0;		
	virtual bool masterWrite(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len ) = 0;
	virtual bool masterWrite(uint8_t address, uint8_t *ptBuffer, uint32_t len ) = 0;
	
protected:
private:

}; //cifI2C

#endif //__CIFI2C_H__
