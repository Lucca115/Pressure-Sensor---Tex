/*
 * cbspI2C.h
 *
 *  Created on: 12 de abr. de 2021
 *      Author: henrique.coser
 */

#ifndef BSP_ESP32_CBSPI2C_H_
#define BSP_ESP32_CBSPI2C_H_

#include <stdint.h>
#include "driver/i2c_master.h"
#include "cifI2C.h"

class cbspI2C : public cifI2C
{
	private:
		i2c_port_t	i2c_port;
		gpio_num_t i2c_gpio_sda;
		gpio_num_t i2c_gpio_scl;
		uint32_t i2c_frequency;
		i2c_master_bus_handle_t i2c_bus;

	public:
		cbspI2C();
		~cbspI2C();
		void init(i2c_port_t i2c_port, gpio_num_t i2c_gpio_sda, gpio_num_t i2c_gpio_scl);
		virtual void lock();
		virtual void unlock();
		virtual bool openAsMaster(uint32_t baudrate);
		virtual bool masterRead(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len );
		virtual bool masterRead(uint8_t address, uint8_t *ptBuffer, uint32_t len );
		virtual bool masterWrite(uint8_t address, uint8_t dataAddress, uint8_t *ptBuffer, uint32_t len );
		virtual bool masterWrite(uint8_t address, uint8_t *ptBuffer, uint32_t len );
};

#endif /* BSP_ESP32_CBSPI2C_H_ */
