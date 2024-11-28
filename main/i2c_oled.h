#ifndef I2C_OLED_H
#define I2C_OLED_H

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

void i2c_oled_start(i2c_master_bus_handle_t i2c_bus);

#ifdef __cplusplus
}
#endif

#endif;