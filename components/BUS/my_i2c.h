#ifndef MY_I2C_H
#define MY_I2C_H

#include "unity.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 11
#define I2C_MASTER_SCL_IO 12

/**
 * @brief i2c master initialization
 */
void i2c_bus_init(void);


#endif