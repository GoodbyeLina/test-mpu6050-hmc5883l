#pragma once

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 12      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 11      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */


/**
 * @brief i2c master initialization
 */
void my_mpu6050_init(void);
mpu6050_handle_t my_mpu6050_get_handle(void);