
#include "my_mpu6050.h"


static const char *TAG = "mpu6050 test";
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
void my_mpu6050_init(void)
{
    esp_err_t ret;

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

mpu6050_handle_t my_mpu6050_get_handle(void)
{
    return mpu6050;
}
