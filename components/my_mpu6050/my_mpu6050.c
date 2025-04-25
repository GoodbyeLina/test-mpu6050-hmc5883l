
#include "my_mpu6050.h"


static mpu6050_handle_t mpu6050 = NULL;
static const char *TAG = "mpu6050 test";

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

void calibrate_gyro(mpu6050_handle_t mpu, float *offsets) {
    mpu6050_gyro_value_t gyro;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for(int i=0; i<CALIBRATION_SAMPLES; i++) {
        mpu6050_get_gyro(mpu, &gyro);
        sum_x += gyro.gyro_x;
        sum_y += gyro.gyro_y; 
        sum_z += gyro.gyro_z;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    offsets[0] = sum_x / CALIBRATION_SAMPLES;
    offsets[1] = sum_y / CALIBRATION_SAMPLES;
    offsets[2] = sum_z / CALIBRATION_SAMPLES;
}
