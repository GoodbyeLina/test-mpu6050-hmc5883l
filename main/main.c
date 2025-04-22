#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hmc5883l.h"
#include "my_mpu6050.h"
#include "my_i2c.h"

#define TAG "HMC5883L_APP"

void app_main(void)
{
    // 初始化I2C总线(使用默认配置)
    i2c_bus_init();

    // 初始化HMC5883L(使用默认配置)
    if (hmc5883l_init(NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return;
    }
    ESP_LOGI(TAG, "HMC5883L initialized successfully");

    hmc5883l_data_t hcm5883l_data;
    
    // 初始化mpu6050(使用默认配置)
    esp_err_t ret;
    mpu6050_handle_t mpu6050 = NULL;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    i2c_sensor_mpu6050_init();

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");


    while (1) {
        // 读取HMC5883L数据
        if (hmc5883l_read(&hcm5883l_data) == ESP_OK) {
            ESP_LOGI(TAG, "X:%.2fG Y:%.2fG Z:%.2fG Heading:%.1f°",
                    hcm5883l_data.x, hcm5883l_data.y, hcm5883l_data.z, hcm5883l_data.heading);
        } else {
            ESP_LOGE(TAG, "Failed to read hcm5883l_data");
        }
        // 读取mpu6050数据
        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
    
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    
    
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    mpu6050_delete(mpu6050);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

}
