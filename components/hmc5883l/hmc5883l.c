#include "hmc5883l.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>

#define TAG "HMC5883L"

// 寄存器地址
#define CONFIG_REG_A     0x00
#define CONFIG_REG_B     0x01
#define MODE_REG         0x02
#define DATA_REG         0x03

// 默认配置
const hmc5883l_config_t default_config = {
    .i2c_port = HMC5883L_DEFAULT_I2C_PORT,
    .i2c_addr = HMC5883L_DEFAULT_ADDR,
    .sda_pin = 11,
    .scl_pin = 12,
    .i2c_freq = HMC5883L_DEFAULT_I2C_FREQ
};

hmc5883l_config_t current_config;
float current_gain = 1090.0; // 默认增益(±1.3Ga)

// 写入寄存器
esp_err_t write_byte(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(current_config.i2c_port, 
                                    current_config.i2c_addr,
                                    write_buf, sizeof(write_buf),
                                    pdMS_TO_TICKS(1000));
}

esp_err_t hmc5883l_init(const hmc5883l_config_t *config) {
    // 使用默认配置或用户配置
    if (config != NULL) {
        current_config = *config;
    } else {
        current_config = default_config;
    }

    // 配置传感器
    esp_err_t ret;
    ret = write_byte(CONFIG_REG_A, 0x78); // 8采样平均，75Hz输出速率
    if (ret != ESP_OK) return ret;

    ret = write_byte(CONFIG_REG_B, 0x20); // ±1.3Ga量程
    if (ret != ESP_OK) return ret;

    return write_byte(MODE_REG, 0x00); // 连续测量模式
}

esp_err_t hmc5883l_read(hmc5883l_data_t *data) {
    uint8_t raw_data[6];
    uint8_t reg_addr = DATA_REG;
    esp_err_t ret = i2c_master_write_read_device(current_config.i2c_port,
                                               current_config.i2c_addr,
                                               &reg_addr, 1,
                                               raw_data, sizeof(raw_data),
                                               pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;
    
    // 转换原始数据
    int16_t x = (raw_data[0] << 8) | raw_data[1];
    int16_t z = (raw_data[2] << 8) | raw_data[3];
    int16_t y = (raw_data[4] << 8) | raw_data[5];

    data->x = x / current_gain;
    data->y = y / current_gain;
    data->z = z / current_gain;

    // 计算航向
    float heading = atan2(data->y, data->x) * 180.0 / M_PI;
    data->heading = heading < 0 ? heading + 360 : heading;

    return ESP_OK;
}

esp_err_t hmc5883l_set_gain(uint8_t gain) {
    esp_err_t ret = write_byte(CONFIG_REG_B, gain << 5);
    if (ret == ESP_OK) {
        // 更新当前增益值(根据数据手册)
        const float gains[] = {1370, 1090, 820, 660, 440, 390, 330, 230};
        current_gain = gains[gain];
    }
    return ret;
}

esp_err_t hmc5883l_set_mode(uint8_t mode) {
    return write_byte(MODE_REG, mode);
}



