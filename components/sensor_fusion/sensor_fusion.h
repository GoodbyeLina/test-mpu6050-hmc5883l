#ifndef __SENSOR_FUSION_H__
#define __SENSOR_FUSION_H__

#include "esp_err.h"
#include "driver/i2c.h"
#include <math.h>
#define M_PI 3.14159265358979323846f

// 简化传感器数据结构
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

typedef struct {
    float mag_x, mag_y, mag_z;
} hmc5883l_data_t;

typedef struct {
    float accel_x, accel_y, accel_z;
} sc7a20h_data_t;

typedef struct {
    float q0, q1, q2, q3;  // 四元数
    float roll, pitch, yaw; // 欧拉角(度)
} attitude_t;

typedef struct {
    float P[6][6];  // 误差协方差矩阵
    float Q[6][6];  // 过程噪声协方差
    float R[9][9];  // 测量噪声协方差
    float x[6];     // 状态向量 [ax, ay, az, gx, gy, gz]
} kalman_filter_t;

/**
 * @brief 初始化传感器融合模块
 */
void sensor_fusion_init(i2c_port_t i2c_port);

/**
 * @brief 卡尔曼滤波更新
 * @param mpu MPU6050数据
 * @param hmc HMC5883L数据  
 * @param sc7 SC7A20H数据
 * @return 融合后的姿态数据
 */
attitude_t sensor_fusion_update(mpu6050_data_t mpu, hmc5883l_data_t hmc, sc7a20h_data_t sc7);

#endif