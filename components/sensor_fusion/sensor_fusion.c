#include "sensor_fusion.h"
#include <math.h>

// 卡尔曼滤波器实例
static kalman_filter_t kf;

// 初始化卡尔曼滤波器参数
static void kalman_filter_init() {
    // 初始化误差协方差矩阵
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            kf.P[i][j] = i==j ? 1.0f : 0.0f;
        }
    }

    // 过程噪声协方差
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            kf.Q[i][j] = i==j ? 0.01f : 0.0f;
        }
    }

    // 测量噪声协方差
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            kf.R[i][j] = i==j ? 0.1f : 0.0f;
        }
    }
}

void sensor_fusion_init(i2c_port_t i2c_port) {
    kalman_filter_init();
}

// 卡尔曼预测步骤
static void kalman_predict(float dt) {
    // 状态转移矩阵F
    float F[6][6] = {0};
    for(int i=0; i<6; i++) F[i][i] = 1.0f;
    
    // 预测状态
    float x_pred[6] = {0};
    for(int i=0; i<6; i++) {
        x_pred[i] = kf.x[i];
    }

    // 预测协方差
    float P_pred[6][6] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            for(int k=0; k<6; k++) {
                P_pred[i][j] += F[i][k] * kf.P[k][j];
            }
            P_pred[i][j] += kf.Q[i][j];
        }
    }

    // 更新状态和协方差
    for(int i=0; i<6; i++) {
        kf.x[i] = x_pred[i];
        for(int j=0; j<6; j++) {
            kf.P[i][j] = P_pred[i][j];
        }
    }
}

// 卡尔曼更新步骤
static void kalman_update(float z[9]) {
    // 测量矩阵H
    float H[9][6] = {0};
    for(int i=0; i<3; i++) {
        H[i][i] = 1.0f;    // 加速度计
        H[i+3][i+3] = 1.0f; // 陀螺仪
        H[i+6][i] = 1.0f;   // SC7A20H加速度计
    }

    // 计算卡尔曼增益K
    float K[6][9] = {0};
    float S[9][9] = {0};
    
    // S = H*P*H' + R
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            for(int k=0; k<6; k++) {
                S[i][j] += H[i][k] * kf.P[k][j];
            }
            S[i][j] += kf.R[i][j];
        }
    }

    // K = P*H'*inv(S)
    // (简化实现，实际应用应考虑矩阵求逆稳定性)
    for(int i=0; i<6; i++) {
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                K[i][j] += kf.P[i][k] * H[j][k] / S[j][j];
            }
        }
    }

    // 更新状态估计
    float y[9];
    for(int i=0; i<9; i++) {
        y[i] = z[i];
        for(int j=0; j<6; j++) {
            y[i] -= H[i][j] * kf.x[j];
        }
    }

    for(int i=0; i<6; i++) {
        for(int j=0; j<9; j++) {
            kf.x[i] += K[i][j] * y[j];
        }
    }

    // 更新协方差估计
    float I[6][6] = {0};
    for(int i=0; i<6; i++) I[i][i] = 1.0f;
    
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            for(int k=0; k<9; k++) {
                kf.P[i][j] -= K[i][k] * H[k][j] * kf.P[i][j];
            }
        }
    }
}

// 四元数转欧拉角
static attitude_t quat_to_euler(float q[4]) {
    attitude_t att;
    
    // 计算翻滚角(roll)
    att.roll = atan2f(2*(q[0]*q[1] + q[2]*q[3]), 
                      1 - 2*(q[1]*q[1] + q[2]*q[2]));
    
    // 计算横滚角(pitch)
    float sinp = 2*(q[0]*q[2] - q[3]*q[1]);
    if(fabs(sinp) >= 1) {
        att.pitch = copysignf(M_PI/2, sinp);
    } else {
        att.pitch = asinf(sinp);
    }
    
    // 计算俯仰角(yaw)
    att.yaw = atan2f(2*(q[0]*q[3] + q[1]*q[2]),
                    1 - 2*(q[2]*q[2] + q[3]*q[3]));
    
    // 弧度转角度
    att.roll *= 180/M_PI;
    att.pitch *= 180/M_PI;
    att.yaw *= 180/M_PI;
    
    return att;
}

attitude_t sensor_fusion_update(mpu6050_data_t mpu, hmc5883l_data_t hmc, sc7a20h_data_t sc7) {
    static uint64_t last_time = 0;
    uint64_t now = esp_timer_get_time();
    float dt = (now - last_time) / 1e6f;
    last_time = now;

    // 准备测量向量 [ax,ay,az,gx,gy,gz,ax2,ay2,az2]
    float z[9] = {
        mpu.accel_x, mpu.accel_y, mpu.accel_z,
        mpu.gyro_x, mpu.gyro_y, mpu.gyro_z,
        sc7.accel_x, sc7.accel_y, sc7.accel_z
    };

    // 卡尔曼滤波步骤
    kalman_predict(dt);
    kalman_update(z);

    // 使用磁力计数据校正偏航角
    float yaw = atan2f(hmc.mag_y, hmc.mag_x) * 180/M_PI;
    
    // 构造四元数 (简化实现)
    float q[4] = {
        cos(kf.x[0]/2)*cos(kf.x[1]/2)*cos(kf.x[2]/2) + sin(kf.x[0]/2)*sin(kf.x[1]/2)*sin(kf.x[2]/2),
        sin(kf.x[0]/2)*cos(kf.x[1]/2)*cos(kf.x[2]/2) - cos(kf.x[0]/2)*sin(kf.x[1]/2)*sin(kf.x[2]/2),
        cos(kf.x[0]/2)*sin(kf.x[1]/2)*cos(kf.x[2]/2) + sin(kf.x[0]/2)*cos(kf.x[1]/2)*sin(kf.x[2]/2),
        cos(kf.x[0]/2)*cos(kf.x[1]/2)*sin(kf.x[2]/2) - sin(kf.x[0]/2)*sin(kf.x[1]/2)*cos(kf.x[2]/2)
    };

    attitude_t att = quat_to_euler(q);
    att.yaw = yaw; // 使用磁力计校正的偏航角
    
    return att;
}