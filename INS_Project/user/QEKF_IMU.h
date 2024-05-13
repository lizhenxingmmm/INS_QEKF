/**
 * @Author: Li Zhenxing
 * @Date: 2024/5/13 08:35:49
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/5/13 08:35:49
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#ifndef QEKF_IMU_H
#define QEKF_IMU_H

#include "KalmanFilter.h"

#define     CHI_SQUARE_VALVE            123
#define     CHI_SQUARE_SKIP_COUNT_MAX   50

typedef struct QEKF_IMU
{
    uint8_t initialized_flag;
    KalmanFilter IMU_QEKF_kf;
    float q[4];
    float w_bias[3];
    float dt;//迭代时间间隔
    //mat_q=0.5*omega*dt+I4
    float mat_q_data[16];//状态转移矩阵A左上角的4x4矩阵
    float mat_Ok_data[8];//状态转移矩阵A右上角的4x2矩阵
    uint8_t convergency_flag;
}QEKF_IMU_t;

void QEKF_IMU_A_Update(QEKF_IMU_t* qekf,float gyro[3]);
void QEKF_IMU_H_Update(QEKF_IMU_t* qekf);
void Chi_square_Check(KalmanFilter* kf);
void eliminate_q3(KalmanFilter* kf);

#endif
