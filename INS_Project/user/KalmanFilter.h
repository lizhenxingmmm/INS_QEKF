/**
 * @Author: Li Zhenxing
 * @Date: 2024/5/11 16:47:16
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/5/11 16:47:16
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "arm_math.h"
#include <stdlib.h>

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else 
#define user_malloc malloc
#endif
#endif

typedef struct kf_struct
{
    float *InputVector_u;
    float *Filtered_StateVector_x;
    float *MeasuredVector_z;
    mat x_hat;
    mat x_hat_minus;
    mat A,A_T;//状态转移矩阵A
    mat B;//输入矩阵
    mat Q;//计算过程噪声协方差矩阵
    mat R;//测量噪声协方差矩阵
    mat H,H_T;//观测矩阵
    mat z;//测量数据
    mat u;//输入数据
    mat K;//卡尔曼增益 Kalman Gain
    mat P;//误差协方差矩阵
    mat P_minus;//先验误差协方差矩阵
    //暂时矩阵
    mat temp_mat_for_xhatminus_Updata,temp_mat_for_xhatminus_Updata2;
    mat temp_mat_for_Pminus_Updata,temp_mat_for_Pminus_Updata2;
    //用于扩展卡尔曼滤波
    void (*getMatrixA)(struct kf_struct* kf);
    void (*getMatrixB)(struct kf_struct* kf);
    void (*getMatrixH)(struct kf_struct* kf);
    void (*getMatrixQ_afterLinearlization)(struct kf_struct* kf);
    void (*getMatrixR_afterLinearlization)(struct kf_struct* kf);
    //user function
    void (*user_function_1)(struct kf_struct* kf);
    void (*user_function_2)(struct kf_struct* kf);
    void (*user_function_3)(struct kf_struct* kf);
    void (*user_function_4)(struct kf_struct* kf);
    void (*user_function_5)(struct kf_struct* kf);
    //矩阵数据存储空间指针
    float *x_hat_data, *x_hat_minus_data;
    float *u_data;
    float *z_data;
    float *P_data, *P_minus_data;
    float *A_data, *AT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *temp_mat_for_xhatminus_Updata_data,*temp_mat_for_xhatminus_Updata_data2;
    float *temp_mat_for_Pminus_Updata_data,*temp_mat_for_Pminus_Updata_data2;
    //状态空间方程中变量的维度
    uint8_t x_size;
    uint8_t u_size;
    uint8_t z_size;
}KalmanFilter;

void KalmanFilter_Init(KalmanFilter* kf,uint8_t x_size,uint8_t u_size,uint8_t z_size);
void KalmanFilter_getMeasurementData(KalmanFilter* kf);
void KalmanFilter_xhatminus_Updata(KalmanFilter* kf);
void KalmanFilter_Pminus_Updata(KalmanFilter* kf);
void KalmanFilter_SetK(KalmanFilter *kf);
void KalmanFilter_xhat_Update(KalmanFilter *kf);
void KalmanFilter_P_Update(KalmanFilter *kf);
float *KalmanFilter_Update(KalmanFilter *kf);

#endif
