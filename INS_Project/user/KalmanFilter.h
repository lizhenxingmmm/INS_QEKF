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
#include <string.h>
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

#define sizeof_float            4

typedef struct kf_struct
{
    //外部传感器数据接口
    float InputVector_u[6];
    float Filtered_StateVector_x[6];
    float MeasuredVector_z[6];
    //矩阵
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
    mat I_x_x;//xsize x xsize 阶单位矩阵
    //暂时矩阵
    mat temp_mat_for_xhatminus_Update,temp_mat_for_xhatminus_Update2;
    mat temp_mat_for_Pminus_Update,temp_mat_for_Pminus_Update2;
    mat temp_x_z_mat,temp_z_x_mat,temp_z_z_mat,temp_z_z_mat2,temp_z_z_mat3;
    mat temp_z_1_mat,temp_z_1_mat2,temp_x_1_mat;
    mat temp_x_x_mat,temp_x_x_mat2;
    //矩阵数据存储空间指针
    // float *x_hat_data, *x_hat_minus_data;
    // float *u_data;
    // float *z_data;
    // float *P_data, *P_minus_data;
    // float *A_data, *AT_data;
    // float *B_data;
    // float *H_data, *HT_data;
    // float *Q_data;
    // float *R_data;
    // float *K_data;
    // float *I_data;
    //静态分配空间，上限6
    float x_hat_data[6], x_hat_minus_data[6];
    float u_data[6];
    float z_data[6];
    float P_data[36], P_minus_data[36];
    float A_data[36], AT_data[36];
    float B_data[36];
    float H_data[36], HT_data[36];
    float Q_data[36];
    float R_data[36];
    float K_data[36];
    float I_data[36];
    //暂时矩阵的数据空间
    // float *temp_mat_for_xhatminus_Update_data,*temp_mat_for_xhatminus_Update_data2;
    // float *temp_mat_for_Pminus_Update_data,*temp_mat_for_Pminus_Update_data2;
    // float *temp_x_z_mat_data,*temp_z_x_mat_data,*temp_z_z_mat_data,*temp_z_z_mat2_data,*temp_z_z_mat3_data;
    // float *temp_z_1_mat_data,*temp_z_1_mat2_data,*temp_x_1_mat_data;
    // float *temp_x_x_mat_data,*temp_x_x_mat2_data;
    float temp_mat_for_xhatminus_Update_data[6],temp_mat_for_xhatminus_Update_data2[6];
    float temp_mat_for_Pminus_Update_data[36],temp_mat_for_Pminus_Update_data2[36];
    float temp_x_z_mat_data[36],temp_z_x_mat_data[36],temp_z_z_mat_data[36],temp_z_z_mat2_data[36],temp_z_z_mat3_data[36];
    float temp_z_1_mat_data[6],temp_z_1_mat2_data[6],temp_x_1_mat_data[6];
    float temp_x_x_mat_data[36],temp_x_x_mat2_data[36];
    //状态空间方程中变量的维度
    uint8_t x_size;
    uint8_t u_size;
    uint8_t z_size;
    //user function 用于扩展卡尔曼滤波
    void (*User_func_before_xhatminusUpdate)(struct kf_struct* kf);
    void (*User_func_before_PminusUpdate)(struct kf_struct* kf);
    void (*User_func_before_SetK)(struct kf_struct* kf);
    void (*User_func_before_xhat_Update)(struct kf_struct* kf);
    void (*User_func_before_P_Update)(struct kf_struct* kf);
    //跳步标志位
    uint8_t xhatminusUpdate_skip;
    uint8_t PminusUpdate_skip;
    uint8_t SetK_skip;
    uint8_t xhat_Update_skip;
    uint8_t P_Update_skip;
}KalmanFilter;

void KalmanFilter_Init(KalmanFilter* kf,uint8_t x_size,uint8_t u_size,uint8_t z_size);
void KalmanFilter_xhatminus_Update(KalmanFilter* kf);
void KalmanFilter_Pminus_Update(KalmanFilter* kf);
void KalmanFilter_SetK(KalmanFilter *kf);
void KalmanFilter_xhat_Update(KalmanFilter *kf);
void KalmanFilter_P_Update(KalmanFilter *kf);
float *KalmanFilter_Update(KalmanFilter *kf);


#endif
