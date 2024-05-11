/**
 * @Author: Li Zhenxing
 * @Date: 2024/5/11 16:46:58
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/5/11 22:19:35
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#include "KalmanFilter.h"

/**
 * @brief kalman filter initialization
 * 
*/
void KalmanFilter_Init(KalmanFilter* kf,uint8_t x_size,uint8_t u_size,uint8_t z_size)
{
    //将size写入kalman filter结构体
    kf->x_size=x_size;
    kf->u_size=u_size;
    kf->z_size=z_size;
    //分配矩阵数据空间
    kf->A_data=(float*)user_malloc(x_size*x_size);
    kf->AT_data=(float*)user_malloc(x_size*x_size);
    kf->B_data=(float*)user_malloc(x_size*u_size);
    kf->H_data=(float*)user_malloc(z_size*x_size);
    kf->HT_data=(float*)user_malloc(x_size*z_size);
    kf->K_data=(float*)user_malloc(x_size*z_size);
    kf->P_data=(float*)user_malloc(x_size*x_size);
    kf->P_minus_data=(float*)user_malloc(x_size*x_size);
    kf->x_hat_data=(float*)user_malloc(x_size);
    kf->x_hat_minus_data=(float*)user_malloc(x_size);
    //用于运算的暂时的矩阵数据空间
    kf->temp_mat_for_xhatminus_Updata_data=(float*)user_malloc(x_size);
    kf->temp_mat_for_xhatminus_Updata_data2=(float*)user_malloc(x_size);
    //初始化矩阵
    Matrix_Init(&(kf->A),x_size,x_size,kf->A_data);
    Matrix_Init(&(kf->A_T),x_size,x_size,kf->AT_data);
    Matrix_Init(&(kf->B),x_size,u_size,kf->B_data);
    Matrix_Init(&(kf->H),z_size,x_size,kf->H_data);
    Matrix_Init(&(kf->H_T),x_size,z_size,kf->HT_data);
    Matrix_Init(&(kf->K),x_size,z_size,kf->K_data);
    Matrix_Init(&(kf->P),x_size,x_size,kf->P_data);
    Matrix_Init(&(kf->P_minus),x_size,x_size,kf->P_minus_data);
    Matrix_Init(&(kf->x_hat),x_size,1,kf->x_hat_data);
    Matrix_Init(&(kf->x_hat_minus),x_size,1,kf->x_hat_minus_data);
    //初始化暂时矩阵
    Matrix_Init((&kf->temp_mat_for_xhatminus_Updata),x_size,1,kf->temp_mat_for_xhatminus_Updata_data);
    Matrix_Init((&kf->temp_mat_for_xhatminus_Updata2),x_size,1,kf->temp_mat_for_xhatminus_Updata_data2);
}
/**
 * @brief 更新卡尔曼滤波器中的x先验估计
*/
void KalmanFilter_xhatminus_Updata(KalmanFilter* kf)
{
    if(kf->u_size==0)
    {
        Matrix_Multiply(&(kf->A),&(kf->x_hat),&(kf->x_hat_minus));
    }
    else
    {
        Matrix_Multiply(&(kf->A),&(kf->x_hat),&(kf->temp_mat_for_xhatminus_Updata));
        Matrix_Multiply(&(kf->B),&(kf->u),&(kf->temp_mat_for_xhatminus_Updata2));
        Matrix_Add(&(kf->temp_mat_for_xhatminus_Updata2),&(kf->temp_mat_for_xhatminus_Updata),&(kf->x_hat_minus));
    }
}
/**
 * @brief 更新先验估计协方差矩阵
*/
void KalmanFilter_Pminus_Updata(KalmanFilter* kf)
{
    Matrix_Multiply(&(kf->A),&(kf->P),&(kf->temp_mat_for_Pminus_Updata));
    Matrix_Multiply(&(kf->temp_mat_for_Pminus_Updata),&(kf->A_T),&(kf->temp_mat_for_Pminus_Updata2));
    Matrix_Add(&(kf->temp_mat_for_Pminus_Updata2),&(kf->Q),&(kf->P_minus));
}
/**
 * @brief 计算卡尔曼增益
*/
void KalmanFilter_SetK(KalmanFilter *kf)
{
    
}
