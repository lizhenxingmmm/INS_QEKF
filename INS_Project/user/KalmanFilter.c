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
    //分配输入数据接口空间
    // kf->InputVector_u=(float*)user_malloc(u_size*sizeof_float);
    // kf->Filtered_StateVector_x=(float*)user_malloc(x_size*sizeof_float);
    // kf->MeasuredVector_z=(float*)user_malloc(z_size*sizeof_float);
    //分配矩阵数据空间
    //kf->A_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->AT_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->B_data=(float*)user_malloc(x_size*u_size*sizeof_float);
    // kf->H_data=(float*)user_malloc(z_size*x_size*sizeof_float);
    // kf->HT_data=(float*)user_malloc(x_size*z_size*sizeof_float);
    // kf->K_data=(float*)user_malloc(x_size*z_size*sizeof_float);
    // kf->P_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    //kf->P_minus_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->x_hat_data=(float*)user_malloc(x_size*sizeof_float);
    // kf->x_hat_minus_data=(float*)user_malloc(x_size*sizeof_float);
    // kf->I_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    //kf->Q_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->R_data=(float*)user_malloc(z_size*z_size*sizeof_float);
    // //用于运算的暂时的矩阵数据空间
    // kf->temp_mat_for_xhatminus_Update_data=(float*)user_malloc(x_size*sizeof_float);
    // kf->temp_mat_for_xhatminus_Update_data2=(float*)user_malloc(x_size*sizeof_float);
    // kf->temp_mat_for_Pminus_Update_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->temp_mat_for_Pminus_Update_data2=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->temp_x_z_mat_data=(float*)user_malloc(x_size*z_size*sizeof_float);
    // kf->temp_z_z_mat_data=(float*)user_malloc(z_size*z_size*sizeof_float);
    // kf->temp_z_z_mat2_data=(float*)user_malloc(z_size*z_size*sizeof_float);
    // kf->temp_z_z_mat3_data=(float*)user_malloc(z_size*z_size*sizeof_float);
    // kf->temp_z_x_mat_data=(float*)user_malloc(z_size*x_size*sizeof_float);
    // kf->temp_z_1_mat_data=(float*)user_malloc(z_size*sizeof_float);
    // kf->temp_z_1_mat2_data=(float*)user_malloc(z_size*sizeof_float);
    // kf->temp_x_x_mat_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    // kf->temp_x_x_mat2_data=(float*)user_malloc(x_size*x_size*sizeof_float);
    //填入常数矩阵数据
    kf->I_data[0]=1;kf->I_data[7]=1;kf->I_data[14]=1;
    kf->I_data[21]=1;kf->I_data[28]=1;kf->I_data[35]=1;
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
    Matrix_Init(&(kf->I_x_x),x_size,x_size,kf->I_data);
    Matrix_Init(&(kf->Q),x_size,x_size,kf->Q_data);
    Matrix_Init(&(kf->R),z_size,z_size,kf->R_data);
    //初始化暂时矩阵
    Matrix_Init((&kf->temp_mat_for_xhatminus_Update),x_size,1,kf->temp_mat_for_xhatminus_Update_data);
    Matrix_Init((&kf->temp_mat_for_xhatminus_Update2),x_size,1,kf->temp_mat_for_xhatminus_Update_data2);
    Matrix_Init((&kf->temp_mat_for_Pminus_Update),x_size,x_size,kf->temp_mat_for_Pminus_Update_data);
    Matrix_Init((&kf->temp_mat_for_Pminus_Update2),x_size,x_size,kf->temp_mat_for_Pminus_Update_data2);
    Matrix_Init(&(kf->temp_z_x_mat),z_size,x_size,kf->temp_z_x_mat_data);
    Matrix_Init(&(kf->temp_x_z_mat),x_size,z_size,kf->temp_x_z_mat_data);
    Matrix_Init(&(kf->temp_z_z_mat),z_size,z_size,kf->temp_z_z_mat_data);
    Matrix_Init(&(kf->temp_z_z_mat2),z_size,z_size,kf->temp_z_z_mat2_data);
    Matrix_Init(&(kf->temp_z_z_mat3),z_size,z_size,kf->temp_z_z_mat3_data);
    Matrix_Init(&(kf->temp_z_1_mat),z_size,1,kf->temp_z_1_mat_data);
    Matrix_Init(&(kf->temp_z_1_mat2),z_size,1,kf->temp_z_1_mat2_data);
    Matrix_Init(&kf->temp_x_x_mat,x_size,x_size,kf->temp_x_x_mat_data);
    Matrix_Init(&kf->temp_x_x_mat2,x_size,x_size,kf->temp_x_x_mat2_data);
    Matrix_Init(&(kf->temp_x_1_mat),x_size,1,kf->temp_x_1_mat_data);
}
/**
 * @brief 更新卡尔曼滤波器中的x先验估计
*/
void KalmanFilter_xhatminus_Update(KalmanFilter* kf)
{
    if(kf->u_size==0)
    {
        Matrix_Multiply(&(kf->A),&(kf->x_hat),&(kf->x_hat_minus));
    }
    else
    {
        Matrix_Multiply(&(kf->A),&(kf->x_hat),&(kf->temp_mat_for_xhatminus_Update));
        Matrix_Multiply(&(kf->B),&(kf->u),&(kf->temp_mat_for_xhatminus_Update2));
        Matrix_Add(&(kf->temp_mat_for_xhatminus_Update2),&(kf->temp_mat_for_xhatminus_Update),&(kf->x_hat_minus));
    }
}
/**
 * @brief 更新先验估计协方差矩阵
*/
void KalmanFilter_Pminus_Update(KalmanFilter* kf)
{
    Matrix_Multiply(&(kf->A),&(kf->P),&(kf->temp_mat_for_Pminus_Update));
    Matrix_Transpose(&(kf->A),&(kf->A_T));
    Matrix_Multiply(&(kf->temp_mat_for_Pminus_Update),&(kf->A_T),&(kf->temp_mat_for_Pminus_Update2));
    Matrix_Add(&(kf->temp_mat_for_Pminus_Update2),&(kf->Q),&(kf->P_minus));
}
/**
 * @brief 计算卡尔曼增益
*/
float debug_inv;
void KalmanFilter_SetK(KalmanFilter *kf)
{
    Matrix_Transpose(&(kf->H),&(kf->H_T));
    Matrix_Multiply(&(kf->P_minus),&(kf->H_T),&(kf->temp_x_z_mat));
    Matrix_Multiply(&(kf->H),&(kf->P_minus),&(kf->temp_z_x_mat));
    Matrix_Multiply(&(kf->temp_z_x_mat),&(kf->H_T),&(kf->temp_z_z_mat));
    Matrix_Add(&(kf->temp_z_z_mat),&(kf->R),&(kf->temp_z_z_mat2));
    Matrix_Inverse(&(kf->temp_z_z_mat2),&(kf->temp_z_z_mat3));//矩阵求逆有问题
    Matrix_Multiply(&(kf->temp_x_z_mat),&(kf->temp_z_z_mat3),&(kf->K));
}
/**
 * @brief 后验，得到最终估计值
*/
void KalmanFilter_xhat_Update(KalmanFilter *kf)
{
    Matrix_Multiply(&(kf->H),&(kf->x_hat_minus),&(kf->temp_z_1_mat));
    Matrix_Subtract(&(kf->z),&(kf->temp_z_1_mat),&(kf->temp_z_1_mat2));
    Matrix_Multiply(&(kf->K),&(kf->temp_z_1_mat2),&(kf->temp_x_1_mat));
    Matrix_Add(&(kf->temp_x_1_mat),&(kf->x_hat_minus),&(kf->x_hat));
}
/**
 * @brief 更新误差协方差矩阵
*/
void KalmanFilter_P_Update(KalmanFilter *kf)
{
    Matrix_Multiply(&(kf->K),&(kf->H),&(kf->temp_x_x_mat));
    Matrix_Subtract(&(kf->I_x_x),&(kf->temp_x_x_mat),&(kf->temp_x_x_mat2));
    Matrix_Multiply(&(kf->temp_x_x_mat2),&(kf->P_minus),&(kf->P));
}
/**
 * @brief kalman filter 迭代得到最优估计数据
*/
float *KalmanFilter_Update(KalmanFilter *kf)
{
    //更新观测值,输入值(如果有)
    memcpy(kf->z_data,kf->MeasuredVector_z,(kf->z_size)*sizeof_float);
    if(kf->u_size!=0)
    {
        memcpy(kf->u_data,kf->InputVector_u,(kf->u_size)*sizeof_float);
    }
    //得到先验估计值
    if(kf->User_func_before_xhatminusUpdate!=NULL)
    {
        kf->User_func_before_xhatminusUpdate(kf);
    }
    if(kf->xhatminusUpdate_skip)
    {
        return kf->Filtered_StateVector_x;
    }
    KalmanFilter_xhatminus_Update(kf);
/********************************************************************************************/   
    //得到先验估计协方差矩阵
    if(kf->User_func_before_PminusUpdate!=NULL)
    {
        kf->User_func_before_PminusUpdate(kf);
    }
    if(kf->PminusUpdate_skip)
    {
        return kf->Filtered_StateVector_x;
    }
    KalmanFilter_Pminus_Update(kf);
/**********************************************************************************************/
    //得到卡尔曼增益
    if(kf->User_func_before_SetK!=NULL)
    {
        kf->User_func_before_SetK(kf);
    }
    if(kf->SetK_skip)
    {
        memcpy(kf->x_hat_data,kf->x_hat_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->P_data,kf->P_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->Filtered_StateVector_x,kf->x_hat_data,(kf->x_size)*sizeof_float);
        return kf->Filtered_StateVector_x;
    }
    KalmanFilter_SetK(kf);
/****************************************************************************************************/
    //得到后验(最终)估计值
    if(kf->User_func_before_xhat_Update!=NULL)
    {
        kf->User_func_before_xhat_Update(kf);
    }
    if(kf->xhat_Update_skip)
    {
        memcpy(kf->x_hat_data,kf->x_hat_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->P_data,kf->P_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->Filtered_StateVector_x,kf->x_hat_data,(kf->x_size)*sizeof_float);
        return kf->Filtered_StateVector_x;
    }
    KalmanFilter_xhat_Update(kf);
/********************************************************************************************************/   
    //得到估计协方差矩阵
    if(kf->User_func_before_P_Update!=NULL)
    {
        kf->User_func_before_P_Update(kf);
    }
    if(kf->P_Update_skip)
    {
        memcpy(kf->x_hat_data,kf->x_hat_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->P_data,kf->P_minus_data,(kf->x_size)*sizeof_float);
        memcpy(kf->Filtered_StateVector_x,kf->x_hat_data,(kf->x_size)*sizeof_float);
        return kf->Filtered_StateVector_x;
    }
    KalmanFilter_P_Update(kf);
/*********************************************************************************************************/ 
    memcpy(kf->Filtered_StateVector_x,kf->x_hat_data,(kf->x_size)*sizeof_float);
    return kf->Filtered_StateVector_x;
}
