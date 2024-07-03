/**
 * @Author: Li Zhenxing
 * @Date: 2024/5/13 08:34:59
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/5/13 08:34:59
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#include "QEKF_IMU.h"

float Chi_square_result=0;
uint32_t Chi_square_skip_count=0;

static float x_init[6]={1,0,0,0,-0.9155f*2*PI/360,0.529f*2*PI/360};//初始化姿态
static float A_mat_data_init[36]={1,0,0,0,0,0,
                                  0,1,0,0,0,0,
                                  0,0,1,0,0,0,
                                  0,0,0,1,0,0,
                                  0,0,0,0,1,0,
                                  0,0,0,0,0,1};
static float P_mat_data_init[36]={10000,0.1,0.1,0.1,0.1,0.1,
                                  0.1,10000,0.1,0.1,0.1,0.1,
                                  0.1,0.1,10000,0.1,0.1,0.1,
                                  0.1,0.1,0.1,10000,0.1,0.1,
                                  0.1,0.1,0.1,0.1,100,0.1,
                                  0.1,0.1,0.1,0.1,0.1,100};                                  
/**
 * @brief init
 * @note    设置4元数计算噪声，陀螺仪零漂噪声，测量噪声协方差矩阵的对角元素
 * @param   process_noisy1  计算噪声
 * @param   process_noisy2  陀螺仪零漂噪声
 * @param   measure_noisy   测量噪声
*/
void QEKF_IMU_Init(QEKF_IMU_t* qekf,float process_noisy1,float process_noisy2,float measure_noisy)
{
    memset(qekf->mat_q_data,0,16*sizeof_float);
    //卡尔曼滤波器初始化
    KalmanFilter_Init(&(qekf->IMU_QEKF_kf),6,0,3);
    //连接函数
    qekf->IMU_QEKF_kf.User_func_before_SetK=Chi_square_Check;//算卡尔曼增益前的卡方检测
    qekf->IMU_QEKF_kf.User_func_before_P_Update=eliminate_q3;//取消重力加速度对偏航角的修正
    //init kalman filter matrix
    memcpy(qekf->IMU_QEKF_kf.A_data,A_mat_data_init,36*sizeof_float);//初始化状态传递矩阵
    memcpy(qekf->IMU_QEKF_kf.P_data,P_mat_data_init,36*sizeof_float);//初始化误差协方差
    memcpy(qekf->IMU_QEKF_kf.x_hat_data,x_init,6*sizeof_float);//初始化姿态和xy轴零漂
    qekf->Q1=process_noisy1;
    qekf->Q2=process_noisy2;
    qekf->R=measure_noisy;
    qekf->initialized_flag=1;
}
/**
 * @brief imu kalman filter updata 卡尔曼迭代
 * @note  kalman filter 需要外部输入的数据有 A,H,B(如果有),Q,R,z
 * @note    状态变量[q0,q1,q2,q3,w_bias_x,w_bias_y]T
 *          状态转移矩阵A[mat_q,mat_Ok
 *                       0    ,I2     ]
 *          观测矩阵为H[-q2,q3,-q0,q1,0,0
 *                      q1,q0,q3,q2,0,0
 *                      q0,-q1,-q2,q3,0,0]*2
 * @param 输入的角速度rad/s ，重力加速度要做单位化处理
*/
void QEKF_IMU_Update(QEKF_IMU_t* qekf,float gyro[3],float accel[3],float dt)
{
    //gyro[0]==Wx,gyro[1]==Wy,gyro[2]==Wz
    //accel[0]==ax,accel[1]==ay,accel==az
    //防止程序未初始化
    if(!qekf->initialized_flag)
    {
        QEKF_IMU_Init(qekf,10,1,0);//噪声参数应该还要调
    }
    memcpy(qekf->accel,accel,3*sizeof_float);
    memcpy(qekf->gyro,gyro,3*sizeof_float);
    //得到两次程序的时间差
    qekf->dt=dt;
    //将加速度输入卡尔曼滤波器的观测值中
    memcpy(qekf->IMU_QEKF_kf.MeasuredVector_z,accel,qekf->IMU_QEKF_kf.z_size*sizeof_float);
    //迭代状态转移矩阵A
   QEKF_IMU_A_Update(qekf,gyro);
   //迭代观测矩阵H
   QEKF_IMU_H_Update(qekf);
   //计算噪声矩阵
    qekf->IMU_QEKF_kf.Q_data[0]=(qekf->Q1)*dt;
    qekf->IMU_QEKF_kf.Q_data[7]=(qekf->Q1)*dt;
    qekf->IMU_QEKF_kf.Q_data[14]=(qekf->Q1)*dt;
    qekf->IMU_QEKF_kf.Q_data[21]=(qekf->Q1)*dt;
    qekf->IMU_QEKF_kf.Q_data[28]=(qekf->Q2)*dt;
    qekf->IMU_QEKF_kf.Q_data[35]=(qekf->Q2)*dt;
    //计算测量矩阵
    qekf->IMU_QEKF_kf.R_data[0]=(qekf->R)*dt;
    qekf->IMU_QEKF_kf.R_data[4]=(qekf->R)*dt;
    qekf->IMU_QEKF_kf.R_data[8]=(qekf->R)*dt;
   //kalman filter update
   KalmanFilter_Update(&(qekf->IMU_QEKF_kf));
   //取数据
   memcpy(qekf->q,qekf->IMU_QEKF_kf.Filtered_StateVector_x,4*sizeof_float);
   memcpy(qekf->w_bias,&(qekf->IMU_QEKF_kf.Filtered_StateVector_x[4]),2*sizeof_float);
   // 利用四元数反解欧拉角
    qekf->Yaw = atan2f(2.0f * (qekf->q[0] * qekf->q[3] + qekf->q[1] * qekf->q[2]), 2.0f * (qekf->q[0] * qekf->q[0] + qekf->q[1] * qekf->q[1]) - 1.0f) * 57.295779513f;
    qekf->Pitch = atan2f(2.0f * (qekf->q[0] * qekf->q[1] + qekf->q[2] * qekf->q[3]), 2.0f * (qekf->q[0] * qekf->q[0] + qekf->q[3] * qekf->q[3]) - 1.0f) * 57.295779513f;
    qekf->Roll = asinf(-2.0f * (qekf->q[1] * qekf->q[3] - qekf->q[0] * qekf->q[2])) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (qekf->Yaw - qekf->YawAngleLast > 180.0f)
    {
        qekf->YawRoundCount--;
    }
    else if (qekf->Yaw - qekf->YawAngleLast < -180.0f)
    {
        qekf->YawRoundCount++;
    }
    qekf->YawTotalAngle = 360.0f * qekf->YawRoundCount + qekf->Yaw;
    qekf->YawAngleLast = qekf->Yaw;
    qekf->UpdateCount++; // 初始化低通滤波用,计数测试用
}
/**
 * @brief 迭代状态转移矩阵A
*/
void QEKF_IMU_A_Update(QEKF_IMU_t* qekf,float gyro[3])
{
    //迭代状态转移矩阵的左上角4x4矩阵
    //mat_q=0.5*omega*dt+I4
    /*omega=
    {0  -wx -wy -wz
    wx  0   wz -wy
    wy -wz  0   wx
    wz  wy -wx  0}*/
   float wx=gyro[0]-qekf->IMU_QEKF_kf.Filtered_StateVector_x[4];
   float wy=gyro[1]-qekf->IMU_QEKF_kf.Filtered_StateVector_x[5];
   float wz=gyro[2];
    qekf->mat_q_data[0]=1;
    qekf->mat_q_data[1]=-wx*0.5f*(qekf->dt);
    qekf->mat_q_data[2]=-wy*0.5f*(qekf->dt);
    qekf->mat_q_data[3]=-wz*0.5f*(qekf->dt);
    qekf->mat_q_data[4]=wx*0.5f*(qekf->dt);
    qekf->mat_q_data[5]=1;
    qekf->mat_q_data[6]=wz*0.5f*(qekf->dt);
    qekf->mat_q_data[7]=-wy*0.5f*(qekf->dt);
    qekf->mat_q_data[8]=wy*0.5f*(qekf->dt);
    qekf->mat_q_data[9]=-wz*0.5f*(qekf->dt);
    qekf->mat_q_data[10]=1;
    qekf->mat_q_data[11]=wx*0.5f*(qekf->dt);
    qekf->mat_q_data[12]=wz*0.5f*(qekf->dt);
    qekf->mat_q_data[13]=wy*0.5f*(qekf->dt);
    qekf->mat_q_data[14]=-wx*0.5f*(qekf->dt);
    qekf->mat_q_data[15]=1;
    //迭代状态转移矩阵的右上角4x2矩阵
    /*mat_Ok=
        {q1*dt*0.5   q2*dt*0.5
        -q0*dt*0.5   q3*dt*0.5 
        -q3*dt*0.5  -q0*dt*0.5
         q2*dt*0.5  -q1*dt*0.5}*/
    //单位化四元数
    float scale=1/sqrt((qekf->q[0])*(qekf->q[0])+(qekf->q[1])*(qekf->q[1])+(qekf->q[2])*(qekf->q[2])+(qekf->q[3])*(qekf->q[3]));
    qekf->q[0]*=scale;
    qekf->q[1]*=scale;
    qekf->q[2]*=scale;
    qekf->q[3]*=scale;//有必要？
    qekf->mat_Ok_data[0]=qekf->q[1]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[1]=qekf->q[2]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[2]=-qekf->q[0]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[3]=qekf->q[3]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[4]=-qekf->q[3]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[5]=-qekf->q[0]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[6]=qekf->q[2]*(qekf->dt)*0.5f;
    qekf->mat_Ok_data[7]=-qekf->q[1]*(qekf->dt)*0.5f;
    //更新kalman filter A_data
    qekf->IMU_QEKF_kf.A_data[0]=qekf->mat_q_data[0];
    qekf->IMU_QEKF_kf.A_data[1]=qekf->mat_q_data[1];
    qekf->IMU_QEKF_kf.A_data[2]=qekf->mat_q_data[2];
    qekf->IMU_QEKF_kf.A_data[3]=qekf->mat_q_data[3];
    qekf->IMU_QEKF_kf.A_data[4]=qekf->mat_Ok_data[0];
    qekf->IMU_QEKF_kf.A_data[5]=qekf->mat_Ok_data[1];
    qekf->IMU_QEKF_kf.A_data[6]=qekf->mat_q_data[4];
    qekf->IMU_QEKF_kf.A_data[7]=qekf->mat_q_data[5];
    qekf->IMU_QEKF_kf.A_data[8]=qekf->mat_q_data[6];
    qekf->IMU_QEKF_kf.A_data[9]=qekf->mat_q_data[7];
    qekf->IMU_QEKF_kf.A_data[10]=qekf->mat_Ok_data[2];
    qekf->IMU_QEKF_kf.A_data[11]=qekf->mat_Ok_data[3];
    qekf->IMU_QEKF_kf.A_data[12]=qekf->mat_q_data[8];
    qekf->IMU_QEKF_kf.A_data[13]=qekf->mat_q_data[9];
    qekf->IMU_QEKF_kf.A_data[14]=qekf->mat_q_data[10];
    qekf->IMU_QEKF_kf.A_data[15]=qekf->mat_q_data[11];
    qekf->IMU_QEKF_kf.A_data[16]=qekf->mat_Ok_data[4];
    qekf->IMU_QEKF_kf.A_data[17]=qekf->mat_Ok_data[5];
    qekf->IMU_QEKF_kf.A_data[18]=qekf->mat_q_data[12];
    qekf->IMU_QEKF_kf.A_data[19]=qekf->mat_q_data[13];
    qekf->IMU_QEKF_kf.A_data[20]=qekf->mat_q_data[14];
    qekf->IMU_QEKF_kf.A_data[21]=qekf->mat_q_data[15];
    qekf->IMU_QEKF_kf.A_data[22]=qekf->mat_Ok_data[6];
    qekf->IMU_QEKF_kf.A_data[23]=qekf->mat_Ok_data[7];
    //左下角为2x4的0矩阵，右下角为2x2单位矩阵
    memset(&(qekf->IMU_QEKF_kf.A_data[24]),0,4*sizeof_float);
    memset(&(qekf->IMU_QEKF_kf.A_data[30]),0,4*sizeof_float);
    qekf->IMU_QEKF_kf.A_data[28]=1;
    qekf->IMU_QEKF_kf.A_data[29]=0;
    qekf->IMU_QEKF_kf.A_data[35]=1;
    qekf->IMU_QEKF_kf.A_data[34]=0;
}
/**
 * @brief 迭代观测矩阵H
*/
void QEKF_IMU_H_Update(QEKF_IMU_t* qekf)
{
    qekf->IMU_QEKF_kf.H_data[0]=-2*(qekf->q[2]);
    qekf->IMU_QEKF_kf.H_data[1]=2*(qekf->q[3]);
    qekf->IMU_QEKF_kf.H_data[2]=-2*(qekf->q[0]);
    qekf->IMU_QEKF_kf.H_data[3]=2*(qekf->q[1]);
    qekf->IMU_QEKF_kf.H_data[4]=0;
    qekf->IMU_QEKF_kf.H_data[5]=0;
    qekf->IMU_QEKF_kf.H_data[6]=2*(qekf->q[1]);
    qekf->IMU_QEKF_kf.H_data[7]=2*(qekf->q[0]);
    qekf->IMU_QEKF_kf.H_data[8]=2*(qekf->q[3]);
    qekf->IMU_QEKF_kf.H_data[9]=2*(qekf->q[2]);
    qekf->IMU_QEKF_kf.H_data[10]=0;
    qekf->IMU_QEKF_kf.H_data[11]=0;
    qekf->IMU_QEKF_kf.H_data[12]=2*(qekf->q[0]);
    qekf->IMU_QEKF_kf.H_data[13]=-2*(qekf->q[1]);
    qekf->IMU_QEKF_kf.H_data[14]=-2*(qekf->q[2]);
    qekf->IMU_QEKF_kf.H_data[15]=2*(qekf->q[3]);
    qekf->IMU_QEKF_kf.H_data[16]=0;
    qekf->IMU_QEKF_kf.H_data[17]=0;
}
/**
 * @brief 卡方检测
 * @note    r=eT*D-1*e
*/
void Chi_square_Check(KalmanFilter* kf)
{
    Matrix_Multiply(&(kf->H),&(kf->x_hat_minus),&(kf->temp_z_1_mat));
    Matrix_Subtract(&(kf->z),&(kf->temp_z_1_mat),&(kf->temp_z_1_mat2));//temp_z_1_mat2 is e(观测误差)
    //算观测误差的协方差
    Matrix_Multiply(&(kf->H),&(kf->P_minus),&(kf->temp_z_x_mat));
    Matrix_Multiply(&(kf->temp_z_x_mat),&(kf->H_T),&(kf->temp_z_z_mat));
    Matrix_Add(&(kf->temp_z_z_mat),&(kf->R),&(kf->temp_z_z_mat2));//temp_z_z_mat2 is D(观测误差的协方差)
    //算卡方检测
    Matrix_Inverse(&(kf->temp_z_z_mat2),&(kf->temp_z_z_mat3));
    Matrix_Multiply(&(kf->temp_z_z_mat3),&(kf->temp_z_1_mat2),&(kf->temp_z_1_mat));
    arm_dot_prod_f32(kf->temp_z_1_mat2_data,kf->temp_z_1_mat_data,kf->z_size,&Chi_square_result);
    if(Chi_square_result>CHI_SQUARE_VALVE)
    {
        kf->SetK_skip=1;
        kf->P_Update_skip=1;
        kf->xhat_Update_skip=1;
        Chi_square_skip_count++;
        //滤波器发散
        if(Chi_square_skip_count>CHI_SQUARE_SKIP_COUNT_MAX)
        {
            kf->SetK_skip=0;
            kf->P_Update_skip=0;
            kf->xhat_Update_skip=0;
        }
    }
    else
    {
        kf->SetK_skip=0;
        kf->P_Update_skip=0;
        kf->xhat_Update_skip=0;
        Chi_square_skip_count=0;
    }
}
/**
 * @brief   重力加速度不能修正偏航角
 * 
*/
void eliminate_q3(KalmanFilter* kf)
{
    kf->I_data[21]=0;
    Matrix_Multiply(&(kf->I_x_x),&(kf->K),&(kf->temp_x_x_mat));
    memcpy(kf->K_data,kf->temp_x_x_mat_data,36*sizeof_float);
    kf->I_data[21]=1;
}
