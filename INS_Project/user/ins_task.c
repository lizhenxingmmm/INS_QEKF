#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "ins_task_h"
#include "KalmanFilter.h"
#include "QEKF_IMU.h"
#include "software_i2c.h"
#include "mpu6050.h"
#include "bsp_dwt.h"

float accel[3];
float gyro[3];
float gyro_rad[3];//roll,pitch,yaw
float accel_norm[3];
uint8_t who_you_are;
INS_T ins_data;
QEKF_IMU_t  qekf_imu;
uint32_t time_line;

void StartInsTask(void const * argument)
{
 DWT_Init(100);
 MPU6050_Init();
 who_you_are=MPU6050_GetID();
QEKF_IMU_Init(&qekf_imu,1000,1,0);
 for(;;)
 {
       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
      MPU6050_GetAccel_Value(accel);
      MPU6050_GetGyro_Value(gyro);
      double t=DWT_GetDeltaT64(&time_line);
      //角速度化为rad/s
      for(int i=0;i<3;i++)
      {
       gyro_rad[i]=gyro[i]*(2*PI/360);
      }
      //加速度单位化
      float scale_accel=1/sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
      for(int i=0;i<3;i++)
      {
        accel_norm[i]=accel[i]*scale_accel;
      }
      QEKF_IMU_Update(&qekf_imu,gyro_rad,accel_norm,t);
      ins_data.Yaw=qekf_imu.Yaw;
      ins_data.Pitch=qekf_imu.Pitch;
      ins_data.Roll=qekf_imu.Roll;
      osDelay(1);
 }
}
