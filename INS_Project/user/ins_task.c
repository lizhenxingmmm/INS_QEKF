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
float gyro_rad[3];
uint8_t who_you_are;
INS_T ins_data;
QEKF_IMU_t  qekf_imu;
uint32_t time_line;

void StartInsTask(void const * argument)
{
  DWT_Init(100);
  MPU6050_Init();
  who_you_are=MPU6050_GetID();
QEKF_IMU_Init(&qekf_imu,10,1,0);
  for(;;)
  {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
       MPU6050_GetAccel_Value(accel);
       MPU6050_GetGyro_Value(gyro);
       double t=DWT_GetDeltaT64(&time_line);
       for(int i=0;i<3;i++)
       {
        gyro_rad[i]=gyro[i]*(2*PI/360);
       }
       QEKF_IMU_Update(&qekf_imu,gyro_rad,accel,t);
       ins_data.Yaw=qekf_imu.Yaw;
       ins_data.Pitch=qekf_imu.Pitch;
       ins_data.Roll=qekf_imu.Roll;
       osDelay(1);
  }
}
