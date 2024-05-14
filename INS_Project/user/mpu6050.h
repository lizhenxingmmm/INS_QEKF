#ifndef MPU_6050_H
#define MPU_6050_H


#ifndef __MPU6050_H
#define __MPU6050_H
 
typedef unsigned char           uint8_t;
typedef short                   int16_t;

#define MPU6050_ADDRESS		    0xD0     //MPU6050的I2C从机地址
 
#define	MPU6050_SMPLRT_DIV		0x19     //采样频率分频器
#define	MPU6050_CONFIG			0x1A     //配置
#define	MPU6050_GYRO_CONFIG		0x1B     //陀螺仪配置
#define	MPU6050_ACCEL_CONFIG	0x1C     //加速度计配置
 
#define	MPU6050_ACCEL_XOUT_H	0x3B     //加速度计X轴高位
#define	MPU6050_ACCEL_XOUT_L	0x3C     //加速度计X轴底位
#define	MPU6050_ACCEL_YOUT_H	0x3D     //加速度计Y轴高位
#define	MPU6050_ACCEL_YOUT_L	0x3E     //加速度计Y轴底位
#define	MPU6050_ACCEL_ZOUT_H	0x3F     //加速度计Z轴高位
#define	MPU6050_ACCEL_ZOUT_L	0x40     //加速度计Z轴底位
#define	MPU6050_TEMP_OUT_H		0x41     //温度测量值高位     
#define	MPU6050_TEMP_OUT_L		0x42     //温度测量值低位
#define	MPU6050_GYRO_XOUT_H		0x43     //陀螺仪X轴高位
#define	MPU6050_GYRO_XOUT_L		0x44     //陀螺仪X轴低位
#define	MPU6050_GYRO_YOUT_H		0x45     //陀螺仪Y轴高位
#define	MPU6050_GYRO_YOUT_L		0x46     //陀螺仪Y轴低位
#define	MPU6050_GYRO_ZOUT_H		0x47     //陀螺仪Z轴高位
#define	MPU6050_GYRO_ZOUT_L		0x48     //陀螺仪Z轴低位
 
#define	MPU6050_PWR_MGMT_1		0x6B     //电源管理1
#define	MPU6050_PWR_MGMT_2		0x6C     //电源管理2
#define	MPU6050_WHO_AM_I		0x75     //设备ID
 
 
 
void MPU6050_Init(void);                       //初始化
uint8_t MPU6050_GetID(void);                   //读取ID
 
void MPU6050_GetACCEL(int16_t *ACCEL_Array);   //读取加速度计原始数据
void MPU6050_GetGYRO(int16_t *GYRO_Array);     //读取陀螺仪原始数据
 
void MPU6050_GetAccel_Value(float *Accel_Value);  //读取转换后的加速度计数值
void MPU6050_GetGyro_Value(float *Gyro_Value);    //读取转换后的陀螺仪数值
 
 
#endif

#endif
