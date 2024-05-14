#include "MPU6050.h"
#include "software_i2c.h"

/**
 * 函数功能：MPU6050写1个寄存器
 * 入口参数：Reg： 需要操作的MPU6050地址
 * 入口参数：Data：需要写入的数据
 * 返 回 值：无
 */
void MPU6050_WriteReg(uint8_t Reg, uint8_t Data)
{
   /*发送从机地址*/
   I2C_Start();                    //I2C起始
   I2C_SendByte(MPU6050_ADDRESS);  //发送MPU6050地址 0xD0  第0位为0是写操作，1是读操作
   I2C_ReceiveAck();               //接收应答，我这里只接收，不作处理。
   
   /*发送需要写操作的寄存器*/
   I2C_SendByte(Reg);              
   I2C_ReceiveAck();
   
   /*发送需要写入的数据*/
   I2C_SendByte(Data);
   I2C_ReceiveAck();
   
   I2C_Stop();                      //I2C终止
}

/**
 * 函数功能：MPU6050写多个字节
 * 入口参数：BufAddress：寄存器首地址
 * 入口参数：BufData   ：数据首地址
 * 入口参数：Length    ：写入的字节数量
 * 返 回 值：无
 */
void MPU6050_WriteBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
   uint8_t i;                               //for循环次数变量
   
   I2C_Start();                           //起始
   I2C_SendByte(MPU6050_ADDRESS | 0x00);  //发送从机地址  | 写模式
   I2C_ReceiveAck();                      //接收应答
   I2C_SendByte(BufAddress);              //发送寄存器首地址
   I2C_ReceiveAck();                      //接收应答
   
   for (i = 0; i < Length; i++) 
   {
       I2C_SendByte(BufData[i]);          //连续写入数据
       I2C_ReceiveAck();                  //接收应答
   }
   I2C_Stop();                            //停止
}

/**
 * 函数功能：读取MPU6050寄存器里的数据
 * 入口参数：Reg：需要读取的寄存器地址
 * 返 回 值：读取到的数据
 */
uint8_t MPU6050_ReadReg(uint8_t Reg)
{
   uint8_t Reg_Data;                      //定义保存读取到的数据变量
   
   /*发送从机地址*/
   I2C_Start();                         //I2C起始
   I2C_SendByte(MPU6050_ADDRESS);       //MPU6050默认地址：0xD0 
   I2C_ReceiveAck();                    //接收MPU6050应答位
   
   /*发送需要读取的寄存器地址*/
   I2C_SendByte(Reg);                   
   I2C_ReceiveAck();
   
   /*发送读取模式*/
   I2C_Start();                           //I2C起始
   I2C_SendByte(MPU6050_ADDRESS | 0x01);  //发送MPU6050地址0xD0 | 0x01 (第0位为0是写操作，1是读操作)
   I2C_ReceiveAck();
   
   /*开始读取从机数据*/
   Reg_Data = I2C_ReceiveByte();          //读取MPU6050的数据
   I2C_SendAck(1);                        //读取完成后，应答位给1（非应答）
   I2C_Stop();                            //I2C终止
   
   return Reg_Data;                         //返回读取到的数据
}

/**
 * 函数功能：连续读MPU6050寄存器
 * 入口参数：BufAddress：寄存器首地址
 * 入口参数：BufData   ：接收数据首地址
 * 入口参数：Length    ：读取寄存器的数量
 */
void MPU6050_ReadBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
   uint8_t i;                              //for循环次数变量
   
   I2C_Start();                          //起始
   I2C_SendByte(MPU6050_ADDRESS | 0x00); //发送从机地址 | 写模式
   I2C_ReceiveAck();                     //接收应答
   I2C_SendByte(BufAddress);             //发送寄存器地址
   I2C_ReceiveAck();                     //接收应答
   
   I2C_Start();                          //起始
   I2C_SendByte(MPU6050_ADDRESS | 0x01); //发送从机地址 | 读模式
   I2C_ReceiveAck();                     //接收应答
   
   /*连续读取数据*/
   for (i = 0; i < Length; i++) 
   {
       BufData[i] = I2C_ReceiveByte();   //把数据存放在i指向的变量
       if( i < (Length - 1) )              
       {
           I2C_SendAck(0);               //发送应答
       }
       else                                
       {
           I2C_SendAck(1);               //最后一个要发送非应答
       }
   }
   I2C_Stop();                           //停止
}

/**
 * 函数功能：将2个字节转换成1个有符号半字，用来处理寄存器的高低位数据。
 * 入口参数：DataL：低位字节
 * 入口参数：DataH: 高位字节
 * 返 回 值：转换完成后的有符号数据
 */
int16_t MPU6050_Byte_to_HalfWord(uint8_t DataL, uint8_t DataH)
{
   int16_t Data;
   
   Data = (DataH << 8) | DataL;  //DataL放到低8位， DataH放到高8位
   
   return Data;                                //返回转换后的数据
}

/**
 * 函数功能：读取MPU6050设备ID
 * 入口参数：无
 * 返 回 值：设备ID
 */
uint8_t MPU6050_GetID(void)
{
   uint8_t ID = 0;
   ID = MPU6050_ReadReg(MPU6050_WHO_AM_I);  //读取设备ID   寄存器：0x75
   
   return ID;
}

/**
 * 函数功能：读取加速度计原始数值
 * 入口参数：ACCEL_Array: X轴保存到外部的第0个元素
                          Y轴保存到外部的第1个元素
                          Z轴保存到外部的第2个元素
 * 返 回 值：无
 */
void MPU6050_GetACCEL(int16_t *ACCEL_Array)
{
   uint8_t DataL;     //定义保存数据低位变量
   uint8_t DataH;     //定义保存数据高位变量

   /*读取加速度计原始数值，寄存器地址含义需要翻手册的描述*/
   DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);           //读取X轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);           //读取X轴高位数据到DataH
   ACCEL_Array[0] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将X轴转换结果存放到ACCEL_Array[0]
   
   DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);           //读取Y轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);           //读取Y轴高位数据到DataH
   ACCEL_Array[1] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将Y轴转换结果存放到ACCEL_Array[1]
   
   DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);           //读取Z轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);           //读取Z轴高位数据到DataH
   ACCEL_Array[2] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将Z轴转换结果存放到ACCEL_Array[2]
}

/**
 * 函数功能: 读取陀螺仪原始数值
 * 入口参数：ACCEL_Array: X轴保存到第0个元素
                          Z轴保存到第2个元素
                          Y轴保存到第1个元素
 * 返 回 值：无           
 */
void MPU6050_GetGYRO(int16_t *GYRO_Array)
{
   uint8_t DataL;     //定义数据低位变量
   uint8_t DataH;     //定义数据高位变量

   DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);           //读取X轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);           //读取X轴高位数据到DataH
   GYRO_Array[0] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将X轴转换结果存放到GYRO_Array[0]
   
   DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);           //读取Y轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);           //读取Y轴高位数据到DataH
   GYRO_Array[1] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将Y轴转换结果存放到GYRO_Array[1]
   
   DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);           //读取Z轴低位数据到DataL
   DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);           //读取Z轴高位数据到DataH
   GYRO_Array[2] = MPU6050_Byte_to_HalfWord(DataL, DataH); //调用转换函数，将Z轴转换结果存放到GYRO_Array[2]
}
/**
 * 函数功能：线性标度变换
 * 入口参数：Sample_Value: 采样回来的原始数值 
 * 入口参数：URV：         量程上限      
 * 入口参数：LRV：         量程下限
 * 返 回 值：变换后的数据
 */
float Scale_Transform(float Sample_Value, float URV, float LRV)
{
   float Data;             //定义用来保存变换后的数据变量
   float Value_L = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
   float Value_U = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
   
   /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
   Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
          
   return Data;
}
/**
 * 函数功能：读取转换后加速度计数值
 * 入口参数：Accel_Value：加速度值转换后保存的变量
 * 返 回 值：无
 */
void MPU6050_GetAccel_Value(float *Accel_Value)
{
   int16_t ACCEL_Array[3];          //定义用来保存加速度计原始数值的变量
   
   /*读取加速度计原始数值*/
   MPU6050_GetACCEL(ACCEL_Array);   
   
   /*开始转换加速度值*/
   Accel_Value[0] = Scale_Transform( (float)ACCEL_Array[0], 16.0, -16.0);  //转换X轴
   Accel_Value[1] = Scale_Transform( (float)ACCEL_Array[1], 16.0, -16.0);  //转换Y轴
   Accel_Value[2] = Scale_Transform( (float)ACCEL_Array[2], 16.0, -16.0);  //转换Z轴
}
/**
 * 函数功能：读取转换后的陀螺仪数值
 * 入口参数：Gyro_Value：陀螺仪转换后保存的变量
 * 返 回 值：无
 */
void MPU6050_GetGyro_Value(float *Gyro_Value)
{
   /*定义用来保存陀螺仪原始数值的数组*/
   int16_t GYRO_Array[3];        
  
   /*读取陀螺仪的原始数值，保存到GYRO_Array[]数组中*/
   MPU6050_GetGYRO(GYRO_Array);  
   
   /*开始转换陀螺仪值*/
   Gyro_Value[0] = Scale_Transform( (float)GYRO_Array[0], 2000.0, -2000.0);  //转换X轴
   Gyro_Value[1] = Scale_Transform( (float)GYRO_Array[1], 2000.0, -2000.0);  //转换Y轴
   Gyro_Value[2] = Scale_Transform( (float)GYRO_Array[2], 2000.0, -2000.0);  //转换Z轴
   
}
/**
 * 函数功能：初始化MPU6050
 * 入口参数：无
 * 返 回 值：无
 */
void MPU6050_Init(void)
{
   /*调用软件I2C初始化函数*/
   I2C_Init();
   
   /*MPU6050寄存器初始化, 需要对照手册描述来配置*/
   MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   //电源管理1寄存器，取消睡眠模式，选择时钟源为X轴陀螺仪
   MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   //电源管理2寄存器，保持默认值，所有轴不休眠。
   MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   //采样率分频寄存器，
   MPU6050_WriteReg(MPU6050_CONFIG, 0x06);       //配置寄存器
   MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  //陀螺仪配置寄存器，选择满量程 ±2000°/s
   MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); //加速度计配置寄存器，选择满量程 ±16g
   
}

