#include "software_i2c.h"
#include "gpio.h"
#include "bsp_dwt.h"

#define  I2C_GPIO_RCC      RCC_APB2Periph_GPIOB
#define  I2C_GPIO          GPIOB
#define  I2C_SCL           GPIO_PIN_7
#define  I2C_SDA           GPIO_PIN_6

/**
 * 函数功能：I2C时钟线写
 * 入口参数：BitVal：控制引脚0或1
 * 返 回 值：无
 */
void I2C_W_SCL(uint8_t BitVal)
{
   HAL_GPIO_WritePin(I2C_GPIO, I2C_SCL, BitVal);
   DWT_Delay(0.00001);
}

/**
 * 函数功能：I2C数据线写
 * 入口参数：BitVal：控制引脚0或1
 * 返 回 值：无
 */
void I2C_W_SDA(uint8_t BitVal)
{
   HAL_GPIO_WritePin(I2C_GPIO, I2C_SDA, BitVal);
   DWT_Delay(0.00001);
}

/**
 * 函数功能：I2C数据线读
 * 入口参数：无
 * 返 回 值：读取数据线引脚的状态
 */
uint8_t I2C_R_SDA(void)
{
   uint8_t i;
   i = HAL_GPIO_ReadPin(I2C_GPIO, I2C_SDA);
   DWT_Delay(0.00001);
   return i;
}


/**
*** 函数功能：软件I2C初始化
*** 入口参数：无
*** 返 回 值：无
***/
void I2C_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};

 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOH_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();

 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

 /*Configure GPIO pins : PB3 PB4 */
 GPIO_InitStruct.Pin = I2C_SCL|I2C_SDA;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    /*初始化引脚*/
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;      //开漏输出模式
//    GPIO_InitStructure.GPIO_Pin = I2C_SCL | I2C_SDA;  //GPIOB_Pin_3 | GPIOB_Pin_4
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(I2C_GPIO, &GPIO_InitStructure); 
   I2C_W_SCL(1);   //时钟线默认高电平
   I2C_W_SDA(1);   //数据线默认高电平
}

/**
 * 函数功能：I2C起始
 * 入口参数：无
 * 返 回 值：无
 */
void I2C_Start(void)
{  
   I2C_W_SDA(1);    //释放数据线
   I2C_W_SCL(1);    //释放时钟线
   I2C_W_SDA(0);    //下拉数据线
   I2C_W_SCL(0);    //下拉时钟线
}    
/**
 * 函数功能：I2C终止
 * 入口参数：无
 * 返 回 值：无
 */
void I2C_Stop(void)
{
   I2C_W_SDA(0);    //下拉数据线
   I2C_W_SCL(1);    //释放时钟线
   I2C_W_SDA(1);    //释放数据线
}
/**
 * 函数功能：I2C发送一个字节
 * 入口参数：Byte: 要发送的一个字节数据
 * 返 回 值：无
 */
void I2C_SendByte(uint8_t Byte)
{
   uint8_t i;
   
   for (i = 0; i < 8; i++)             //循环发送8次，完成一个字节的发送
   {
       I2C_W_SDA(Byte & (0x80 >> i));//从高位到地位一次发送
       I2C_W_SCL(1);                 //释放时钟线
       I2C_W_SCL(0);                 //下拉时钟线
   }
}
/**
 * 函数功能：I2C接收一个字节
 * 入口参数：无
 * 返 回 值：接收到的字节数据
 */
uint8_t I2C_ReceiveByte(void)
{
   uint8_t i;
   uint8_t Byte = 0x00;        //定义接收的数据
   
   I2C_W_SDA(1);             //释放数据线
   for (i = 0; i < 8; i++)     //重复接收8次， 完成接收一个字节
   {
       I2C_W_SCL(1);         //释放时钟线
       if (I2C_R_SDA() == 1) //读取数据线的状态，1就保存数据， 0就不作处理
       {
           Byte |= (0x80 >> i); 
       }
       I2C_W_SCL(0);          //下拉时钟线
   }
   
   return Byte;                 //返回接收到的数据
}
/**
 * 函数功能：I2C发送应答
 * 入口参数：AckBit：要发送的应答。0表示应答，1表示非应答。
 * 返 回 值：无
 */
void I2C_SendAck(uint8_t AckBit)
{
   I2C_W_SDA(AckBit);
   I2C_W_SCL(1);          //释放时钟线
   I2C_W_SCL(0);          //下拉时钟线
}

/**
 * 函数功能：I2C接收应答位
 * 入口参数：无
 * 返 回 值：接收到的应答位。0表示应答，1表示非应答。
 */
uint8_t I2C_ReceiveAck(void)
{
   uint8_t AckBit;             //定义应答位变量
   I2C_W_SDA(1);             //释放数据线
   I2C_W_SCL(1);             //释放时钟线
   AckBit = I2C_R_SDA();     //接收应答
   I2C_W_SCL(0);             //下拉时钟线
   
   return AckBit;              //返回应答
}
