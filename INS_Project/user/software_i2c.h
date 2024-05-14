#ifndef __soft_I2C_H
#define __soft_I2C_H
 
typedef unsigned char           uint8_t;
typedef short                   int16_t;
 
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t Byte);
uint8_t I2C_ReceiveByte(void);
void I2C_SendAck(uint8_t AckBit);
uint8_t I2C_ReceiveAck(void);

#endif
