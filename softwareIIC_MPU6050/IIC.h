#ifndef _IIC_H
#define _IIC_H

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(uint8_t byte);
uint8_t IIC_ReceiveByte(void);
void IIC_SendAck(uint8_t AckBit);
uint8_t IIC_ReceiveAck(void);

#endif
