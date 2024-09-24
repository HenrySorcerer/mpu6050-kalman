#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "IIC.h"

#define MPU6050_ADDRESS 0xD0 // 从机写地址 11010000

// 指定地址写
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();                   // 1.发送起始位
    IIC_SendByte(MPU6050_ADDRESS); // 2.发送寻找 + 写
    IIC_ReceiveAck();              // 3.接收应答位-这里先不做处理
    IIC_SendByte(RegAddress);      // 4.发送需要写的地址
    IIC_ReceiveAck();              // 5.接收应答位-这里先不做处理
    IIC_SendByte(Data);            // 6.发送需要写的数据
    IIC_ReceiveAck();              // 7.接收应答位-这里先不做处理
    IIC_Stop();                    // 8.发送终止条件
}

// 指定地址读
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    // 1.发送起始位
    IIC_Start();
    // 2.寻址 + 写
    IIC_SendByte(MPU6050_ADDRESS);
    IIC_ReceiveAck();
    // 3.发送需写的地址
    IIC_SendByte(RegAddress);
    IIC_ReceiveAck();
    // 重复起始位
    IIC_Start();
    // 寻址 + 读
    IIC_SendByte(MPU6050_ADDRESS | 0x01); // 把最后1位改成1，即变成了 1101 0001
    IIC_ReceiveAck();
    Data = IIC_ReceiveByte();
    IIC_SendAck(1); // 发送相应位不应答，从机收到后就不会继续发数据
    // 停止
    IIC_Stop();
    return Data;
}

void MPU6050_Init(void)
{
    IIC_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // 解除睡眠模式，并且使用X轴陀螺仪的时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // 所有轴不需要待机

    //MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   // 设置为10分频

    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00);

    //MPU6050_WriteReg(MPU6050_CONFIG, 0x06);       // 设置低通滤波值

    MPU6050_WriteReg(MPU6050_CONFIG, 0x03); // 41Hz 带宽，适合平衡应用
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);  // 设置 FS_SEL = 00，量程 ±250°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);  // 设置 AFS_SEL = 00，量程 ±2g

    //MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  // 陀螺仪设置量程为最大量程
    //MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 加速度计设置量程为最大量程
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}

int16_t MPU6050_GetTemp(void)
{
    uint8_t DataH, DataL;								
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);	
    int16_t temp = (int16_t)(DataH << 8 | DataL);
    return temp;
}
