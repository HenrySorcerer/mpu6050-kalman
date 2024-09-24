#include "stm32f1xx_hal.h"
#include "mpu6050.h"
#include "IIC.h"

#define MPU6050_ADDRESS 0xD0 // �ӻ�д��ַ 11010000

// ָ����ַд
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();                   // 1.������ʼλ
    IIC_SendByte(MPU6050_ADDRESS); // 2.����Ѱ�� + д
    IIC_ReceiveAck();              // 3.����Ӧ��λ-�����Ȳ�������
    IIC_SendByte(RegAddress);      // 4.������Ҫд�ĵ�ַ
    IIC_ReceiveAck();              // 5.����Ӧ��λ-�����Ȳ�������
    IIC_SendByte(Data);            // 6.������Ҫд������
    IIC_ReceiveAck();              // 7.����Ӧ��λ-�����Ȳ�������
    IIC_Stop();                    // 8.������ֹ����
}

// ָ����ַ��
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    // 1.������ʼλ
    IIC_Start();
    // 2.Ѱַ + д
    IIC_SendByte(MPU6050_ADDRESS);
    IIC_ReceiveAck();
    // 3.������д�ĵ�ַ
    IIC_SendByte(RegAddress);
    IIC_ReceiveAck();
    // �ظ���ʼλ
    IIC_Start();
    // Ѱַ + ��
    IIC_SendByte(MPU6050_ADDRESS | 0x01); // �����1λ�ĳ�1��������� 1101 0001
    IIC_ReceiveAck();
    Data = IIC_ReceiveByte();
    IIC_SendAck(1); // ������Ӧλ��Ӧ�𣬴ӻ��յ���Ͳ������������
    // ֹͣ
    IIC_Stop();
    return Data;
}

void MPU6050_Init(void)
{
    IIC_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // ���˯��ģʽ������ʹ��X�������ǵ�ʱ��
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // �����᲻��Ҫ����

    //MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   // ����Ϊ10��Ƶ

    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00);

    //MPU6050_WriteReg(MPU6050_CONFIG, 0x06);       // ���õ�ͨ�˲�ֵ

    MPU6050_WriteReg(MPU6050_CONFIG, 0x03); // 41Hz �����ʺ�ƽ��Ӧ��
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);  // ���� FS_SEL = 00������ ��250��/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);  // ���� AFS_SEL = 00������ ��2g

    //MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  // ��������������Ϊ�������
    //MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // ���ٶȼ���������Ϊ�������
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//����WHO_AM_I�Ĵ�����ֵ
}

/**
  * ��    ����MPU6050��ȡ����
  * ��    ����AccX AccY AccZ ���ٶȼ�X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * ��    ����GyroX GyroY GyroZ ������X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * �� �� ֵ����
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
	*AccX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//��ȡ���ٶȼ�Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//��ȡ���ٶȼ�Y��ĵ�8λ����
	*AccY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//��ȡ���ٶȼ�Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//��ȡ���ٶȼ�Z��ĵ�8λ����
	*AccZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//��ȡ������X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//��ȡ������X��ĵ�8λ����
	*GyroX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//��ȡ������Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//��ȡ������Y��ĵ�8λ����
	*GyroY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
	*GyroZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
}

int16_t MPU6050_GetTemp(void)
{
    uint8_t DataH, DataL;								
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);	
    int16_t temp = (int16_t)(DataH << 8 | DataL);
    return temp;
}
