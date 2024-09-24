#include "stm32f1xx_hal.h"
#include "IIC.h"
#include "tim.h"

// /* ����IIC�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define IIC_GPIO_PORT GPIOA     /* GPIO�˿� */
#define IIC_SCL_PIN GPIO_PIN_10 /* ���ӵ�SCLʱ���ߵ�GPIO */
#define IIC_SDA_PIN GPIO_PIN_11 /* ���ӵ�SDA�����ߵ�GPIO */

// ���ģ��IIC,GPIO��TIMER�ĳ�ʼ����cubemx����(gpio.c��tim.c)
void IIC_Init(void)
{
    HAL_TIM_Base_Start(&htim4); // ʹ�ܶ�ʱ�����ṩ����ʱ����
}

void IIC_Delay(uint8_t us) // ΢�뼶��ʱ
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim4); // ��ȡ��ǰ����ֵ
    while ((__HAL_TIM_GET_COUNTER(&htim4) - start) < us)
    {
        // ����ʱ����������
        if (__HAL_TIM_GET_COUNTER(&htim4) < start)
        {
            break; // ��������������ֱ���˳�ѭ��
        }
    }
}

void IIC_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, (GPIO_PinState)BitValue);
    IIC_Delay(10);
}

void IIC_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, (GPIO_PinState)BitValue);
    IIC_Delay(10);
}

uint8_t IIC_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN);
    IIC_Delay(10);
    return BitValue;
}

// ��ʼ�ź�
void IIC_Start(void)
{
    /*Ϊ�˼����ظ���ʼλ���������Ƚ�SDA���ߡ�����SCL�����ߣ�����ʱ��SDAΪ�͵�ƽ(��ǰʱ�����)��
    ��SDA���ߣ��ͻ����SCL�ߵ�ƽ�ڼ�SDA�����ء��ӻ�������Ϊ��ֹ����*/
    IIC_W_SDA(1);
    IIC_W_SCL(1);
    IIC_W_SDA(0);
    // ��ʼ������SCLΪ�͵�ƽ
    IIC_W_SCL(0);
}

// ֹͣ�ź�
void IIC_Stop(void)
{
    // SDA�ڽ���ǰ�ĵ�ƽʱ��ȷ���ģ���ǰʱ�����������ǰʱ��������Ǹߵ�ƽ����ô�������˽�������(SCL�ߵ�ƽ�ڼ�SDA������)������������Ҫ�Ƚ�SDA����
    IIC_W_SDA(0);
    IIC_W_SCL(1);
    IIC_W_SDA(1);
}

// ����һ���ֽ�
void IIC_SendByte(uint8_t byte)
{
    // ����ֹͣ�źţ������ź�����󶼻ὫSCL���ͣ����Ե�ǰ������Ҫ����SCL
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        // SCL�͵�ƽʱSDA������
        IIC_W_SDA(byte & (0x80 >> i));
        // SCL���øߵ�ƽ���ôӻ���ȡSDA������
        IIC_W_SCL(1);
        // SCL�����óɵ͵�ƽ(���Է������ݺ�SCL��Ȼ�ǵ͵�ƽ)
        IIC_W_SCL(0);
    }
}

// ����һ���ֽ�
uint8_t IIC_ReceiveByte(void)
{
    uint8_t i, byte = 0x00;

    // �����ͷ�SDA��ӻ���⵽�������ݷ���SDA��
    IIC_W_SDA(1);

    for (i = 0; i < 8; i++)
    {
        IIC_W_SCL(1);
        if (IIC_R_SDA() == 1)
        {
            byte |= (0x80 >> i);
        }
        IIC_W_SCL(0);
    }
    return byte;
}

void IIC_SendAck(uint8_t AckBit)
{
    // SCL�͵�ƽʱSDA������
    IIC_W_SDA(AckBit);
    // SCL���øߵ�ƽ���ôӻ���ȡSDA������
    IIC_W_SCL(1);
    IIC_W_SCL(0);
}

uint8_t IIC_ReceiveAck(void)
{
    uint8_t AckBit;
    IIC_W_SDA(1);
    IIC_W_SCL(1);
    AckBit = IIC_R_SDA();
    IIC_W_SCL(0);
    return AckBit;
}
