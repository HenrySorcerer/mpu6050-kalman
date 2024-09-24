#include "stm32f1xx_hal.h"
#include "IIC.h"
#include "tim.h"

// /* 定义IIC总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define IIC_GPIO_PORT GPIOA     /* GPIO端口 */
#define IIC_SCL_PIN GPIO_PIN_10 /* 连接到SCL时钟线的GPIO */
#define IIC_SDA_PIN GPIO_PIN_11 /* 连接到SDA数据线的GPIO */

// 软件模拟IIC,GPIO和TIMER的初始化由cubemx生成(gpio.c和tim.c)
void IIC_Init(void)
{
    HAL_TIM_Base_Start(&htim4); // 使能定时器，提供给延时函数
}

void IIC_Delay(uint8_t us) // 微秒级延时
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim4); // 获取当前计数值
    while ((__HAL_TIM_GET_COUNTER(&htim4) - start) < us)
    {
        // 处理定时器溢出的情况
        if (__HAL_TIM_GET_COUNTER(&htim4) < start)
        {
            break; // 如果计数器溢出，直接退出循环
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

// 起始信号
void IIC_Start(void)
{
    /*为了兼容重复起始位，这里需先将SDA拉高。假如SCL先拉高，而此时的SDA为低电平(由前时序决定)，
    将SDA拉高，就会造成SCL高电平期间SDA上升沿。从机会误判为终止条件*/
    IIC_W_SDA(1);
    IIC_W_SCL(1);
    IIC_W_SDA(0);
    // 起始条件后SCL为低电平
    IIC_W_SCL(0);
}

// 停止信号
void IIC_Stop(void)
{
    // SDA在结束前的电平时不确定的，由前时序决定。假如前时序输出的是高电平，那么就做不了结束条件(SCL高电平期间SDA上升沿)。所以这里需要先降SDA拉低
    IIC_W_SDA(0);
    IIC_W_SCL(1);
    IIC_W_SDA(1);
}

// 发送一个字节
void IIC_SendByte(uint8_t byte)
{
    // 除了停止信号，其他信号在最后都会将SCL拉低，所以当前不再需要拉低SCL
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        // SCL低电平时SDA放数据
        IIC_W_SDA(byte & (0x80 >> i));
        // SCL设置高电平，让从机读取SDA的数据
        IIC_W_SCL(1);
        // SCL再设置成低电平(所以发完数据后SCL仍然是低电平)
        IIC_W_SCL(0);
    }
}

// 接收一个字节
uint8_t IIC_ReceiveByte(void)
{
    uint8_t i, byte = 0x00;

    // 主机释放SDA后从机检测到后会把数据放在SDA上
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
    // SCL低电平时SDA放数据
    IIC_W_SDA(AckBit);
    // SCL设置高电平后让从机读取SDA的数据
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
