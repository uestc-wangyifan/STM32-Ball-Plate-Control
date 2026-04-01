#include "oled_i2c.h"

// 极短延时，匹配 STM32F103 的 72MHz 主频
void IIC_delay(void)
{
    uint8_t t = 3;
    while(t--);
}

// 产生IIC起始信号
void IIC_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Clr();
    IIC_delay();
    OLED_SCL_Clr(); // 钳住I2C总线，准备发送或接收数据
}

// 产生IIC停止信号
void IIC_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Set(); // 发送I2C总线结束信号
}

// 等待应答信号到来
// OLED是单向写设备，为了极致效率和避开HAL库繁琐的输入输出切换，我们直接忽略硬件ACK
unsigned char IIC_Wait_Ack(void)
{
    OLED_SDA_Set();
    IIC_delay();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
    return 0; // 永远返回 0 (假装应答成功)，保证程序绝对不卡死
}

// IIC发送一个字节
void IIC_Send_Byte(unsigned char txd)
{
    unsigned char t;
    for(t = 0; t < 8; t++)
    {
        if((txd & 0x80) >> 7)
            OLED_SDA_Set();
        else
            OLED_SDA_Clr();
            
        txd <<= 1;
        IIC_delay();
        OLED_SCL_Set();
        IIC_delay();
        OLED_SCL_Clr();
        IIC_delay();
    }
}

static int OLED_WriteByte(unsigned char data, unsigned char control)
{
    IIC_Start();
    IIC_Send_Byte(0x78);
    IIC_Wait_Ack();
    IIC_Send_Byte(control);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
    return 0;
}

int OLED_Command(unsigned char Command)
{
    return OLED_WriteByte(Command, 0x00);
}

int OLED_Data(unsigned char Data)
{
    return OLED_WriteByte(Data, 0x40);
}
























