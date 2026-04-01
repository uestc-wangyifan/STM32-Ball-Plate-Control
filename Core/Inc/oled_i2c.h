#ifndef __OLED_I2C_H
#define __OLED_I2C_H

#include "main.h"

// 精准匹配你在 CubeMX 里设置的 PB8(SCL) 和 PB9(SDA)
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

// HAL库专属引脚翻转宏
#define OLED_SCL_Clr() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET)
#define OLED_SCL_Set() HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET)

#define OLED_SDA_Clr() HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET)
#define OLED_SDA_Set() HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET)

// 严格匹配 oled.c 中调用的函数名
void IIC_delay(void);
void IIC_Start(void);
void IIC_Stop(void);
unsigned char IIC_Wait_Ack(void);      // 注意这里是 unsigned char，不是 void
void IIC_Send_Byte(unsigned char txd); // 匹配 oled.c 的调用名

#endif