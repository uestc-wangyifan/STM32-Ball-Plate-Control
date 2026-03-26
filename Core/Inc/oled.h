#ifndef __OLED_H
#define __OLED_H

#include "main.h"       // 【核心修复】用 main.h 替换掉原有的 sys.h 和 stm32f10x.h
#include "oled_i2c.h"   // 引入我们刚刚写好的 I2C 底层驱动
#include <stdlib.h>

#define OLED_CMD  0
#define OLED_DATA 1

// OLED 控制与显示API声明
void OLED_ShowNum(unsigned char x, unsigned char y, unsigned int num, unsigned char TextSize);
void OLED_ShowNum1(unsigned char x, unsigned char y, unsigned int num, unsigned char TextSize);
void OLED_ShowSNum(unsigned char x, unsigned char y, unsigned int num, unsigned char TextSize);
int Num_Digit(int num);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

void OLED_Clear(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_Init(void);
int OLED_Command(unsigned char Command);
int OLED_Data(unsigned char Data);

#endif