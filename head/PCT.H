#ifndef __PCT__H
#define __PCT__H

//引用的相关的库文件的头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "public.h"

//获取光电开关状态
#define PCT_1_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)	//光电开关1状态获取
#define PCT_2_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5)	//光电开关1状态获取
#define PCT_3_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)	//光电开关1状态获取
#define PCT_4_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)	//光电开关1状态获取
#define PCT_5_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)	//光电开关1状态获取
#define PCT_6_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)	//光电开关1状态获取
#define PCT_7_STATUS	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0)	//光电开关1状态获取
#define PCT_8_STATUS	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)	//光电开关1状态获取

#define PCT_TRIGGER				0
#define PCT_NO_TRIGGER			1

//声明相关函数
void PCT_Init(void);
u8 PCT_Get_Status(PCT_NUMBER num);

#endif



















