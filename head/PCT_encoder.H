#ifndef _PCT_ENCODER_H_
#define _PCT_ENCODER_H_

#include "sys.h"

//相关宏定义实现

#define  Read_OSB   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)
#define  Read_OSA   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)
#define  Read_OSI   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)

void PCT_Encoder_Init(void);
void PCT_Exti_Set(void);
void PCT_Timer_Set(u16 arr);
void PCT_Encoder_Para_Init(u32 count);
void PCT_Encoder_Para_Set(void);

#endif

