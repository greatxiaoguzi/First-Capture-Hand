#ifndef __STEP_MOTOR__H
#define __STEP_MOTOR__H

#include <math.h>
#include "sys.h"
#include "delay.h"
#include "spi.h"
#include "drv8711.h"
#include "timer.h"
#include "pct_encoder.h"
#include "public.h"

//相关宏定义

//电机相关引脚
#define Motor_1_Reset_On		GPIO_SetBits(GPIOE, GPIO_Pin_7)		//电机1复位
#define Motor_1_Reset_Off		GPIO_ResetBits(GPIOE, GPIO_Pin_7)	//关闭电机1复位

#define Motor_2_Reset_On		GPIO_SetBits(GPIOE, GPIO_Pin_13)	//电机2复位
#define Motor_2_Reset_Off		GPIO_ResetBits(GPIOE, GPIO_Pin_13)	//关闭电机2复位

#define Motor_3_Reset_On		GPIO_SetBits(GPIOC, GPIO_Pin_5)		//电机3复位
#define Motor_3_Reset_Off		GPIO_ResetBits(GPIOC, GPIO_Pin_5)	//关闭电机3复位

#define Motor_4_Reset_On		GPIO_SetBits(GPIOC, GPIO_Pin_0)		//电机4复位
#define Motor_4_Reset_Off		GPIO_ResetBits(GPIOC, GPIO_Pin_0)	//关闭电机4复位

#define Motor_1_Dir_Forward		GPIO_SetBits(GPIOD, GPIO_Pin_13)	//电机1顺时针转
#define Motor_1_Dir_Invert		GPIO_ResetBits(GPIOD, GPIO_Pin_13)	//电机1逆时针转

#define Motor_2_Dir_Forward		GPIO_SetBits(GPIOB, GPIO_Pin_3)		//电机2顺时针转
#define Motor_2_Dir_Invert		GPIO_ResetBits(GPIOB, GPIO_Pin_3)	//电机2逆时针转

#define Motor_3_Dir_Forward		GPIO_SetBits(GPIOB, GPIO_Pin_5)		//电机3顺时针转
#define Motor_3_Dir_Invert		GPIO_ResetBits(GPIOB, GPIO_Pin_5)	//电机3逆时针转

#define Motor_4_Dir_Forward		GPIO_SetBits(GPIOA, GPIO_Pin_1)		//电机4顺时针转
#define Motor_4_Dir_Invert		GPIO_ResetBits(GPIOA, GPIO_Pin_1)	//电机4逆时针转

#define Motor_1_nFAULT			GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_0)	//电机1反馈错误信息中断引脚
#define Motor_2_nFAULT			GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15)	//电机2反馈错误信息中断引脚
#define Motor_3_nFAULT			GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_12)	//电机3反馈错误信息中断引脚
#define Motor_4_nFAULT			GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)	//电机4反馈错误信息中断引脚

#define Motor_1_nSTALL			GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1)	//电机1发生了紧急停止中断引脚
#define Motor_2_nSTALL			GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)	//电机2发生了紧急停止中断引脚
#define Motor_3_nSTALL			GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13)	//电机3发生了紧急停止中断引脚
#define Motor_4_nSTALL			GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)	//电机4发生了紧急停止中断引脚

#if 0
//Motor_1_BEMF		PF7		ADC3_IN5
//Motor_2_BEMF		PF8		ADC3_IN6
//Motor_3_BEMF		PF9		ADC3_IN7
//Motor_4_BEMF		PF10	ADC3_IN8

//Motor_1_STEP		PA15	TIM2_CH1
//Motor_2_STEP		PB4		TIM3_CH1
//Motor_3_STEP		PD12	TIM4_CH1
//Motor_4_STEP		PA0		TIM5_CH1
#endif

void Motor_Init(void);
void Motor_Enable(MOTOR_CHIP_SELECT sel);
void Motor_Disable(MOTOR_CHIP_SELECT sel);
void Motor_Self_Lock(MOTOR_CHIP_SELECT sel);
void Motor_Motion_Setup(MOTOR_CHIP_SELECT sel);

#endif

