#ifndef __EM_SW__H
#define __EM_SW__H

#include "sys.h"

//相关宏定义
#define OUTA_Low    GPIO_ResetBits(GPIOD, GPIO_Pin_7)
#define OUTA_High   GPIO_SetBits(GPIOD, GPIO_Pin_7)

#define OUTB_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_9)
#define OUTB_High   GPIO_SetBits(GPIOG, GPIO_Pin_9)

#define OUTC_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_10)
#define OUTC_High   GPIO_SetBits(GPIOG, GPIO_Pin_10)

#define OUTD_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_11)
#define OUTD_High   GPIO_SetBits(GPIOG, GPIO_Pin_11)

#define OUTE_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_12)
#define OUTE_High   GPIO_SetBits(GPIOG, GPIO_Pin_12)

#define OUTF_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_13)
#define OUTF_High   GPIO_SetBits(GPIOG, GPIO_Pin_13)

#define OUTG_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_14)
#define OUTG_High   GPIO_SetBits(GPIOG, GPIO_Pin_14)

#define OUTH_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_15)
#define OUTH_High   GPIO_SetBits(GPIOG, GPIO_Pin_15)

typedef enum
{
	EM_SW_STATUS_OFF = 0,//电磁阀断开/直流电机停止转动
	EM_SW_STATUS_ON,	//电磁阀吸合/直流电机开始转动
}EM_SW_STATUS;

typedef enum
{
	EM_SW_CH_A = 0,		//对应OUT-A接口
	EM_SW_CH_B,			//对应OUT-B接口
	EM_SW_CH_C,			//对应OUT-C接口
	EM_SW_CH_D,			//对应OUT-D接口
	EM_SW_CH_E,			//对应OUT-E接口
	EM_SW_CH_F,			//对应OUT-F接口
	EM_SW_CH_G,			//对应OUT-G接口
	EM_SW_CH_H,			//对应OUT-H接口
	EM_SW_CH_MAX
}EM_SW_CH;

typedef struct
{
	FunctionalState enable[8];//当电磁阀闭合时，在闭合开始点使能，完全闭合后关闭，主要是监测闭合过程是否完成
	EM_SW_STATUS status[8];	//对应电磁阀/直流电机工作状态，打开还是关闭
	u8 count_100us[8];		//PWM最小输出脉宽100us
	u8 count_1ms[8];		//PWM周期1ms
	u8 count_10ms[8];		//10ms切换一次PWM占空比	
	u8 flag;				//统计当前有几个电磁阀/直流电机在工作，8bit代表8个电磁阀，每个bit，如果是1表示在工作，0表示不工作
}EM_SW_CTRL_INFO;

void EM_SW_Init(void);
void EM_SW_Set_IO(EM_SW_CH ch);
void EM_SW_Reset_IO(EM_SW_CH ch);
EM_SW_STATUS EM_SW_Get_Status(EM_SW_CH ch);
void EM_SW_OUT(EM_SW_CH ch, EM_SW_STATUS status);

#endif

