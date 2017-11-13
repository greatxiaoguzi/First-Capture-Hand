#ifndef __SYS_H
#define __SYS_H	

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"

//����Ϊ��ຯ��
void WFI_SET(void);			//ִ��WFIָ��
void INTX_DISABLE(void);	//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 
void SystemReset(void);

#define __ENABLE_ALL_INTERRUPT		INTX_ENABLE()
#define __DISABLE_ALL_INTERRUPT		INTX_DISABLE()

#endif

