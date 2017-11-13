#ifndef __SYS_H
#define __SYS_H	

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"

//以下为汇编函数
void WFI_SET(void);			//执行WFI指令
void INTX_DISABLE(void);	//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址 
void SystemReset(void);

#define __ENABLE_ALL_INTERRUPT		INTX_ENABLE()
#define __DISABLE_ALL_INTERRUPT		INTX_DISABLE()

#endif

