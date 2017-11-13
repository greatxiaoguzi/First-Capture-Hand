#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
#include "led.h"
#include "spi.h"
#include "pct.h"
#include "step_motor.h"

void Motor_PWM_Init(u8 count);
void TIMx_Disable(TIM_TypeDef* TIMx);
void TIMx_Enable(TIM_TypeDef* TIMx);
void TIM6_Init(void);
void Timer7_Int_Init(void);
void Timer_Motor_Setup(MOTOR_CHIP_SELECT sel, u16 *buff, u16 count);
#endif

