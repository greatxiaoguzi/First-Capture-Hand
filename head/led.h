#ifndef __LED_H_
#define __LED_H_

#include "sys.h"

#define LED_ON		GPIO_ResetBits(GPIOE, GPIO_Pin_0)
#define LED_OFF		GPIO_SetBits(GPIOE, GPIO_Pin_0)

void LED_Init(void);//≥ı ºªØ
void LED_Reversal(void);
		 				    
#endif
