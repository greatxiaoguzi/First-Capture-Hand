#ifndef __ADC__H
#define __ADC__H

#include "sys.h"

typedef enum
{
	ADC_AIN1 = ADC_Channel_14,
	ADC_AIN2 = ADC_Channel_4,
}ADC_CHANNEL_SEL;

void Init_ADC(void);
u16 Get_Adc(ADC_CHANNEL_SEL ch);
u16 Get_Adc_Average(ADC_CHANNEL_SEL ch, u8 times);

#endif

