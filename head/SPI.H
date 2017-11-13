#ifndef __SPI1__H
#define __SPI1__H

#include "sys.h"
#include "delay.h"
#include "public.h"

#define DRV8711_CS1_ENABLE			GPIO_SetBits(GPIOE, GPIO_Pin_8)		//第一片DRV8711使能
#define DRV8711_CS1_DISABLE			GPIO_ResetBits(GPIOE, GPIO_Pin_8)	//第一片DRV8711禁止使能

#define DRV8711_CS2_ENABLE			GPIO_SetBits(GPIOE, GPIO_Pin_14)	//第二片DRV8711使能
#define DRV8711_CS2_DISABLE			GPIO_ResetBits(GPIOE, GPIO_Pin_14)	//第二片DRV8711禁止使能

#define DRV8711_CS3_ENABLE			GPIO_SetBits(GPIOF, GPIO_Pin_11)	//第三片DRV8711使能
#define DRV8711_CS3_DISABLE			GPIO_ResetBits(GPIOF, GPIO_Pin_11)	//第三片DRV8711禁止使能

#define DRV8711_CS4_ENABLE			GPIO_SetBits(GPIOC, GPIO_Pin_1)		//第四片DRV8711使能
#define DRV8711_CS4_DISABLE			GPIO_ResetBits(GPIOC, GPIO_Pin_1)	//第四片DRV8711禁止使能

#define EXTERNAL_SPI_CS_ENABLE		GPIO_ResetBits(GPIOB, GPIO_Pin_12)	//外部SPI片选脚使能
#define EXTERNAL_SPI_CS_DISABLE		GPIO_SetBits(GPIOB, GPIO_Pin_12)	//外部SPI片选脚禁止使能

#define DRV8711_ALL_CS_DISABLE		DRV8711_CS1_DISABLE; DRV8711_CS2_DISABLE; DRV8711_CS3_DISABLE; DRV8711_CS4_DISABLE;		//4片DRV8711都禁止使能

//相关函数声明
void SPI1_Init(void);
u16 SPI1_ReadWriteByte(MOTOR_CHIP_SELECT x, u16 TxData);
void SPI2_Init(void);
u16 SPI2_ReadWriteByte(u8 TxData);

#endif

