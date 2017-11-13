#ifndef __AT24CXX__H
#define __AT24CXX__H

#include "iic.h"

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767

#define EE_TYPE		AT24C08

#define AT24CXX_PTOTECT_ON		GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define AT24CXX_PTOTECT_OFF		GPIO_ResetBits(GPIOB, GPIO_Pin_8)

#define USER_ADDRESS		0x0000
#define	FACTORY_ADDRESS		0x0200

#define ADDR		1023
#define WRITE_DATA	0xaa

void AT24CXX_Init(void);
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead);
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite);
u8 AT24CXX_Check(void);

#endif

