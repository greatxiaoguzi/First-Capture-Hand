#ifndef __IIC__H
#define __IIC__H

#include "sys.h"

#define	IIC_Speed				400000		//I2C速度设置为400kHz
#define	IIC1_OWN_ADDRESS7		0x0A
#define	EEPROM_Block0_ADDRESS	0xA0
#define	IIC_PageSize			16			//ATC08每页有16个字节

void IIC_Init(void);
void IIC_EE_ByteWrite(u8 *pBuffer, u16 WriteAddr);
void IIC_EE_PageWrite(u8 *pBuffer, u16 WriteAddr, u8 NumByteToWrite);
void IIC_EE_BufferWrite(u8 *pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void IIC_EE_WaitEepromStandbyState(void);
void IIC_EE_BufferRead(u8 *pBuffer, u16 ReadAddr, u16 NumByteToRead);

#endif 

