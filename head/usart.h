#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200

#define UART1_RECEIVE_SIZE		1 * 1024		//UART 存储buff 设置为1k字节
#define UART1_SEND_SIZE			15 * 1024		//UART 发送buff 设置为15k字节

//extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//extern u16 USART_RX_STA;         		//接收状态标记

void Usart1_Init(u32 bound);
void Usart_RX_Enable(USART_TypeDef* USARTx);
void Usart_RX_Disable(USART_TypeDef* USARTx);
void Usart1_Send_Data(u8 *buff, u16 len);
void write_data_to_pc(u16 *dat, u16 len);
void Uart4_Init(u32 bound);
void Uart4_Send_Data(u8 *buff, u16 len);

#endif

