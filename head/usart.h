#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200

#define UART1_RECEIVE_SIZE		1 * 1024		//UART �洢buff ����Ϊ1k�ֽ�
#define UART1_SEND_SIZE			15 * 1024		//UART ����buff ����Ϊ15k�ֽ�

//extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
//extern u16 USART_RX_STA;         		//����״̬���

void Usart1_Init(u32 bound);
void Usart_RX_Enable(USART_TypeDef* USARTx);
void Usart_RX_Disable(USART_TypeDef* USARTx);
void Usart1_Send_Data(u8 *buff, u16 len);
void write_data_to_pc(u16 *dat, u16 len);
void Uart4_Init(u32 bound);
void Uart4_Send_Data(u8 *buff, u16 len);

#endif

