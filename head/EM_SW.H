#ifndef __EM_SW__H
#define __EM_SW__H

#include "sys.h"

//��غ궨��
#define OUTA_Low    GPIO_ResetBits(GPIOD, GPIO_Pin_7)
#define OUTA_High   GPIO_SetBits(GPIOD, GPIO_Pin_7)

#define OUTB_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_9)
#define OUTB_High   GPIO_SetBits(GPIOG, GPIO_Pin_9)

#define OUTC_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_10)
#define OUTC_High   GPIO_SetBits(GPIOG, GPIO_Pin_10)

#define OUTD_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_11)
#define OUTD_High   GPIO_SetBits(GPIOG, GPIO_Pin_11)

#define OUTE_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_12)
#define OUTE_High   GPIO_SetBits(GPIOG, GPIO_Pin_12)

#define OUTF_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_13)
#define OUTF_High   GPIO_SetBits(GPIOG, GPIO_Pin_13)

#define OUTG_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_14)
#define OUTG_High   GPIO_SetBits(GPIOG, GPIO_Pin_14)

#define OUTH_Low    GPIO_ResetBits(GPIOG, GPIO_Pin_15)
#define OUTH_High   GPIO_SetBits(GPIOG, GPIO_Pin_15)

typedef enum
{
	EM_SW_STATUS_OFF = 0,//��ŷ��Ͽ�/ֱ�����ֹͣת��
	EM_SW_STATUS_ON,	//��ŷ�����/ֱ�������ʼת��
}EM_SW_STATUS;

typedef enum
{
	EM_SW_CH_A = 0,		//��ӦOUT-A�ӿ�
	EM_SW_CH_B,			//��ӦOUT-B�ӿ�
	EM_SW_CH_C,			//��ӦOUT-C�ӿ�
	EM_SW_CH_D,			//��ӦOUT-D�ӿ�
	EM_SW_CH_E,			//��ӦOUT-E�ӿ�
	EM_SW_CH_F,			//��ӦOUT-F�ӿ�
	EM_SW_CH_G,			//��ӦOUT-G�ӿ�
	EM_SW_CH_H,			//��ӦOUT-H�ӿ�
	EM_SW_CH_MAX
}EM_SW_CH;

typedef struct
{
	FunctionalState enable[8];//����ŷ��պ�ʱ���ڱպϿ�ʼ��ʹ�ܣ���ȫ�պϺ�رգ���Ҫ�Ǽ��պϹ����Ƿ����
	EM_SW_STATUS status[8];	//��Ӧ��ŷ�/ֱ���������״̬���򿪻��ǹر�
	u8 count_100us[8];		//PWM��С�������100us
	u8 count_1ms[8];		//PWM����1ms
	u8 count_10ms[8];		//10ms�л�һ��PWMռ�ձ�	
	u8 flag;				//ͳ�Ƶ�ǰ�м�����ŷ�/ֱ������ڹ�����8bit����8����ŷ���ÿ��bit�������1��ʾ�ڹ�����0��ʾ������
}EM_SW_CTRL_INFO;

void EM_SW_Init(void);
void EM_SW_Set_IO(EM_SW_CH ch);
void EM_SW_Reset_IO(EM_SW_CH ch);
EM_SW_STATUS EM_SW_Get_Status(EM_SW_CH ch);
void EM_SW_OUT(EM_SW_CH ch, EM_SW_STATUS status);

#endif
