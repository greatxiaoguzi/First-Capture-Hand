#ifndef __MAIN_APP_H_
#define __MAIN_APP_H_
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "public.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "step_motor.h"
#include "can.h"
#include "at24cxx.h"
#include "em_sw.h"


#define PI        3.14159265358
////�ر������ж�
//void INTX_DISABLE(void)
//{		  
//	__ASM volatile("cpsid i");
//}
////���������ж�
//void INTX_ENABLE(void)
//{
//	__ASM volatile("cpsie i");		  
//}
//�����λ�ò���
typedef struct
{
	u32 acc_stage;			//���ٶ��ж��ٸ�Ƶ��
	u32 dec_stage;			//���ٶ��ж��ٸ�Ƶ��
	u32 const_stage;		//���ٶ��ж��ٸ�Ƶ��
	u8 multiple;
	u16 pct_zero_run_pulse;	//��翪�ر������������ж��ٸ����嵽�������
	struct
	{
		u32 acc_stage;			//���ٶ��ж��ٸ�Ƶ��
		u32 dec_stage;			//���ٶ��ж��ٸ�Ƶ��
		u32 const_stage;		//���ٶ��ж��ٸ�Ƶ��
		u8 multiple;
		u16 SpeedOffset;
		u16 pct_zero_run_pulse;
	}Output_CabinPara;
}MotorResetPara_TypeDef;

void Init(void);
u8 Can_Receiver_Analyze(Package_Info * data);
void Create_Speed_Change_Table(u16 min_speed, u16 max_speed, u16 point_num)	;
u8 GetAllMotorStatus(void);


void CombineReset(void);
void Motor_Init_Motion(MOTOR_CHIP_SELECT sel,u16 *AccTab,u16 *DecTab);
void Test_SW_Status(Package_Info* package_data);
void TestEeprom(void);

u16 GetUsartData(void);
void UsartSendCmd(u8 Cmd,u16 Datalen,u8 *databuf);
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);


#endif

