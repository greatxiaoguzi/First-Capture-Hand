#ifndef __PUBLIC_H_
#define __PUBLIC_H_

#include "sys.h"

//�����������
//������е������ã���߲�����2700����2.7A
#define MOTOR_1_RUNING_CURRENT	1300//700
#define MOTOR_2_RUNING_CURRENT	1500//800
#define MOTOR_3_RUNING_CURRENT	1100
#define MOTOR_4_RUNING_CURRENT	1100

//���������������
#define MOTOR_1_LOCK_CURRENT	100
#define MOTOR_2_LOCK_CURRENT	100
#define MOTOR_3_LOCK_CURRENT	100
#define MOTOR_4_LOCK_CURRENT	100

//���ϸ����ȷ��
#define MOTOR_1_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_2_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_3_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_4_LEVEL_SET		DRV_LEVEL_8

//1:ʹ�ܸù�翪�ض�ӦIO�ⲿ�жϣ�0:ʧ�ܶ�Ӧ�ⲿ�ж�
#define PCT_1_ENABLE			1
#define PCT_2_ENABLE			1
#define PCT_3_ENABLE			1
#define PCT_4_ENABLE			1
#define PCT_5_ENABLE			1
#define PCT_6_ENABLE			1
#define PCT_7_ENABLE			1
#define PCT_8_ENABLE			1

//����ģʽֻ��:EXTI_Trigger_Rising(�����ش���)��EXTI_Trigger_Falling(�½��ش���)��EXTI_Trigger_Rising_Falling(���ش���) 3 �ֿ�ѡ
#define PCT_1_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_2_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_3_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_4_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_5_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_6_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_7_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_8_TRIGGER_MODE		EXTI_Trigger_Falling

//OUTA~OUTH�˿�����,1:���˿�����ΪPWM������ӵ͵�ƽ���ߵ�ƽ������100ms��0:���˿�����Ϊ��ͨIO���
#define CONFIG_OUT_A			1
#define CONFIG_OUT_B			1
#define CONFIG_OUT_C			0
#define CONFIG_OUT_D			0
#define CONFIG_OUT_E			0
#define CONFIG_OUT_F			0
#define CONFIG_OUT_G			0
#define CONFIG_OUT_H			0

typedef enum
{
	MOTOR_1 = 0,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_MAX,
}MOTOR_CHIP_SELECT;

typedef enum
{
	FWD = 0,
	REV = 1
}MOTOR_DIR;

typedef enum
{
	NO_FINISH = 0,
	FINISHED = 1,
}MOTOR_COMPLETION_STATUS;

typedef enum
{
	NO_TRIGGER = 0,
	TRIGGERED = 1,
}MOTOR_TRIGGER_STATUS;

typedef enum
{
	PCT_NO = 0,
	PCT1,
	PCT2,
	PCT3,
	PCT4,
	PCT5,
	PCT6,
	PCT7,
	PCT8,
}PCT_NUMBER;

typedef enum
{
	PCT_ENCODER_NO = 0,
	PCT_ENCODER_1,
	PCT_ENCODER_2,
}PCT_ENCODER_NUMBER;

//��� �����е�������
typedef enum
{
	M_MOTION_FREE,			//�������
	M_MOTION_INIT,			//�����ʼ���׶�ǰ��
	M_MOTION_INIT_RETURN,	//�����ʼ���׶ι���λ
	M_MOTION_INPUT_CABIN,	//�����ʼ����ʱ����λ
//	M_MOTION_ADC_OUT,		//�������ADC �������˶��ɼ�����
//	M_MOTION_ADC_IN,		//�������ADC �������˶��ɼ�����

	MOTION_SINGLE_TUBE_GO,	//�����ǰ��1���Թܵľ���
	MOTION_SINGLE_TUBE_RETURN,//���������1���Թܵľ���

//	M1_FAST_GO,
//	M1_FAST_RETURN,

//	M2_MOTION_SUCK_SAMPLE_POS,
//	M2_MOTION_ADD_SAMPLE_POS,
//	M2_MOTION_WASH_POS,
//	M2_MOTION_ENHANCE_WASH_POS,

}MOTOR_MOTION_INFO;

typedef struct
{
	MOTOR_MOTION_INFO status;//����˶�״̬
	MOTOR_MOTION_INFO prev_status;//�����һ�ε�״̬
	MOTOR_COMPLETION_STATUS finish;	//��״̬�����Ƿ����
	MOTOR_DIR direction;	//����˶�����

	u32 acc_pulse;			//���ٶ��ж��ٸ�����
	u32 dec_pulse;			//���ٶ��ж��ٸ�����
	u32 acc_const_pulse;	//���ٶ�+���ٶ��ж��ٸ�����
	u32 const_pulse;		//���ٶ��ж��ٸ�����

	u32 pulse_count;		//��������������
	u32 pulse_max;			//��������г���Ҫ���е�������

	u8 multiple;			//����Ƶ���ж��ٸ�����

	u32 acc_stage;			//���ٶ��ж��ٸ�Ƶ��
	u32 dec_stage;			//���ٶ��ж��ٸ�Ƶ��
	u32 const_stage;		//���ٶ��ж��ٸ�Ƶ��

	u32 const_stage_count;	//���ٶ�Ƶ�����

	u32 stage_count;		//���Ƶ��������
	u32 stage_max;			//����ܹ��ж��ٸ�Ƶ��(���ٸ�̨��)

	u16 *speed_up_buff;		//��������˶��������߲���
	u16 *speed_down_buff;	//��������˶��������߲���

	MOTOR_TRIGGER_STATUS triggered;//�����������翪���ˣ�1��ʾ�Ѵ�����0��ʾδ����

	s16 offset;				//΢�����ʱ���ƫ����

	PCT_NUMBER pct_number;	//�ö���ʹ�õ��ĸ���翪��
	u8 pct_pre_status;		//��翪��֮ǰ��״̬
	u8 pct_current_status;	//��翪�����ڵ�״̬
	u16 pct_zero_run_pulse;	//��翪�ر������������ж��ٸ����嵽�������

	PCT_ENCODER_NUMBER pct_encoder_number;//�ö���ʹ�õ��ĸ���������
	u32 pct_encoder_count;	//����������ֵ
	u32 pct_encoder_max;	//�����������г̼��������ֵ
}MOTOR_INFO;

#endif

