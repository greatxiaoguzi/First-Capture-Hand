#ifndef __PUBLIC_H_
#define __PUBLIC_H_

#include "sys.h"

//电机部分设置
//电机运行电流设置，最高不超过2700，即2.7A
#define MOTOR_1_RUNING_CURRENT	1300//700
#define MOTOR_2_RUNING_CURRENT	1500//800
#define MOTOR_3_RUNING_CURRENT	1100
#define MOTOR_4_RUNING_CURRENT	1100

//电机自锁电流设置
#define MOTOR_1_LOCK_CURRENT	100
#define MOTOR_2_LOCK_CURRENT	100
#define MOTOR_3_LOCK_CURRENT	100
#define MOTOR_4_LOCK_CURRENT	100

//电机细分数确定
#define MOTOR_1_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_2_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_3_LEVEL_SET		DRV_LEVEL_8
#define MOTOR_4_LEVEL_SET		DRV_LEVEL_8

//1:使能该光电开关对应IO外部中断；0:失能对应外部中断
#define PCT_1_ENABLE			1
#define PCT_2_ENABLE			1
#define PCT_3_ENABLE			1
#define PCT_4_ENABLE			1
#define PCT_5_ENABLE			1
#define PCT_6_ENABLE			1
#define PCT_7_ENABLE			1
#define PCT_8_ENABLE			1

//触发模式只有:EXTI_Trigger_Rising(上升沿触发)、EXTI_Trigger_Falling(下降沿触发)、EXTI_Trigger_Rising_Falling(边沿触发) 3 种可选
#define PCT_1_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_2_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_3_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_4_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_5_TRIGGER_MODE		EXTI_Trigger_Rising
#define PCT_6_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_7_TRIGGER_MODE		EXTI_Trigger_Falling
#define PCT_8_TRIGGER_MODE		EXTI_Trigger_Falling

//OUTA~OUTH端口配置,1:将端口配置为PWM输出，从低电平到高电平上升沿100ms；0:将端口配置为普通IO输出
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

//电机 的所有单个动作
typedef enum
{
	M_MOTION_FREE,			//电机空闲
	M_MOTION_INIT,			//电机初始化阶段前进
	M_MOTION_INIT_RETURN,	//电机初始化阶段归零位
	M_MOTION_INPUT_CABIN,	//电机开始动作时归零位
//	M_MOTION_ADC_OUT,		//电机启动ADC 并向外运动采集数据
//	M_MOTION_ADC_IN,		//电机启动ADC 并向内运动采集数据

	MOTION_SINGLE_TUBE_GO,	//电机往前走1个试管的距离
	MOTION_SINGLE_TUBE_RETURN,//电机往回走1个试管的距离

//	M1_FAST_GO,
//	M1_FAST_RETURN,

//	M2_MOTION_SUCK_SAMPLE_POS,
//	M2_MOTION_ADD_SAMPLE_POS,
//	M2_MOTION_WASH_POS,
//	M2_MOTION_ENHANCE_WASH_POS,

}MOTOR_MOTION_INFO;

typedef struct
{
	MOTOR_MOTION_INFO status;//电机运动状态
	MOTOR_MOTION_INFO prev_status;//电机上一次的状态
	MOTOR_COMPLETION_STATUS finish;	//本状态运行是否完成
	MOTOR_DIR direction;	//电机运动方向

	u32 acc_pulse;			//加速段有多少个脉冲
	u32 dec_pulse;			//减速段有多少个脉冲
	u32 acc_const_pulse;	//加速段+匀速段有多少个脉冲
	u32 const_pulse;		//匀速段有多少个脉冲

	u32 pulse_count;		//电机驱动脉冲计数
	u32 pulse_max;			//电机整个行程需要运行的脉冲数

	u8 multiple;			//单个频点有多少个脉冲

	u32 acc_stage;			//加速段有多少个频点
	u32 dec_stage;			//减速段有多少个频点
	u32 const_stage;		//匀速段有多少个频点

	u32 const_stage_count;	//匀速段频点计数

	u32 stage_count;		//电机频点数计数
	u32 stage_max;			//电机总共有多少个频点(多少个台阶)

	u16 *speed_up_buff;		//电机加速运动调用曲线参数
	u16 *speed_down_buff;	//电机减速运动调用曲线参数

	MOTOR_TRIGGER_STATUS triggered;//电机触发过光电开关了，1表示已触发，0表示未触发

	s16 offset;				//微调电机时电机偏移量

	PCT_NUMBER pct_number;	//该动作使用的哪个光电开关
	u8 pct_pre_status;		//光电开关之前的状态
	u8 pct_current_status;	//光电开关现在的状态
	u16 pct_zero_run_pulse;	//光电开关被触发后再运行多少个脉冲到绝对零点

	PCT_ENCODER_NUMBER pct_encoder_number;//该动作使用的哪个光电编码器
	u32 pct_encoder_count;	//编码器计数值
	u32 pct_encoder_max;	//编码器根据行程计算出来的值
}MOTOR_INFO;

#endif

