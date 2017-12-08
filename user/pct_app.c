#include "pct_app.h"
#include "MotionDispatch.h"
#include "Config.h"
extern MOTOR_INFO motor_info[4];
extern u32 Mix_Circle_Cnt;
extern u32 Mix_Max_Circle;
u32 X_Debug_Position;
u32 Y_Debug_Position;
u32 Z_Debug_Position;
//获取所有的光电开关的当前状态
u8 GetAllPctStatus(void)
{
	u8 PctStatus;
	PctStatus = (!(u8)PCT_8_STATUS)<<7|(!(u8)PCT_7_STATUS)<<6|(u8)PCT_6_STATUS<<5|(u8)PCT_5_STATUS<<4|
	(!(u8)PCT_4_STATUS)<<3|(u8)PCT_3_STATUS<<2|(u8)PCT_2_STATUS<<1|(u8)PCT_1_STATUS;
	return PctStatus;
}
#if PCT_1_ENABLE == 1
void PCT_1_Handle(void)   //X轴光电开关
{
	//LED_Reversal();
	if(PCT_1_STATUS == 1)
	{
		//delay_us(10);
		if(PCT_1_STATUS == 1)
		{
			switch(motor_info[MOTOR_1].status)
			{
				case M_MOTION_INIT:
				case X_TO_ORIGIN:
				/*case M_MOTION_INIT_RETURN:*/
					//TIMx_Disable(TIM4);
					X_Debug_Position = motor_info[MOTOR_1].pulse_count;
					motor_info[MOTOR_1].pulse_count = motor_info[MOTOR_1].pulse_max - X_PCT_ZERO_RUN_PULSE;
					//TIMx_Enable(TIM4);
				break;
				default:break;
			}
		}
	}
}
#endif

#if PCT_2_ENABLE == 1
void PCT_2_Handle(void)//Y轴光电开关
{
	//LED_Reversal();
	if(PCT_2_STATUS == 1)
	{
		//delay_us(10);
		if(PCT_2_STATUS == 1)
		{
			switch(motor_info[MOTOR_2].status)
			{
				case M_MOTION_INIT:
				case Y_TO_ORIGIN:
				/*case M_MOTION_INIT_RETURN:*/
					Y_Debug_Position = motor_info[MOTOR_2].pulse_count;
					motor_info[MOTOR_2].pulse_count = motor_info[MOTOR_2].pulse_max - Y_PCT_ZERO_RUN_PULSE;
					break;
				default:break;
			}
		}
	}
	
}
#endif

#if PCT_3_ENABLE == 1
void PCT_3_Handle(void)			//Z轴光电开关
{
	//LED_Reversal();
	if(PCT_3_STATUS == 1)
	{
		if(PCT_3_STATUS == 1)
		{
			switch(motor_info[MOTOR_3].status)
			{
				case M_MOTION_INIT:
				case Z_TO_ORIGIN:
					//TIMx_Disable(TIM3);
					Z_Debug_Position = motor_info[MOTOR_3].pulse_count;
					motor_info[MOTOR_3].pulse_count = motor_info[MOTOR_3].pulse_max - Z_PCT_ZERO_RUN_PULSE;
					//TIMx_Enable(TIM3);  
					break;
				default:break;
			}
		}
	}
}
#endif

#if PCT_4_ENABLE == 1 
void PCT_4_Handle(void)//混匀盘光电开关
{
	//LED_Reversal();
	if(PCT_4_STATUS == 0)
	{
		delay_us(10);
		if(PCT_4_STATUS == 0)
		{
			switch(motor_info[MOTOR_4].status)
			{
				case M_MOTION_INIT:
				case MIX_TO_ORIGIN:
				/*case M_MOTION_INIT_RETURN:*/
					motor_info[MOTOR_4].pulse_count = motor_info[MOTOR_4].pulse_max - MIX_PCT_ZERO_RUN_PULSE;
					break;
				case MIX_START_RUNING:
				{
					Mix_Circle_Cnt ++;
					if(Mix_Circle_Cnt >= Mix_Max_Circle)
					{
						motor_info[MOTOR_4].pulse_count = motor_info[MOTOR_4].pulse_max - MIX_PCT_ZERO_RUN_PULSE;
						Mix_Circle_Cnt = 0;
					}
				}break;
				default:break;
			}
		}
	}
}
#endif
extern COM_ListTypeDef CurrComMotion;  		//组合动作运行的当前动作
extern FaultLevel_TypeDef CurrFaultLevel;
extern CaptureFinger_Status_TypeDef CaptureFingerStatus;
//抓杯手指光电开关
#if PCT_5_ENABLE == 1
void PCT_5_Handle(void)
{
	if(PCT_5_STATUS == 0)
	{
		delay_ms(10);
		if(PCT_5_STATUS == 0)
		{
//			switch(CurrComMotion)    //根据不同调度过程分别置位运行等级
//			{
//				case C0M_CUP_TO_HATCHIN:		CaptureFingerStatus = CAPTURE_FINGER_CATCHED_CUP;break;
//				case C0M_CUP_TO_HATCHOUT:		CaptureFingerStatus = CAPTURE_FINGER_CATCHED_CUP;break;
//				case C0M_HATCH_TO_MIX:			CaptureFingerStatus = CAPTURE_FINGER_CATCHED_CUP;break;
//				case C0M_MIX_TO_HATCHIN:		CaptureFingerStatus = CAPTURE_FINGER_UNCATCHED_CUP;break;
//				case C0M_MIX_TO_HATCHOUT:		CaptureFingerStatus = CAPTURE_FINGER_CATCHED_CUP;break;
//				case C0M_HATCH_TO_WASTE:		CaptureFingerStatus = CAPTURE_FINGER_CATCHED_CUP;break;
//				case COM_UNLOAD_CUP:			CaptureFingerStatus = CAPTURE_FINGER_THROW_AWAY_CUP;break;
//				default:break;
//			}
		}
	}
	else if(PCT_5_STATUS == 1)
	{
		delay_ms(10);
		if(PCT_5_STATUS == 1)
		{
		}
	}
}
#endif
//废料桶1光电开关
#if PCT_6_ENABLE == 1
void PCT_6_Handle(void)
{
	if(PCT_6_STATUS == 0)
	{
		delay_ms(10);
		if(PCT_6_STATUS == 0)
		{
			
		}
	}
	else if(PCT_6_STATUS == 1)
	{
		delay_ms(10);
		if(PCT_6_STATUS == 1)
		{
		
		}
	}
}
#endif
//废料桶2光电开关
#if PCT_7_ENABLE == 1
void PCT_7_Handle(void)
{
	if(PCT_7_STATUS == 0)
	{
		delay_ms(10);
		if(PCT_7_STATUS == 0)
		{
			
		}
	}
	else if(PCT_7_STATUS == 1)
	{
		delay_ms(10);
		if(PCT_7_STATUS == 1)
		{
			
		}
	}
}
#endif
//抽屉光电开关
#if PCT_8_ENABLE
void PCT_8_Handle(void)  //抽屉光电开关用
{
	//LED_Reversal();
	if(PCT_8_STATUS == 0)
	{
		delay_us(10);
		if(PCT_8_STATUS == 0)
		{
			CabinorEnable(CABIN_ELECTM_ENABLE);
		}
	}
	else if(PCT_8_STATUS == 1)
	{
		delay_ms(10);
		if(PCT_8_STATUS == 1)
			CurrFaultLevel = PAUSE_LEVEL;
	}
}
#endif

















