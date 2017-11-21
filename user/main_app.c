#include "main_app.h"
#include "usart.h"
#include "MotionDispatch.h"
#include "Config.h"

//小距离用加减速表，非加速到最高速度
u16 mindist_up_table[MOTOR_MAX][100] = 
{
	{16667,2857,1667,1316,1136,1031,962,926,909,893,881,870},
	{16667,2857,1667,1316,1136,1031,962,926,909,893,881,870},
	{1000,984,941,878,803,724,648,578,516,461,432,350,170},
	{8333,2500,1389,982,768,635,545,481,432,394,363,150},
};
u16 mindist_down_table[MOTOR_MAX][100] =
{
	{870,881,893,909,926,962,1031,1136,1316,1667,2857,16667},
	{870,881,893,909,926,962,1031,1136,1316,1667,2857,16667},
	{300,350,432,461,516,578,648,724,803,878,941,984,1000},
	{338,363,394,432,481,545,635,768,982,1389,2500,5882},
};
/*******************************************************************************/
//长距离加减速用
u16 speed_up_table[MOTOR_MAX][100] = 
{
	{
		5882,2500,1389,982,768,635,545,481,432,394,363,338,
		318,300,285,272,260,250,241,233,226,219,213,207,202,
		197,192,188,184,180,177,174,170,167,165,162,160,157,
		155,153,151,149,147,145,143,142,140,139,137,136,135, 
		134,132,131,130,129,128,127,127,126,125
	},
	{
		2500,1389,982,768,635,545,481,432,394,363,338,318,300,
		285,272,260,250,241,233,226,219,213,207,202,197,192,188,184,
		180,177,174,170,167,165,162,160,157,155,153,151,149,147,
		145,143,142,140,139,137,136,135,134,132,131,130,129,128,
		127,127,126,125,
	},
	{
		2500,1389,982,768,635,545,481,432,394,363,338,318,300,285,
		272,260,250,241,233,226,219,213,207,202,197,192,188,184,180,
		177,174,170,167,165,162,160,157,155,153,151,149,147,145,143,
		142,140,139,137,136,135,134,132,131,130,129,128,127,127,126,
		125,
	},
	{
		5882,2500,1389,982,768,635,545,481,432,394,363,338,
		318,300,285,272,260,250,241,233,226,219,213,207,202,
		197,192,188,184,180,177,174,170,167,165,162,160,157,
		155,153,151,149,147,145,143,142,140,139,137,136,135,
		134,132,131,130,129,128,127,127,126,125
	}
};

u16 speed_down_table[MOTOR_MAX][100] = 
{
	{
		125,126,127,127,128,129,130,131,132,134,135,136,
		137,139,140,142,143,145,147,149,151,153,155,157,160,
		162,165,167,170,174,177,180,184,188,192,197,202,207,
		213,219,226,233,241,250,260,272,285,300,318,338,363,
		394,432,481,545,635,768,982,1389,2500,5882
	},
	{
		125,126,127,127,128,129,130,131,132,134,135,136,137,
		139,140,142,143,145,147,149,151,153,155,157,160,162,165,
		167,170,174,177,180,184,188,192,197,202,207,213,219,
		226,233,241,250,260,272,285,300,318,338,363,394,432,
		481,545,635,768,982,1389,2500,
	},
	{
		125,126,127,127,128,129,130,131,132,134,135,136,137,139,
		140,142,143,145,147,149,151,153,155,157,160,162,165,167,
		170,174,177,180,184,188,192,197,202,207,213,219,226,233,
		241,250,260,272,285,300,318,338,363,394,432,481,545,635,
		768,982,1389,2500,
	},
	{
		125,126,127,127,128,129,130,131,132,134,135,136,
		137,139,140,142,143,145,147,149,151,153,155,157,160,
		162,165,167,170,174,177,180,184,188,192,197,202,207,
		213,219,226,233,241,250,260,272,285,300,318,338,363,
		394,432,481,545,635,768,982,1389,2500,5882
	}
};

u16 drv_lock_current[4] = {MOTOR_1_LOCK_CURRENT, MOTOR_2_LOCK_CURRENT, MOTOR_3_LOCK_CURRENT, MOTOR_4_LOCK_CURRENT};
u16 drv_runing_current[4] = {MOTOR_1_RUNING_CURRENT, MOTOR_2_RUNING_CURRENT, MOTOR_3_RUNING_CURRENT, MOTOR_4_RUNING_CURRENT};
u16 drv_level_buff[4] = {MOTOR_1_LEVEL_SET, MOTOR_2_LEVEL_SET, MOTOR_3_LEVEL_SET, MOTOR_4_LEVEL_SET};

EXTITrigger_TypeDef pct_trigger_mode[8] = {PCT_1_TRIGGER_MODE, PCT_2_TRIGGER_MODE, PCT_3_TRIGGER_MODE, PCT_4_TRIGGER_MODE,
											PCT_5_TRIGGER_MODE, PCT_6_TRIGGER_MODE, PCT_7_TRIGGER_MODE, PCT_8_TRIGGER_MODE};

MOTOR_INFO motor_info[4];

void Init(void)
{
	delay_init();	    	 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	Usart1_Init(115200);	 //串口初始化为115200
	LED_Init();			     //LED端口初始化

	AT24CXX_Init();

	CAN_Mode_Init();

	EM_SW_Init();

	PCT_Init();

	Motor_Init();
	LED_ON;
	delay_ms(500);
	LED_OFF;
	CabinorEnable(CABIN_ELECTM_ENABLE);
}
//电机复位用参数(常量)
const MotorResetPara_TypeDef MotorResetPrar[4] = 
{
	{X_PCT_ZERO_RUN_PULSE,X_PCT_ZERO_RUN_PULSE,800,8,0,{0,X_PCT_ZERO_RUN_PULSE,12000,8,10,0}},  	//X
	{Y_PCT_ZERO_RUN_PULSE,Y_PCT_ZERO_RUN_PULSE,800,8,0,{0,Y_PCT_ZERO_RUN_PULSE,12000,8,10,0}}, 		//Y
	{Z_PCT_ZERO_RUN_PULSE,Z_PCT_ZERO_RUN_PULSE,800,8,0,{0,Z_PCT_ZERO_RUN_PULSE,5000,8,29,0}},		//Z
	{MIX_PCT_ZERO_RUN_PULSE,MIX_PCT_ZERO_RUN_PULSE,100,8,0,{0,MIX_PCT_ZERO_RUN_PULSE,1000,8,19,0}}
};
void Motion_Init(MOTOR_CHIP_SELECT sel,MOTOR_DIR Dir,const MotorResetPara_TypeDef *MotorResetPara,u16 *AccTab,u16 *DecTab)
{
	motor_info[sel].status = M_MOTION_INIT;
	motor_info[sel].direction = Dir;
	motor_info[sel].acc_pulse = MotorResetPara->acc_stage;
	motor_info[sel].dec_pulse = MotorResetPara->dec_stage;
	motor_info[sel].const_pulse = MotorResetPara->const_stage;
	motor_info[sel].multiple = MotorResetPara->multiple;
	motor_info[sel].speed_up_buff = AccTab;//speed_up_table[sel];
	motor_info[sel].speed_down_buff = DecTab;//speed_down_table[sel];
	motor_info[sel].pct_number = PCT_NO;
	motor_info[sel].pct_zero_run_pulse = MotorResetPara->pct_zero_run_pulse;
	motor_info[sel].pct_encoder_number = PCT_ENCODER_NO;

	Motor_Motion_Setup(sel);
}

void Motion_Init_Return(MOTOR_CHIP_SELECT sel,MOTOR_DIR Dir,PCT_NUMBER PctSel,const MotorResetPara_TypeDef *MotorResetPara,u16 *AccTab,u16 *DecTab)
{
	motor_info[sel].status = M_MOTION_INIT;
	motor_info[sel].direction = Dir;
	motor_info[sel].acc_pulse = MotorResetPara->acc_stage;
	motor_info[sel].dec_pulse = MotorResetPara->dec_stage;
	motor_info[sel].const_pulse = MotorResetPara->Output_CabinPara.const_stage;
	motor_info[sel].multiple = MotorResetPara->multiple;
	motor_info[sel].speed_up_buff = AccTab;//speed_up_table[sel];
	motor_info[sel].speed_down_buff = DecTab;//speed_down_table[sel];
	motor_info[sel].pct_number = PctSel;
	motor_info[sel].pct_zero_run_pulse = MotorResetPara->pct_zero_run_pulse;
	motor_info[sel].pct_encoder_number = PCT_ENCODER_NO;

	Motor_Motion_Setup(sel);
}

void Motion_Output_Carbin(MOTOR_CHIP_SELECT sel,PCT_NUMBER Pct_Num,MOTOR_DIR Dir,const MotorResetPara_TypeDef *MotorResetPara,u16 *AccTab,u16 *DecTab)
{
	motor_info[sel].status = M_MOTION_INIT;
	motor_info[sel].direction = Dir;
	motor_info[sel].acc_pulse = MotorResetPara->Output_CabinPara.acc_stage;
	motor_info[sel].dec_pulse = MotorResetPara->Output_CabinPara.dec_stage;
	motor_info[sel].const_pulse = MotorResetPara->Output_CabinPara.const_stage;	//这个长度要根据实际尺寸计算的
	motor_info[sel].multiple = MotorResetPara->Output_CabinPara.multiple;
	motor_info[sel].speed_up_buff = AccTab + MotorResetPara->Output_CabinPara.SpeedOffset;//&speed_up_table[sel][MotorResetPara->Output_CabinPara.SpeedOffset];
	motor_info[sel].speed_down_buff = DecTab;//speed_down_table[sel];
	motor_info[sel].pct_number = Pct_Num;
	motor_info[sel].pct_zero_run_pulse = MotorResetPara->Output_CabinPara.pct_zero_run_pulse;
	motor_info[sel].pct_encoder_number = PCT_ENCODER_NO;

	Motor_Motion_Setup(sel);
}

void Mtion_Void(MOTOR_CHIP_SELECT sel)
{
	
}
//系统硬件状态
extern SysStatusInquiry_TypeDef SysCurrSataus;
DeviceUnit_TypeDef MotorTab[4] = {DEVICE_UNIT_X,DEVICE_UNIT_Y,DEVICE_UNIT_Z,DEVICE_UNIT_MIX};
//所有电机组合复位
void CombineReset(void)
{
	
	Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[2],speed_up_table[MotorTab[2]],speed_down_table[MotorTab[2]]);
	delay_ms(50);
	Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[0],speed_up_table[MotorTab[0]],speed_down_table[MotorTab[0]]);
	delay_ms(50);
	Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[1],speed_up_table[MotorTab[1]],speed_down_table[MotorTab[1]]);
	delay_ms(50);
	Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[3],speed_up_table[MotorTab[3]],speed_down_table[MotorTab[3]]);
}
void Motor_Init_Motion(MOTOR_CHIP_SELECT sel,u16 *AccTab,u16 *DecTab)
{
	switch(sel)
	{
		case MOTOR_1:
		{
			//	if(PCT1 == PCT_TRIGGER) 							//光电编码器被挡住了
			if(!PCT_Get_Status(PCT1) == PCT_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT1) == PCT_TRIGGER)
				{
					Motion_Init(MOTOR_1,REV,&MotorResetPrar[0],mindist_up_table[MOTOR_1],mindist_down_table[MOTOR_1]);
					delay_ms(500);
					while(motor_info[MOTOR_1].finish == NO_FINISH);
					Motion_Init_Return(MOTOR_1,FWD,PCT_NO,&MotorResetPrar[0],mindist_up_table[MOTOR_1],mindist_down_table[MOTOR_1]);
					delay_ms(500);
					while(motor_info[MOTOR_1].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_X= 1;
				}
			}
			else if(!PCT_Get_Status(PCT1) == PCT_NO_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT1) == PCT_NO_TRIGGER)
				{
					Motion_Output_Carbin(MOTOR_1,PCT_NO,FWD,&MotorResetPrar[0],mindist_up_table[MOTOR_1],mindist_down_table[MOTOR_1]);
					delay_ms(500);
					while(motor_info[MOTOR_1].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_X= 1;
					//Motion_Init(MOTOR_1,FWD,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
					//delay_ms(500);
					//Motion_Init_Return(MOTOR_1,REV,X_AXIS_PCT,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
					//delay_ms(500);
					//while(motor_info[MOTOR_1].status != M_MOTION_FREE);
				}
			}
		}break;
		case MOTOR_2:
		{
			//	if(PCT1 == PCT_TRIGGER) 		//光电编码器被挡住了
			if(!PCT_Get_Status(PCT2) == PCT_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT2) == PCT_TRIGGER)
				{
					Motion_Init(MOTOR_2,FWD,&MotorResetPrar[1],mindist_up_table[MOTOR_2],mindist_down_table[MOTOR_2]);
					delay_ms(500);
					while(motor_info[MOTOR_2].finish == NO_FINISH);
					Motion_Init_Return(MOTOR_2,REV,PCT_NO,&MotorResetPrar[1],mindist_up_table[MOTOR_2],mindist_down_table[MOTOR_2]);
					delay_ms(500);
					while(motor_info[MOTOR_2].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_Y= 1;
				}
			}
			else if(!PCT_Get_Status(PCT2) == PCT_NO_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT2) == PCT_NO_TRIGGER)
				{
					Motion_Output_Carbin(MOTOR_2,PCT_NO,REV,&MotorResetPrar[1],mindist_up_table[MOTOR_2],mindist_down_table[MOTOR_2]);
					delay_ms(500);
					while(motor_info[MOTOR_2].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_Y= 1;
					//Motion_Init(MOTOR_2,FWD,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
					//delay_ms(500);
					//while(motor_info[MOTOR_2].status != M_MOTION_FREE);
					//Motion_Init_Return(MOTOR_2,REV,Y_AXIS_PCT,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
					//delay_ms(500);
					//while(motor_info[MOTOR_2].status != M_MOTION_FREE);
				}
			}
		}break;
		case MOTOR_3:
		{
			//	if(PCT1 == PCT_TRIGGER) 		//光电编码器被挡住了
			if(!PCT_Get_Status(PCT3) == PCT_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT3) == PCT_TRIGGER)
				{
					Motion_Init(MOTOR_3,FWD,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
					delay_ms(500);
					while(motor_info[MOTOR_3].finish == NO_FINISH);
//					{
//						delay_ms(10);
//					}
					Motion_Init_Return(MOTOR_3,REV,PCT_NO,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
					delay_ms(500);
					while(motor_info[MOTOR_3].finish == NO_FINISH);
//					{
//						delay_ms(10);
//					}
					SysCurrSataus.HardStatus.UnitStatus.Pct_Z= 1;
				}
			}
			else if(!PCT_Get_Status(PCT3) == PCT_NO_TRIGGER)
			{
				delay_ms(10);
				if(!PCT_Get_Status(PCT3) == PCT_NO_TRIGGER)
				{
					Motion_Output_Carbin(MOTOR_3,PCT_NO,REV,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
					delay_ms(500);
					while(motor_info[MOTOR_3].finish == NO_FINISH)
					{
						delay_ms(10);
					}
					SysCurrSataus.HardStatus.UnitStatus.Pct_Z= 1;
					//Motion_Init(MOTOR_3,FWD,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
					//delay_ms(500);
					//while(motor_info[MOTOR_3].status != M_MOTION_FREE);
					//Motion_Init_Return(MOTOR_3,REV,Z_AXIS_PCT,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);					
					//delay_ms(500);
					//while(motor_info[MOTOR_3].status != M_MOTION_FREE);
				}
			}
		}break;
		case MOTOR_4:
		{
			if(PCT_Get_Status(PCT4) == PCT_TRIGGER)
			{
				delay_ms(10);
				if(PCT_Get_Status(PCT4) == PCT_TRIGGER)
				{
					Motion_Init(MOTOR_4,FWD,&MotorResetPrar[3],speed_up_table[MOTOR_4],speed_down_table[MOTOR_4]);
					delay_ms(500);
					while(motor_info[MOTOR_4].finish == NO_FINISH);
					Motion_Init_Return(MOTOR_4,REV,PCT_NO,&MotorResetPrar[3],speed_up_table[MOTOR_4],speed_down_table[MOTOR_4]);
					delay_ms(500);
					while(motor_info[MOTOR_4].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_Mix= 1;
				}
			}
			else if(PCT_Get_Status(PCT4) == PCT_NO_TRIGGER)
			{
				delay_ms(10);
				if(PCT_Get_Status(PCT4) == PCT_NO_TRIGGER)
				{
					Motion_Output_Carbin(MOTOR_4,PCT_NO,REV,&MotorResetPrar[3],speed_up_table[MOTOR_4],speed_down_table[MOTOR_4]);
					delay_ms(500);
					while(motor_info[MOTOR_4].finish == NO_FINISH);
					SysCurrSataus.HardStatus.UnitStatus.Pct_Mix= 1;
				}
			}
		}break;
		default:break;
	}
	
}
void Test_SW_Status(Package_Info* package_data)
{
	switch(package_data->command)
	{
		case 0x01:
		{
			OUTA_High;
			delay_ms((u16)(package_data->buff[0]<<8|package_data->buff[1]));
			OUTA_Low;
			printf("电磁阀A已动作");
		}break;
		case 0x02:
		{
			OUTB_High;
			delay_ms((u16)(package_data->buff[0]<<8|package_data->buff[1]));
			OUTB_Low;
			printf("电磁阀B已动作");
		}break;
		case 0x03:
		{
			OUTC_High;
			delay_ms((u16)(package_data->buff[0]<<8|package_data->buff[1]));
			OUTC_Low;
		}break;
		case 0x04:
			//Motor_Self_Lock(MOTOR_1);
			//Motor_Self_Lock(MOTOR_2);
			Motor_Self_Lock(MOTOR_3);
			break;
		case 0x05:
			if(package_data->buff[0] == 0x01)
				Motion_Init(MOTOR_1,REV,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
			else if(package_data->buff[0] == 0x02)
				Motion_Init(MOTOR_2,REV,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
			else if(package_data->buff[0] == 0x03)
				Motion_Init(MOTOR_3,REV,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
			else if(package_data->buff[0] == 0x04)
			{
				Motion_Init(MOTOR_1,REV,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
				Motion_Init(MOTOR_2,REV,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
				Motion_Init(MOTOR_3,REV,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
			}
			break;
		case 0x06:
			if(package_data->buff[0] == 0x01)
				Motion_Init(MOTOR_1,FWD,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
			else if(package_data->buff[0] == 0x02)
				Motion_Init(MOTOR_2,FWD,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
			else if(package_data->buff[0] == 0x03)
				Motion_Init(MOTOR_3,FWD,&MotorResetPrar[2],speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
			else if(package_data->buff[0] == 0x04)
			{
				Motion_Init(MOTOR_1,FWD,&MotorResetPrar[0],speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
				Motion_Init(MOTOR_2,FWD,&MotorResetPrar[1],speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
				//Motion_Init(MOTOR_3,FWD,&MotorResetPrar[2]);
			}
			break;
		case 0x07:   //复位动作
			switch(package_data->buff[0])
			{
				case 0x01:
				{
					Motor_Init_Motion(MOTOR_1,speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);  //X轴电机复位
				}break;
				case 0x02:
				{
					Motor_Init_Motion(MOTOR_2,speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);  //Y轴电机复位
				}break;
				case 0x03:
				{
					Motor_Init_Motion(MOTOR_3,speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);   //Z轴电机复位
				}break;
				default:break;
			}
			break;
		case 0x08:
			Motor_Disable(MOTOR_1);
			Motor_Disable(MOTOR_2);
			Motor_Disable(MOTOR_3);
			break;
		default:break;	
	}
}

u8 Can_Receiver_Analyze(Package_Info* data)
{
	//data = data;
	Test_SW_Status(data);
	return 0;
}
//测试EEPROM
void TestEeprom(void)
{
	u8 buf[100];
	AT24CXX_Write(0,"wojiaoguchenglong",sizeof("wojiaoguchenglong"));
	AT24CXX_Read(0, buf,17);
	printf("%s\r\n",buf);
}
//#define PI 3.14159265358
u16 Test_speed_up_table[100] = {0};
u16 Test_speed_down_table[100] = {0};
//创建加减速表
//入口参数:min_speed:最小频率，max_speed:最大频率，point_num:加/减速频率点数
void Create_Speed_Change_Table(u16 min_speed, u16 max_speed, u16 point_num)	
{
	u16 t;
	u8 i = 0;

	for(t = point_num; t <= 2*point_num; t++)
	{
 		Test_speed_up_table[t-point_num] = 1000000 / (min_speed + (max_speed - min_speed) / 2 * (cos(t*PI/point_num)+1));
	}
	for(i = 1; i <= 10; i++)
	{
		Test_speed_up_table[point_num+i] = Test_speed_up_table[point_num];
	}
	for(t = 0; t <= point_num; t++)
	{
		Test_speed_down_table[t] = 1000000 / (min_speed + (max_speed - min_speed) / 2 * (cos(t*PI/point_num)+1));
	}
	for(i = 1; i <= 10; i++)
	{
		Test_speed_down_table[point_num+i] = Test_speed_down_table[point_num];
	}
	for(i=0;i<30;i++)
	{
		printf("%d\r\n",Test_speed_up_table[i]);
		delay_ms(10);
	}
	printf("**********************************\n");
	for(i=0;i<30;i++)
	{
		printf("%d\r\n",Test_speed_down_table[i]);
		delay_ms(10);
	}
}
/***********************************************串口通信协议相关部分********************************************************************/
u8 G_UART1_Receive_Buff[UART1_RECEIVE_SIZE];
u16 G_UART1_Receive_Count = 0;
u8 G_UART1_Receive_OK = 0;
//----------------------------------------------//
//在长度为len的缓存s1中查找s2存储	//
//的内容，找到则返回s2内容在s1中//
//的地址，否则返回NULL				//
//s1: 被搜索的存储区的指针			//
//s2: 要在s1中查找的内容的指针		//
//返回值: 找到,地址；未找到,NULL	//
//----------------------------------------------//
u8 *find_start(u8 *s1, u8 *s2, u16 len)
{
	u8 s2_len = 0;
	u16 i = 0;

	s2_len = sizeof(s2);

	if(len <= s2_len)	return NULL;		//检查参数有效性

	for(i = 0; i < (len - s2_len); i++)
	{
		if(*(s1+i) == *s2)						//如果在s1数组中找到了s2数组中的第一个值
		{
			if(memcmp(s1+i, s2, s2_len) == 0)	//如果在s1数组中找到了s2数组中的所有值
			{
				return s1+i;
			}
		}
	}
	return NULL;
}
static char *local_address1 = "CCH";   //液面检测板1地址
u8 SendFrame[] = {'&','C','C','H',0X00,0X00,0X00};
//串口数据解析
u16 Analyze_USART_Receive_Data(u8 *data)
{
	//u16 res;
	if(*data == '&')  //祯头判断
	{
		if(memcmp(data+1, local_address1, 3) == 0)  	//地址判断
		{
			if(*(data+10) == '\n')
			{
				return (u16)(*(data+4)<<8|*(data+7));
			}
			else
				return 0XFF;
		}
		else
			return 0XFF;
	}
	else
		return 0XFF;
}
u16 GetUsartData(void)
{
	u16 Res = 0;
	if(G_UART1_Receive_OK == 1)
	{
		Res = Analyze_USART_Receive_Data(G_UART1_Receive_Buff);
		memset(G_UART1_Receive_Buff,0,USART_REC_LEN);
		G_UART1_Receive_OK = 0;
		G_UART1_Receive_Count = 0;
		return Res;
	}
	else
		return 0XFF;
}
//串口发送指令协议
void UsartSendCmd(u8 Cmd,u16 Datalen,u8 *databuf)
{
	//u8 i;
	u8 SendFrame[20];
	SendFrame[0] = '&';  //祯头
	SendFrame[1] = 'C';  //地址
	SendFrame[2] = 'C';
	SendFrame[3] = 'H';
	SendFrame[4] = Cmd;   //指令
	SendFrame[5] = (u8)((Datalen>>8)&0XFF);  //数据长度
	SendFrame[6] = (u8)(Datalen&0XFF);
	SendFrame[7] = *databuf;  //数据
	SendFrame[8] = 0x00;  	  //校验
	SendFrame[9] = '\n';
	Usart1_Send_Data(SendFrame,10);
}

#define MAX_INT_DIGITS  8
//读取浮点数
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)
{
  char *ptr = line + *char_counter;
  unsigned char c;

  // Grab first character and increment pointer. No spaces assumed in line.
  c = *ptr++;

  // Capture initial positive/minus character
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }

  // Extract number into fast integer. Track decimal in terms of exponent value.
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  // Drop overflow digits
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }

  // Return if no digits have been read.
  if (!ndigit) { return(false); };

  // Convert integer into floating point.
  float fval;
  fval = (float)intval;

  // Apply decimal. Should perform no more than two floating point multiplications for the
  // expected range of E0 to E-4.
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01;
      exp += 2;
    }
    if (exp < 0) {
      fval *= 0.1;
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    }
  }

  // Assign floating point value with correct sign.
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; // Set char_counter to next statement

  return(true);
}






























