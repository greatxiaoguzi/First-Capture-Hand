//调试记录:从原点到抓杯点后ComMotionFinishFlag未置位FALSE,因为没进去状态判断的那个if

#include "main_app.h"
#include "step_motor.h"
#include "MotionDispatch.h"
extern MOTOR_INFO motor_info[4];
extern u8 can_flag;
extern Package_Info unpack_data;

//测试光电开关状态:挡住的时候输出高电平，没档的时候输出低电平
void Test_Pct_Status(void)
{ 
	u8 PctStatus;
	PctStatus = (u8)PCT_1_STATUS<<7|(u8)PCT_2_STATUS<<6|(u8)PCT_3_STATUS<<5|(u8)PCT_4_STATUS<<4|
	(u8)PCT_5_STATUS<<3|(u8)PCT_6_STATUS<<2|(u8)PCT_7_STATUS<<1|(u8)PCT_8_STATUS;
	if((PctStatus&(1<<7))==0)
		printf("PCT1没挡住\r\n");
	else
		printf("          PCT1被挡住\r\n");
	
	if((PctStatus&(1<<6))==0)
		printf("PCT2没挡住\r\n");
	else
		printf("                    PCT2被挡住\r\n");
	
	if((PctStatus&(1<<5))==0)
		printf("PCT3没挡住\r\n");
	else
		printf("                              PCT3被挡住\r\n");
	
	if((PctStatus&(1<<4))==0x10)
		printf("PCT4没挡住\r\n");
	else
		printf("                                        PCT4被挡住\r\n");
	
	if((PctStatus&(1<<3))==0)
		printf("PCT5没挡住\r\n");
	else
		printf("                                                  PCT5被挡住\r\n");
	
	if((PctStatus&(1<<2))==0)
		printf("PCT6没挡住\r\n");
	else
		printf("                                                            PCT6被挡住\r\n");
	
	if((PctStatus&(1<<1))==0)
		printf("PCT7没挡住\r\n");
	else
		printf("                                                                      PCT7被挡住\r\n");
	
	if((PctStatus&(1<<0))==0x01)
		printf("PCT8没挡住\r\n");
	else
		printf("                                                                                PCT8被挡住\r\n");
	printf("\n");
}
int main(void)
     {
	u8 i = 0;
	u8 buff[5];  
 
	Init();  
	//Create_Speed_Change_Table(400,9600,30);
	CombineReset();
	SysInit();
	memcpy(buff, "&ABC\n", 5);
	delay_ms(100);
	Usart1_Send_Data(buff, 5);
	delay_ms(100);
	Usart1_Send_Data(buff, 5);
	delay_ms(100);
	printf("%d\r\n",sizeof(CapturePara_TpyeDef));
	//EEpromTest();
	while(1);
	{
		/*if(can_flag == 0)
		{
			can_flag = 0xff;
			printf("接收到CAN数据");
			Can_Receiver_Analyze(&unpack_data);
			//Test_SW_Status(&unpack_data);
		}

		for(i = 0; i < MOTOR_MAX; i++)
		{
			if(motor_info[i].status == M_MOTION_FREE)	//电机处于自由状态
			{
				
			}
		}*/
		//Test_Pct_Status();
		//delay_ms(700);
		//Test_SW_Status(&unpack_data);
		MotionHandle();
	}


}	



















