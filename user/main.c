//���Լ�¼:��ԭ�㵽ץ�����ComMotionFinishFlagδ��λFALSE,��Ϊû��ȥ״̬�жϵ��Ǹ�if

#include "main_app.h"
#include "step_motor.h"
#include "MotionDispatch.h"
extern MOTOR_INFO motor_info[4];
extern u8 can_flag;
extern Package_Info unpack_data;

//���Թ�翪��״̬:��ס��ʱ������ߵ�ƽ��û����ʱ������͵�ƽ
void Test_Pct_Status(void)
{ 
	u8 PctStatus;
	PctStatus = (u8)PCT_1_STATUS<<7|(u8)PCT_2_STATUS<<6|(u8)PCT_3_STATUS<<5|(u8)PCT_4_STATUS<<4|
	(u8)PCT_5_STATUS<<3|(u8)PCT_6_STATUS<<2|(u8)PCT_7_STATUS<<1|(u8)PCT_8_STATUS;
	if((PctStatus&(1<<7))==0)
		printf("PCT1û��ס\r\n");
	else
		printf("          PCT1����ס\r\n");
	
	if((PctStatus&(1<<6))==0)
		printf("PCT2û��ס\r\n");
	else
		printf("                    PCT2����ס\r\n");
	
	if((PctStatus&(1<<5))==0)
		printf("PCT3û��ס\r\n");
	else
		printf("                              PCT3����ס\r\n");
	
	if((PctStatus&(1<<4))==0x10)
		printf("PCT4û��ס\r\n");
	else
		printf("                                        PCT4����ס\r\n");
	
	if((PctStatus&(1<<3))==0)
		printf("PCT5û��ס\r\n");
	else
		printf("                                                  PCT5����ס\r\n");
	
	if((PctStatus&(1<<2))==0)
		printf("PCT6û��ס\r\n");
	else
		printf("                                                            PCT6����ס\r\n");
	
	if((PctStatus&(1<<1))==0)
		printf("PCT7û��ס\r\n");
	else
		printf("                                                                      PCT7����ס\r\n");
	
	if((PctStatus&(1<<0))==0x01)
		printf("PCT8û��ס\r\n");
	else
		printf("                                                                                PCT8����ס\r\n");
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
			printf("���յ�CAN����");
			Can_Receiver_Analyze(&unpack_data);
			//Test_SW_Status(&unpack_data);
		}

		for(i = 0; i < MOTOR_MAX; i++)
		{
			if(motor_info[i].status == M_MOTION_FREE)	//�����������״̬
			{
				
			}
		}*/
		//Test_Pct_Status();
		//delay_ms(700);
		//Test_SW_Status(&unpack_data);
		MotionHandle();
	}


}	



















