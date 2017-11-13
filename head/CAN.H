#ifndef __CAN_H_
#define __CAN_H_

#include <string.h>
#include "sys.h"	    
	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    

typedef struct
{
	u8 src_id;				//源节点ID
	u8 dest_id;				//目标节点ID
	u8 pack_status;			//包状态标志
	u8 command;				//MCU可执行命令
	u8 seg_polo;			//分段标志
	u8 seg_num;				//分段编号
	u8 dlc;					//数据域长度
	u8 buff[8];				//数据
	u8 ide;					//帧类型
	u8 rtr;					//数据包类型
	u16 upgrade_pack_num;	//升级包包编号
}Package_Info;

void CAN_Mode_Init(void);
u8 CAN_Get_Local_ID(void);
u8 CAN_Send_Msg(Package_Info* package_data);

#endif

