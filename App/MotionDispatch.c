#include "MotionDispatch.h"
#include "main_app.h"
#include "step_motor.h"
#include "em_sw.h"
#include "can.h"
#include "string.h"
#include "math.h"
#include "pct_app.h"
#include "Config.h"
#include "crc16.h"
//外部调用参数
extern Package_Info unpack_data;
extern u8 local_id;
extern u8 can_flag;
extern u16 speed_up_table[MOTOR_MAX][100];
extern u16 speed_down_table[MOTOR_MAX][100];
extern u16 mindist_up_table[MOTOR_MAX][100];
extern u16 mindist_down_table[MOTOR_MAX][100];
extern MOTOR_INFO motor_info[4];   		//四通道电机参数数据结构
//内部参数
u32 Mix_Circle_Cnt = 0;   					//混匀盘转动圈数
u32 Mix_Max_Circle = 0;
u32 CurrCaptureCupNum = 1; 				//已经抓取的杯子的数量
u32 ThrowCupNum = 0;  					//扔掉的杯子
u32 MixRunMaxPulse = 0;  				//混匀器最大混匀运行的脉冲数
CaptureCupStatus_TypeDef CaptureCupExecuteStatus = NO_EXECUTE; 					//手指当前的抓杯状态
CaptureFinger_Status_TypeDef CaptureFingerStatus = CAPTURE_FINGER_IDLE;//抓杯手指当前的运行状态

CapturePara_TpyeDef CapturePara;		//关机之后到有杯子的坐标点(在开机参数配置中从EEPROM中获取)
CupDishCalibrationProperty_TypeDef CupDishCalibrationProperty; //杯盘校准后的属性

//MOTOR_COMPLETION_STATUS FingerEcttmStatus = NO_FINISH;   //抓杯电磁铁运行完成状态标志
MOTOR_COMPLETION_STATUS CabinEcttmStatus = NO_FINISH;  //抽屉打开后完成状态标志
MOTOR_COMPLETION_STATUS SendOtherStatusInfoFlag = NO_FINISH;   //发送其余状态信息完成标志

//机器绝对零点和相对零点
Machine_Position_TypeDef Mchine_Position =
{
	.Machine_Absolute_Origin_X 		= 0,
	.Machine_Absolute_Origin_Y 		= 0,
	.Machine_Absolute_Origin_Z 		= 0,
	.Machine_Relative_Origin_X 		= 0,
	.Machine_Relative_Oringin_Y 	= 0,
	.Machine_Relative_Oringin_Z 	= 0,
}; 
CaptureFinger_Position_TypeDef FingerPositionData;            //抓杯手指坐标状态(X,Y,Z的当前和目标坐标值)
//机器工作位参数表，初始默认值
Work_Para_TypeDef Work_Para;  			//工作位参数
WorkParaCombine_TypeDef WorkParaCombine = 
{
	{
		.CupDish1_LeftBottom_Position 		= 4720,
		.CupDish1_LeftTop_Position 			= 4690,
		.CupDish1_RightBottom_Position 		= 98,
		.CupDish1_RightTop_Position 		= 98,
		
		.CupDish2_LeftBottom_Position 		= 4720,
		.CupDish2_LeftTop_Position 			= 4720,
		.CupDish2_RightBottom_Position 		= 114,
		.CupDish2_RightTop_Position 		= 114,
		
		.CupDish3_LeftBottom_Position 		= 4730,
		.CupDish3_LeftTop_Position 			= 4739,
		.CupDish3_RightBottom_Position 		= 111,
		.CupDish3_RightTop_Position 		= 124,
		
		.Hatch_In_Position 					= 6745,
		.Hatch_Out_Position 				= 6599,
		.Mix_Position 						= 6402,
		.Waste1_Position 					= 5924,
		.Waste2_Position 					= 5896,
	},
	{
		.CupDish1_LeftBottom_Position 		= 117,
		.CupDish1_LeftTop_Position 			= 2586,
		.CupDish1_RightBottom_Position 		= 132,
		.CupDish1_RightTop_Position 		= 2608,
		
		.CupDish2_LeftBottom_Position 		= 3419,
		.CupDish2_LeftTop_Position 			= 5887,
		.CupDish2_RightBottom_Position 		= 3424,
		.CupDish2_RightTop_Position 		= 5903,
		
		.CupDish3_LeftBottom_Position 		= 6608,
		.CupDish3_LeftTop_Position 			= 9169,
		.CupDish3_RightBottom_Position 		= 6714,
		.CupDish3_RightTop_Position 		= 9199,
		
		.Hatch_In_Position 					= 9160,
		.Hatch_Out_Position 				= 8713,
		.Mix_Position 						= 6088,
		.Waste1_Position 					= 4201,
		.Waste2_Position 					= 2705,
	},
	{
		.CupDish_Position 					= 3139,
		.Hatch_Position						= 3171,
		.Mix_Position 						= 3041,
		.Waste_Position 					= 3637,
	}
};
//调试用参数
#if DEBUG_MODE == 1
	DebugPara_TypeDef Debug_CorrVal =
	{
		.X_OffsetVal 						= 0,
		.Y_OffsetVal 						= 0,
		.Z_OffsetVal 						= 0,
		.MixRunningTime 					= 0,
	}; 
#endif
const FaultTypeCmdTab_TypeDef FaultCmdTab[FAULT_TYPE_MAX_NUM] = 
{
	{REC_ACK,							TEXT_REC_ACK},
	{REC_NACK,							TEXT_REC_NACK},
	{FIN_ACK,							TEXT_FIN_ACK},
	{VERSION_INQUIE,					TEXT_VERSION_INQUIE},
	{FAULT_RESET_X_AXIS,				TEXT_FAULT_RESET_X_AXIS},
	{FAULT_RESET_Y_AXIS,				TEXT_FAULT_RESET_Y_AXIS},
	{FAULT_RESET_Z_AXIS,				TEXT_FAULT_RESET_Z_AXIS},
	{FAULT_RESET_MIX,					TEXT_FAULT_RESET_MIX,},
	{FAULT_ORIGIN_TO_CAPTURECUP,		TEXT_FAULT_ORIGIN_TO_CAPTURECUP},
	{FAULT_ORIGIN_TO_HATCHIN,			TEXT_FAULT_ORIGIN_TO_HATCHIN},
	{FAULT_ORIGIN_TO_HATCHOUT,			TEXT_FAULT_ORIGIN_TO_HATCHOUT},
	
	{FAULT_CUP_TO_HATCHIN,				TEXT_FAULT_CUP_TO_HATCHIN},
	{FAULT_CUP_TO_HATCHOUT,				TEXT_FAULT_CUP_TO_HATCHOUT},
	{FAULT_HATCH_TO_CUP,				TEXT_FAULT_HATCH_TO_CUP},
	{FAULT_HATCH_TO_MIX,				TEXT_FAULT_HATCH_TO_MIX},
	{FAULT_MIX_TO_HATCHIN,				TEXT_FAULT_MIX_TO_HATCHIN},
	{FAULT_MIX_TO_HATCHOUT,				TEXT_FAULT_MIX_TO_HATCHOUT},
	{FAULT_HATCH_TO_WASTE,				TEXT_FAULT_HATCH_TO_WASTE},
	{FAULT_WASTE_TO_HATCHIN,			TEXT_FAULT_WASTE_TO_HATCHIN},
	{FAULT_WASTE_TO_HATCHOUT,			TEXT_FAULT_WASTE_TO_HATCHOUT},
	{FAULT_ENABLE_CABIN,				TEXT_FAULT_ENABLE_CABIN},
	{FAULT_MIX_RUNNING,					TEXT_FAULT_MIX_RUNNING},
	{FAULT_X_TO_CAPTURE,				TEXT_FAULT_X_TO_CAPTURE},
	{FAULT_X_TO_HATCHIN,				TEXT_FAULT_X_TO_HATCHIN},
	{FAULT_X_TO_HATCHOUT,				TEXT_FAULT_X_TO_HATCHOUT},
	{FAULT_X_TO_MIX,					TEXT_FAULT_X_TO_MIX},
	{FAULT_X_TO_WASTE1,					TEXT_FAULT_X_TO_WASTE1},
	{FAULT_X_TO_WASTE2,					TEXT_FAULT_X_TO_WASTE2},
	{FAULT_Y_TO_CAPTURE,				TEXT_FAULT_Y_TO_CAPTURE},
	{FAULT_Y_TO_HATCHIN,				TEXT_FAULT_Y_TO_HATCHIN},
	{FAULT_Y_TO_HATCHOUT,				TEXT_FAULT_Y_TO_HATCHOUT},
	{FAULT_Y_TO_MIX,					TEXT_FAULT_Y_TO_MIX},
	{FAULT_Y_TO_WASTE1,					TEXT_FAULT_Y_TO_WASTE1},
	{FAULT_Y_TO_WASTE2,					TEXT_FAULT_Y_TO_WASTE2},
	{FAULT_Z_TO_CUP,					TEXT_FAULT_Z_TO_CUP},
	{FAULT_Z_TO_HATCH,					TEXT_FAULT_Z_TO_HATCH},
	{FAULT_Z_TO_MIX,					TEXT_FAULT_Z_TO_MIX},
	{FAULT_Z_TO_WASTE,					TEXT_FAULT_Z_TO_WASTE},
	{FAULT_FINGER_OPEN,					TEXT_FAULT_FINGER_OPEN}
};
SysStatusInquiry_TypeDef SysCurrSataus;          		//硬件状态和当前运行参数
FaultLevel_TypeDef CurrFaultLevel = LEVEL_LOWEST;   	//系统故障级别
//电机底层参数配置
DispatchUnitExecuteFun_TypeDef DispatchUnitFun =
{
	X_MotorParaConfig,
	Y_MotorParaConfig,
	Z_MotorParaConfig,
	MixMotor_ParaConfig,
};
CombineStage Combine_Stage;  						 	//组合动作完成阶段的列表
COM_ListTypeDef CurrComMotion = COM_NO_MOTION;  		//组合动作运行的当前动作
COM_ListTypeDef DebugComMotion = COM_NO_MOTION; 		//调试的时候用到的组合动作
Status_TypeDef ComMotionFinishFlag = FALSE;    			//组合动作运行是否完成标志
//动作组合函数组
XYZFingerMotion_TypeDef XYZFiner_Motion_Sel =
{
	FingerOpen,
	Z_To_WorkStation,
	XY_To_WorkStation,
	Mix_StartRun,
};
//组合动作调度函数组
CanCmdMotionDispatch_TypeDef CAN_Cmd_Combine_Motion[] =
{
	{CMD_COM_NULL, 					NULL,						COM_NO_MOTION},
	{CMD_VERSION_INQUIRE, 			Go_SysSoftwareVersionCheck,	COM_VERSION_INQUIRE},
	{CMD_REPORT_HARD_STAUS, 		Go_SendStatusInfo,			COM_REPORT_HARD_STAUS},
	{CMD_REPORT_CONFIG_PARA,		Go_SendConfigPara,			COM_REPORT_CONFIG_PARA},
	
	{CMD_COM_CURR_TO_ORIGIN,	    Go_Curr_To_Origin,			COM_CURR_TO_ORIGIN},
	{CMD_COM_CURR_TO_CAPTUREP_POS,	Go_Curr_To_CaptureCup,		COM_CURR_TO_CAPTURE},
	{CMD_COM_ORIGN_TO_HATCH_IN,		Go_Orign_To_HatchIn,		COM_ORIGN_TO_HATCHIN},
	{CMD_COM_ORIGN_TO_HATCH_OUT,	Go_Orign_To_HatchOut,		COM_ORIGN_TO_HATCHOUT},
	
	{CMD_COM_ORIGN_TO_MIX,			Go_Orign_To_Mix,			COM_ORIGN_TO_MIX},
	{CMD_COM_ORIGN_TO_WASTE1,		Go_Orign_To_Waste1,			COM_ORIGN_TO_WASTE1},
	{CMD_COM_ORIGN_TO_WASTE2,		Go_Orign_To_Waste2,			COM_ORIGN_TO_WASTE2},
	
	{CMD_COM_CUP_TO_HATCH_IN,		Go_Cup_To_HatchIn,			C0M_CUP_TO_HATCHIN},
	{CMD_COM_CUP_TO_HATCH_OUT,		Go_Cup_To_HatchOut,			C0M_CUP_TO_HATCHOUT},
	
	{CMD_COM_CUPDISH_TO_WASTE1,		Go_Cup_To_Wast1,			COM_CUPDISH_TO_WASTE1},
	{CMD_COM_CUPDISH_TO_WASTE2,		Go_Cup_To_Wast2,			COM_CUPDISH_TO_WASTE2},
	{CMD_COM_CUPDISH_TO_MIX,		Go_Cup_To_Mix,				COM_CUPDISH_TO_MIX},
	{CMD_COM_MIX_TO_WASTE1,			Go_Mix_To_Wast1,			COM_MIX_TO_WASTE1},
	{CMD_COM_MIX_TO_WASTE2,			Go_Mix_To_Wast2,			COM_MIX_TO_WASTE2},
	
	{CMD_COM_STARTMACHINE_THROW_CUP,Go_Orign_To_Waste_Throw_Cup,COM_STARTMACHINE_THROW_CUP},
	{CMD_COM_HATCH_TO_CUP,			Go_Hatch_To_CupDish,		C0M_HATCH_TO_CUP},
	{CMD_COM_HATCH_TO_MIX,			Go_Hatch_To_Mix,			C0M_HATCH_TO_MIX},
	{CMD_COM_MIX_TO_HATCH_IN,		Go_Mix_To_HatchIn,			C0M_MIX_TO_HATCHIN},
	{CMD_COM_MIX_TO_HATCH_OUT,		Go_Mix_To_HatchOut,			C0M_MIX_TO_HATCHOUT},
	{CMD_COM_HATCH_TO_WASTE,		Go_Hatch_To_Waste,			C0M_HATCH_TO_WASTE},
	{CMD_COM_WASTE_TO_HATCH_IN,		Go_Waste_To_HatchIn,		C0M_WASTE_TO_HATCHIN},
	{CMD_COM_WASTE_TO_HATCH_OUT,	Go_Waste_To_HatchOut,		C0M_WASTE_TO_HATCHOUT},
	{CMD_COM_OPEN_CABIN,			Go_Open_Cabin,				COM_OPEN_CABIN},
	{CMD_COM_CLOSE_CABIN,			Go_Lock_Cabin,				COM_CLOSE_CABIN},
	{CMD_COM_MIX_RUNNING,			Go_Mix_Running,				COM_MIX_RUNNING},
	{CMD_COM_UNLOAD_CUP,			Go_Unload_Cup,				COM_UNLOAD_CUP},
};
ExecuteCmd_TypeDf ExecuteCmd;
//工作位参数加载，从EEPROM结构体融合并且加载到系统运行结构体中
void WorkPositionParaConfig(void)
{
	WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish1_LeftTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_LEFTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_RIGHTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH1_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.X_Work_Para.CupDish2_LeftBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish2_LeftTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_LEFTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_RIGHTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH2_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.X_Work_Para.CupDish3_LeftBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish3_LeftTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_LEFTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_RIGHTTOP_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_CUPDISH3_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.X_Work_Para.Hatch_In_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_HATCH_IN_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_HATCH_IN_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.Hatch_Out_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_HATCH_OUT_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_HATCH_OUT_POSITION].Adjust_Value;
	
	WorkParaCombine.X_Work_Para.Mix_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_MIX_POSITIN].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_MIX_POSITIN].Adjust_Value;
	WorkParaCombine.X_Work_Para.Waste1_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_WASTE_1_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_WASTE_1_POSITION].Adjust_Value;
	WorkParaCombine.X_Work_Para.Waste1_Position = Work_Para.X_Motor_Para_Info.Config_Para_Info[X_WASTE_2_POSITION].Normal_Value + Work_Para.X_Motor_Para_Info.Config_Para_Info[X_WASTE_2_POSITION].Adjust_Value;

	/*********************************************************************************************************************************************************************************************************************************************/
	WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish1_LeftTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_LEFTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish1_RightTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_RIGHTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH1_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish2_LeftTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_LEFTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish2_RightTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_RIGHTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH2_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_LEFTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_LEFTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish3_LeftTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_LEFTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_LEFTTOP_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_RIGHTBOTTOM_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_RIGHTBOTTOM_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.CupDish3_RightTop_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_RIGHTTOP_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_CUPDISH3_RIGHTTOP_POSITION].Adjust_Value;
	
	WorkParaCombine.Y_Work_Para.Hatch_In_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_HATCH_IN_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_HATCH_IN_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.Hatch_Out_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_HATCH_OUT_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_HATCH_OUT_POSITION].Adjust_Value;
	
	WorkParaCombine.Y_Work_Para.Mix_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_MIX_POSITIN].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_MIX_POSITIN].Adjust_Value;
	WorkParaCombine.Y_Work_Para.Waste1_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_WASTE_1_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_WASTE_1_POSITION].Adjust_Value;
	WorkParaCombine.Y_Work_Para.Waste1_Position = Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_WASTE_2_POSITION].Normal_Value + Work_Para.Y_Motor_Para_Info.Config_Para_Info[Y_WASTE_2_POSITION].Adjust_Value;

	WorkParaCombine.Z_Work_Para.CupDish_Position = Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_CUPDISH_POSITION].Normal_Value + Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_CUPDISH_POSITION].Adjust_Value;
	WorkParaCombine.Z_Work_Para.Hatch_Position = Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_HATCH_POSITION].Normal_Value + Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_HATCH_POSITION].Adjust_Value;
	WorkParaCombine.Z_Work_Para.Mix_Position = Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_MIX_POSITION].Normal_Value + Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_MIX_POSITION].Adjust_Value;
	WorkParaCombine.Z_Work_Para.Waste_Position = Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_WASTE_POSITION].Normal_Value + Work_Para.Z_Motor_Para_Info.Config_Para_Info[Z_WASTE_POSITION].Adjust_Value;
}
/****************************
	轴坐标系和手指坐标初始化配置(单位为Step)    1600*1/PI*BELT_DIA
	手指初始坐标为机械相对原点坐标
	开机系统参数初始化配置
	硬件状态
	系统参数
*****************************/
void SysParaInitConfig(void)
{
	u8 PCT_CurrStatus;
	u16 crc_Value;
	EM_SW_STATUS EM_SW_Status;
	u8 buf[1];
	u16 DataLen = 0;
	PCT_CurrStatus = GetAllPctStatus();
	EM_SW_Status = ((u8)EM_SW_Get_Status(EM_SW_CH_H)<<7|(u8)EM_SW_Get_Status(EM_SW_CH_G)<<6|
					(u8)EM_SW_Get_Status(EM_SW_CH_F)<<5|(u8)EM_SW_Get_Status(EM_SW_CH_E)<<4|
					(u8)EM_SW_Get_Status(EM_SW_CH_D)<<3|(u8)EM_SW_Get_Status(EM_SW_CH_C)<<2|
					(u8)EM_SW_Get_Status(EM_SW_CH_B)<<1|(u8)EM_SW_Get_Status(EM_SW_CH_A)<<0);
	DataLen = sizeof(CapturePara);
//	AT24CXX_Read(EEPROM_FOR_CUP_DATA_START_ADDR,&CapturePara.CupDishNum,DataLen);  	//读出之前保存在EEPROM中的杯位数据
//	CurrCaptureCupNum = CapturePara.CurrentCaptureNum;         					//当前抓取的杯子数目
//	CalMixMaxPoint(Work_Para.Mix_Motor_Para_Info.Config_Para_Info[MIX_TIME]);
	CapturePara.CurrentCaptureNum = CurrCaptureCupNum;
	CalMixMaxPoint(5);
	SysCurrSataus.HardStatus.AllStatus = (u16)(EM_SW_Status<<8 | PCT_CurrStatus);
	SysCurrSataus.ExecutePara.CurrCaptureCupNum = CurrCaptureCupNum,
	SysCurrSataus.ExecutePara.CurrThrowCupNum = CUP_TOTAL_NUM - CurrCaptureCupNum - 2;
	SysCurrSataus.ExecutePara.CurrExecuteMotion = COM_NO_MOTION,
	Mchine_Position.Machine_Relative_Origin_X = Mchine_Position.Machine_Absolute_Origin_X - X_PCT_ZERO_RUN_PULSE;
	Mchine_Position.Machine_Relative_Oringin_Y = Mchine_Position.Machine_Absolute_Origin_Y - Y_PCT_ZERO_RUN_PULSE;
	Mchine_Position.Machine_Relative_Oringin_Z = Mchine_Position.Machine_Absolute_Origin_Z - Z_PCT_ZERO_RUN_PULSE;

	FingerPositionData.Curr_Position.Fingrer_Position_X = Mchine_Position.Machine_Relative_Origin_X;	//Mchine_Position.Machine_Relative_Origin_X;
	FingerPositionData.Curr_Position.Fingrer_Position_Y = Mchine_Position.Machine_Relative_Oringin_Y;	//Mchine_Position.Machine_Relative_Oringin_Y;
	FingerPositionData.Curr_Position.Fingrer_Position_Z = Mchine_Position.Machine_Relative_Oringin_Z;	// Mchine_Position.Machine_Relative_Oringin_Z;
	if(CONFIG_PARA_OPTION == 1)
	{
		DataLen = sizeof(Work_Para_TypeDef);
		AT24CXX_Read(EEPROM_FOR_WORK_PARA_START_ADDR,(u8*)&Work_Para.Para_Version[0],DataLen);
		crc_Value = Work_Para.crc_value;
		if(CRC16_1((u8*)&Work_Para.Para_Version[0],sizeof(Work_Para_TypeDef)-2) == crc_Value)
			;
		else
			return;
	}
	else if(CONFIG_PARA_OPTION == 0)
	{
		DataLen = sizeof(WorkParaCombine_TypeDef);
		AT24CXX_Read(EEPROM_FOR_WORK_PARA_START_ADDR,(u8*)&WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position,DataLen);
		crc_Value = Work_Para.crc_value;
		if(CRC16_1((u8*)&WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position,sizeof(WorkParaCombine_TypeDef)-2) == crc_Value)
			;
		else
			return;
	}
//	WorkPositionParaConfig();    			//读取机器坐标校准的数据
//	SaveCupDataToEEprom();
}
/********************************************************
	功能：		根据校准的参数来配置三个杯盘相对坐标系的实际的杯间距值
	输入参数：	无
	返回值：		无
********************************************************/
void CupDishPropertyConfig(void)
{
	CupDishCalibrationProperty.Cup1_X_Actual_Space = abs(WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position - WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position)/13;
	CupDishCalibrationProperty.Cup1_Y_Actual_Space = abs(WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position - WorkParaCombine.Y_Work_Para.CupDish1_RightTop_Position)/7;
	
	CupDishCalibrationProperty.Cup2_X_Actual_Space = abs(WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position - WorkParaCombine.X_Work_Para.CupDish2_LeftBottom_Position)/13;
	CupDishCalibrationProperty.Cup2_Y_Actual_Space = abs(WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position - WorkParaCombine.Y_Work_Para.CupDish2_RightTop_Position)/7;
	
	CupDishCalibrationProperty.Cup3_X_Actual_Space = abs(WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position - WorkParaCombine.X_Work_Para.CupDish3_LeftBottom_Position)/13;
	CupDishCalibrationProperty.Cup3_Y_Actual_Space = abs(WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position - WorkParaCombine.Y_Work_Para.CupDish3_RightTop_Position)/7;
}
/********************************************************
	功能：		根据当前杯盘号得到杯子在杯盘中的偏移位置
	输入参数：	杯盘号，X方向上的偏移值，Y方向上的偏移值
	返回值：		无
********************************************************/
void GetCaptureCupOffset(u16 CurrCaptureCupNum,u16 *XOffsetCoord,u16 *YOffsetCoord)
{
	u16 Xoffset,Yoffset;
	if(CurrCaptureCupNum <= 112)
	{
		 if(((CUP_TOTAL_NUM - CurrCaptureCupNum - 224)%14) != 0)
		 {
			Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 224)/14;
			Xoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 224)%14;
		 }
		 else if(((CUP_TOTAL_NUM - CurrCaptureCupNum - 224)%14) == 0)
		 {
			if(CurrCaptureCupNum == 112)
			{
				Xoffset = 0;
				Yoffset = 0;
			}
			else
			{
				Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 224)/14;
				Xoffset = 0;
			}
		 }
	}
	else if(CurrCaptureCupNum>112 && CurrCaptureCupNum<=224)
	{
		if(((CUP_TOTAL_NUM - CurrCaptureCupNum - 112)%14) != 0)
		 {
			Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 112)/14;
			Xoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 112)%14;
		 }
		 else if(((CUP_TOTAL_NUM - CurrCaptureCupNum - 112)%14) == 0)
		 {
			if(CurrCaptureCupNum == 224)
			{
				Xoffset = 0;
				Yoffset = 0;
			}
			else
			{
				Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum - 112)/14;
				Xoffset = 0;
			}
		 }
	}
	else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
	{
		if(((CUP_TOTAL_NUM - CurrCaptureCupNum)%14) != 0)
		 {
			Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum)/14;
			Xoffset =  (CUP_TOTAL_NUM - CurrCaptureCupNum)%14;
		 }
		 else if(((CUP_TOTAL_NUM - CurrCaptureCupNum)%14) == 0)
		 {
			if(CurrCaptureCupNum == 336)
			{
				Xoffset = 0;
				Yoffset = 0;
			}
			else
			{
				Yoffset = (CUP_TOTAL_NUM - CurrCaptureCupNum)/14;
				Xoffset =  0;
			}
		 }
	}
	else
	{
		Xoffset = 0;
		Yoffset = 0;
	}
	*XOffsetCoord = Xoffset;
	*YOffsetCoord = Yoffset;
}
/********************************************************
	功能：		根据当前杯盘号得到杯子的具体的脉冲数
	输入参数：	杯盘号，X方向上的偏移值，Y方向上的偏移值，转换后的X方向上的脉冲数，转换后的Y方向上的脉冲数
	返回值：		无
********************************************************/
void GetCaptureCupPosition(u8 CupNum,u16 X_Offset,u16 Y_Offset,u32 *X_Offset_Posiotion,u32 *Y_Offset_Position)
{
	float X_Loction_Space = 0;  //X方向上偏移起始点之间的间距
	float Y_Loction_Space = 0;  //Y方向上偏移起始点之间的间距
	static u8 Pre_X_Offset = 0XFF;
	static u8 Pre_Y_Offset = 0XFF;
	switch(CupNum)
	{   
		case 1:  	//如果是杯盘1
		{
			if(abs(WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position) >= 25)
			{
				X_Loction_Space = abs(WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position)/7;
			}
			if(WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position <= WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position + (7-Y_Offset)*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup1_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position + X_Offset*CupDishCalibrationProperty.Cup1_X_Actual_Space);
			}
			else if(WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position <= WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position + Y_Offset*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup1_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position + X_Offset*CupDishCalibrationProperty.Cup1_X_Actual_Space);
			}
			if(abs(WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position) >= 65)
			{
				Y_Loction_Space = abs(WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position)/13;
			}
			if(WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position + (13-X_Offset)*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup1_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup1_Y_Actual_Space);
			}
			else if(WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position + X_Offset*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup1_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup1_Y_Actual_Space);
			}
		}break;
		case 2:		//如果是杯盘2
		{
			if(abs(WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position) >= 25)
			{
				X_Loction_Space = abs(WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position)/7;
			}
			if(WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position <= WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position + (7-Y_Offset)*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup2_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position + X_Offset*CupDishCalibrationProperty.Cup2_X_Actual_Space);
			}
			else if(WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position <= WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position + Y_Offset*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup2_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position + X_Offset*CupDishCalibrationProperty.Cup2_X_Actual_Space);
			}
			if(abs(WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position) >= 65)
			{
				Y_Loction_Space = abs(WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position)/13;
			}
			if(WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position + (13-X_Offset)*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup2_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup2_Y_Actual_Space);
			}
			else if(WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position + X_Offset*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup2_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup2_Y_Actual_Space);
			}
		}break;
		case 3:		//如果是杯盘3
		{
			if(abs(WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position) >= 25)
			{
				X_Loction_Space = abs(WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position - WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position)/7;
			}
			if(WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position <= WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position + (7-Y_Offset)*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup3_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position + X_Offset*CupDishCalibrationProperty.Cup3_X_Actual_Space);
			}
			else if(WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position <= WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position)
			{
				if(X_Offset != Pre_X_Offset)
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position + Y_Offset*X_Loction_Space + X_Offset*CupDishCalibrationProperty.Cup3_X_Actual_Space);
				else
					*X_Offset_Posiotion = (u32)(WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position + X_Offset*CupDishCalibrationProperty.Cup3_X_Actual_Space);
			}
			if(abs(WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position) >= 65)
			{
				Y_Loction_Space = abs(WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position - WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position)/13;
			}
			if(WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position + (13-X_Offset)*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup3_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup3_Y_Actual_Space);
			}
			else if(WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position <= WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position)
			{
				if(Y_Offset != Pre_Y_Offset)
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position + X_Offset*Y_Loction_Space + Y_Offset*CupDishCalibrationProperty.Cup3_Y_Actual_Space);
				else
					*Y_Offset_Position = (u32)(WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position + Y_Offset*CupDishCalibrationProperty.Cup3_Y_Actual_Space);
			}
		}break;
		default:break;
	}
	if(X_Offset != Pre_X_Offset)
		Pre_X_Offset = X_Offset;
	if(Y_Offset != Pre_Y_Offset)
		Pre_Y_Offset = Y_Offset;
}
/********************************************************
	功能：		保存抓杯参数到EEPROM中
	输入参数：	无
	返回值：		无
********************************************************/
void SaveCupDataToEEprom(void)
{
	u8 buf[1];
	u16 DataLen = 0;
	u16 X_Offset = 0;
	u16 Y_Offset = 0;
	u32 X_Offset_Position;
	u32 Y_Offset_Position;
	DataLen = sizeof(CapturePara_TpyeDef);
	if(CurrCaptureCupNum>=0 && CurrCaptureCupNum<=112)
		CapturePara.CupDishNum = 3;
	else if(CurrCaptureCupNum>112 && CurrCaptureCupNum<=224)
		CapturePara.CupDishNum = 2;
	else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
		CapturePara.CupDishNum = 1;
	CapturePara.CurrentCaptureNum = CurrCaptureCupNum;
	GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
	
	switch(CapturePara.CupDishNum)
	{
		case 1:   //第一个杯盘
		{
			GetCaptureCupPosition(1,X_Offset,Y_Offset,&X_Offset_Position,&Y_Offset_Position);
		}break;
		case 2:   //第二个杯盘
		{
			GetCaptureCupPosition(2,X_Offset,Y_Offset,&X_Offset_Position,&Y_Offset_Position);
		}break;
		case 3:   //第三个杯盘
		{
			GetCaptureCupPosition(3,X_Offset,Y_Offset,&X_Offset_Position,&Y_Offset_Position);
		}break;
		default:break;
	}
	CapturePara.X_Position = X_Offset_Position;
	CapturePara.Y_Position = Y_Offset_Position;
	AT24CXX_Write(EEPROM_FOR_CUP_DATA_START_ADDR,&CapturePara.CupDishNum,DataLen);
}
/********************************************************
	功能：		保存机器工作位参数到EEPROM中
	输入参数：	无
	返回值：		无
********************************************************/
void SaveMachhineWorkStationToEeprom(void)
{
	u16 DataLen = 0;
	u16 crc_Value;
	if(CONFIG_PARA_OPTION == 1)
	{
		DataLen = sizeof(Work_Para_TypeDef);
		crc_Value = CRC16_1((u8*)&Work_Para.Para_Version[0],DataLen-2);
		Work_Para.crc_value = crc_Value;
		AT24CXX_Write(EEPROM_FOR_WORK_PARA_START_ADDR,(u8*)&Work_Para.Para_Version[0],DataLen);
	}
	else if(CONFIG_PARA_OPTION == 0)
	{
		DataLen = sizeof(WorkParaCombine_TypeDef);
		crc_Value = CRC16_1((u8*)&WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position,DataLen-2);
		WorkParaCombine.crc_value = crc_Value;
		AT24CXX_Write(EEPROM_FOR_WORK_PARA_START_ADDR,(u8*)&WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position,DataLen);
	}
}
/********************************************************
	功能：		上报硬件状态
	输入参数：	无
	返回值：		无
********************************************************/
void ReportHardwaretatus(void)
{
	u8 PCT_CurrStatus;
	EM_SW_STATUS EM_SW_Status;
	Package_Info package_data;
	memset(package_data.buff,0,8);
	PCT_CurrStatus = GetAllPctStatus();					//得到当前所有的光电开关的状态信息
	EM_SW_Status = ((u8)EM_SW_Get_Status(EM_SW_CH_H)<<7|(u8)EM_SW_Get_Status(EM_SW_CH_G)<<6|
					(u8)EM_SW_Get_Status(EM_SW_CH_F)<<5|(u8)EM_SW_Get_Status(EM_SW_CH_E)<<4|
					(u8)EM_SW_Get_Status(EM_SW_CH_D)<<3|(u8)EM_SW_Get_Status(EM_SW_CH_C)<<2|
					(u8)EM_SW_Get_Status(EM_SW_CH_B)<<1|(u8)EM_SW_Get_Status(EM_SW_CH_A)<<0);
	SysCurrSataus.HardStatus.AllStatus = (u16)(EM_SW_Status<<8 | PCT_CurrStatus);

	SysCurrSataus.ExecutePara.CurrCaptureCupNum = CurrCaptureCupNum;
	SysCurrSataus.ExecutePara.CurrThrowCupNum = CUP_TOTAL_NUM - CurrCaptureCupNum - 2;
	SysCurrSataus.ExecutePara.CurrExecuteMotion = CurrComMotion;

	package_data.command = REPORT_HARD_STAUS;
	package_data.src_id = local_id;
	package_data.dest_id = CAN_MAIN_MACHINE_ID;
	package_data.pack_status = 0;
	package_data.ide = CAN_ID_STD;
	package_data.rtr = CAN_RTR_DATA;
	package_data.upgrade_pack_num = 0;
	package_data.dlc = sizeof(SysStatusInquiry_TypeDef);
	memcpy(package_data.buff,&SysCurrSataus.HardStatus.AllStatus,package_data.dlc);
	CAN_Send_Msg(&package_data);
}
/*
StartMachineCupCoord_TpyeDef TestBuf;
//eeprom测试
void EEpromTest(void)
{
	u16 DataLen;
	StartMachineCupCoord_TpyeDef TempTest;
	DataLen = sizeof(StartMachineCupCoord_TpyeDef);
	TempTest.CupNum = 1;
	TempTest.X_Coord = 111.23;
	TempTest.Y_Coord = 222.34;
	AT24CXX_Write(TO_EEPROM_ADDR+100,&TempTest.CupNum, DataLen);
	AT24CXX_Read(TO_EEPROM_ADDR+100, &TestBuf.CupNum, DataLen);
	printf("%d - %d - %f - %f\r\n",DataLen,TestBuf.CupNum,TestBuf.X_Coord,TestBuf.Y_Coord);
	printf("输出完成");
}*/
//CAN发送故障状态机器版本信息，应答信息
/********************************************************
	功能：		通过CAN总线发送指令应答信息给上位机
	输入参数：	执行指令，应答码，应答类型
	返回值：		无
********************************************************/
void CanSendData(u8 DestId,CanMotionDispatchCmd_TypeDef ExecuteCmd,AnswerCode_TypeDef AnswerType)
{
	Package_Info package_data;
	package_data.src_id = local_id;
	package_data.dest_id = DestId;
	package_data.pack_status = 0;
	package_data.command = ExecuteCmd;
	package_data.ide = CAN_ID_STD;
	package_data.rtr = CAN_RTR_DATA;
	package_data.upgrade_pack_num = 0;
	for(u8 i=0;i<FAULT_TYPE_MAX_NUM;i++)
	{
		if(AnswerType == FaultCmdTab[i].Fault_Type)
		{
			strcpy((char*)package_data.buff,(char*)FaultCmdTab[i].FaultText);
			break;
		}
	}
	package_data.dlc = strlen((char*)package_data.buff);
	CAN_Send_Msg(&package_data);
}
//CAN发送工作位校准参数
void CanSendWorkStationInfo(u8 DestId,CanMotionDispatchCmd_TypeDef ExecuteCmd,u8 *buf,u16 buf_len)
{
	u16 stage;
	stage = buf_len / 8;
	for(u16 i=0;i<stage;i++)
	{
		u16 len;
		Package_Info package_data;
		package_data.src_id = local_id;
		package_data.dest_id = DestId;
		package_data.pack_status = 0;
		package_data.command = ExecuteCmd;
		package_data.ide = CAN_ID_STD;
		package_data.rtr = CAN_RTR_DATA;
		package_data.upgrade_pack_num = 0;
		package_data.seg_num = i;
		package_data.seg_polo = 0x10;
		memcpy(package_data.buff,buf,8);
		package_data.dlc = 8;
		CAN_Send_Msg(&package_data);
		buf += 8;
	}
}
#if DEBUG_MODE == 1
/********************************************************
	功能：		机器调试阶段的微调控制
	输入参数：	CAN数据帧，电机号，电机运行方向
	返回值：		无
********************************************************/
void Wispy_Adjust(Package_Info *unpack_data)
{
	s16 TargetPulse = 0;
	u8 Dir;
	DeviceUnit_TypeDef MotorSel;
	
	MotorSel = (DeviceUnit_TypeDef)unpack_data->buff[1];
	Dir = unpack_data->buff[2];
	TargetPulse = (u16)(unpack_data->buff[3]<<8|unpack_data->buff[4]);
	switch(MotorSel)
	{
		case DEVICE_UNIT_X:
		{
			motor_info[MotorSel].status = (MOTOR_MOTION_INFO)X_WISPY_ADJUST;
			if(Dir == 0)
			{
				motor_info[MotorSel].direction = REV;
				Debug_CorrVal.X_OffsetVal += TargetPulse;
			}
			else
			{
				motor_info[MotorSel].direction = FWD;
				Debug_CorrVal.X_OffsetVal -= TargetPulse;
			}

		}break;
		case DEVICE_UNIT_Y:
		{
			motor_info[MotorSel].status = (MOTOR_MOTION_INFO)Y_WISPY_ADJUST;
			if(Dir == 0)
			{
				motor_info[MotorSel].direction = FWD;
				Debug_CorrVal.Y_OffsetVal += TargetPulse;
			}
			else
			{ 
				motor_info[MotorSel].direction = REV;
				Debug_CorrVal.Y_OffsetVal -= TargetPulse;
			}
		}break;
		case DEVICE_UNIT_Z:
		{
			motor_info[MotorSel].status = (MOTOR_MOTION_INFO)Z_WISPY_ADJUST;
			if(Dir == 0)
			{
				motor_info[MotorSel].direction = FWD;
				Debug_CorrVal.Z_OffsetVal += TargetPulse;
			}
			else
			{
				motor_info[MotorSel].direction = REV;
				Debug_CorrVal.Z_OffsetVal -= TargetPulse;
			}
		}break;
		case DEVICE_UNIT_MIX:
		{
			Debug_CorrVal.MixRunningTime = unpack_data->buff[3];
		}break;
		default:break;
	}
	motor_info[MotorSel].acc_pulse = 0;
	motor_info[MotorSel].dec_pulse = 0;
	motor_info[MotorSel].const_pulse = TargetPulse;
	motor_info[MotorSel].multiple = 1;
	motor_info[MotorSel].speed_up_buff = &mindist_up_table[MotorSel][0];//
	motor_info[MotorSel].speed_down_buff = NULL;//
	motor_info[MotorSel].pct_number = PCT_NO;
	motor_info[MotorSel].pct_zero_run_pulse = 0;
	motor_info[MotorSel].pct_encoder_number = 0;

	Motor_Motion_Setup((MOTOR_CHIP_SELECT)MotorSel);
}
#endif
//定时器工作频率：1MHz
//计算混匀器混匀时间
//匀速时间=总时间-加速时间-减速时间
//加速阶段所有脉冲点所用时间+减速阶段的+匀速阶段的所有点数时间
//单位：秒
/********************************************************
	功能：		根据混匀时间计算出混匀器所需要的脉冲数
	输入参数：	混匀时间
	返回值：		无
********************************************************/
void CalMixMaxPoint(u8 AllTime)
{
	u8 i ;
	float ConstTime=0;
	float AccTime=0,DecTime=0;
	for(i=0;i<MIX_LARGE_ACC_PULSE/8;i++)   //加速时间
	{
		AccTime += (float)speed_up_table[DEVICE_UNIT_MIX][i]/1000000;
	}
	for(i=0;i<MIX_LARGE_DEC_PULSE/8;i++) //减速时间
	{
		DecTime += (float)speed_down_table[DEVICE_UNIT_MIX][i]/1000000;
	}
	ConstTime = AllTime - (AccTime + DecTime)*MIX_PER_FP_PULSE_NUM;
	if(ConstTime < 0)
	{
		MixRunMaxPulse = 0;
		return;
	}
	MixRunMaxPulse = (u32)(ConstTime/((float)speed_up_table[DEVICE_UNIT_MIX][MIX_LARGE_ACC_PULSE/8]/1000000));
	Mix_Max_Circle = (MixRunMaxPulse/1600)*3;
}
/********************************************************
	功能：		根据当前坐标和目标坐标计算出行程所需要的脉冲数
	输入参数：	抓杯手的坐标数据结构
	返回值：		每个轴方向上的脉冲数结构
********************************************************/
CaptureFingerPulse_TypeDef CalRouteTotalPulse(CaptureFinger_Position_TypeDef *Finger_Position)
{
	CaptureFingerPulse_TypeDef TempCaptureFingerPulse;
	
	if(Finger_Position->Target_Position.Fingrer_Position_X > Finger_Position->Curr_Position.Fingrer_Position_X)
		TempCaptureFingerPulse.X_Dir = REV;  //正向
	else if(Finger_Position->Target_Position.Fingrer_Position_X < Finger_Position->Curr_Position.Fingrer_Position_X)
		TempCaptureFingerPulse.X_Dir = FWD;  //反向
	else if(Finger_Position->Target_Position.Fingrer_Position_X == Finger_Position->Curr_Position.Fingrer_Position_X)
		TempCaptureFingerPulse.X_Dir = FWD;
	
	if(Finger_Position->Target_Position.Fingrer_Position_Y > Finger_Position->Curr_Position.Fingrer_Position_Y)
		TempCaptureFingerPulse.Y_Dir = FWD;  //正向
	else if(Finger_Position->Target_Position.Fingrer_Position_Y < Finger_Position->Curr_Position.Fingrer_Position_Y)
		TempCaptureFingerPulse.Y_Dir = REV;  //反向
	else if(Finger_Position->Target_Position.Fingrer_Position_Y == Finger_Position->Curr_Position.Fingrer_Position_Y)
		TempCaptureFingerPulse.Y_Dir = FWD;
	
	if(Finger_Position->Target_Position.Fingrer_Position_Z > Finger_Position->Curr_Position.Fingrer_Position_Z)
		TempCaptureFingerPulse.Z_Dir = FWD;	//正向`
	else if(Finger_Position->Target_Position.Fingrer_Position_Z < Finger_Position->Curr_Position.Fingrer_Position_Z)
		TempCaptureFingerPulse.Z_Dir = REV;	//反向
	else if(Finger_Position->Target_Position.Fingrer_Position_Z == Finger_Position->Curr_Position.Fingrer_Position_Z)
		TempCaptureFingerPulse.Z_Dir = FWD;
	
	TempCaptureFingerPulse.Finger_Pulse_X = abs(Finger_Position->Target_Position.Fingrer_Position_X - Finger_Position->Curr_Position.Fingrer_Position_X);  //(距离/周长)*一圈的脉冲数 = 圈数
	TempCaptureFingerPulse.Finger_Pulse_Y = abs(Finger_Position->Target_Position.Fingrer_Position_Y - Finger_Position->Curr_Position.Fingrer_Position_Y);
	TempCaptureFingerPulse.Finger_Pulse_Z = abs(Finger_Position->Target_Position.Fingrer_Position_Z - Finger_Position->Curr_Position.Fingrer_Position_Z);
	return TempCaptureFingerPulse;
}
/********************************************************
	功能：		抓杯手指电磁铁开关控制
	输入参数：	无
	返回值：		无
********************************************************/
static void Finger_Sw_Status(MotionCmd_TypeDef Status)
{
	if(Status == FINGER_OPEN)
		SW_FINGER_ENABLE;
	else if(Status == FINGER_CLOSE)
		SW_FINGER_DISABLE;
}
/********************************************************
	功能：		抽屉电磁铁开关控制
	输入参数：	状态使能量
	返回值：		无
********************************************************/
void CabinorEnable(MotionCmd_TypeDef ElectmStatus)
{
	if(ElectmStatus == CABIN_ELECTM_ENABLE)
	{
		SW_CABIN_ENABLE;
	}
	else if(ElectmStatus == CABIN_ELECTM_DISABLE)
	{
		SW_CABIN_DISABLE;
	}
}
/********************************************************
	功能：		手指进行自检是否工作正常
	输入参数：	无
	返回值：		无
********************************************************/
void FingerSelfCheck(PCT_NUMBER Pct_Num)
{
	u16 Cnt = 0;
	Finger_Sw_Status(FINGER_OPEN);
	while(Cnt < 100)
	{
		delay_ms(50);
		Cnt ++;
		if(PCT_Get_Status(Pct_Num))
		{
			break;
		}
	}
	Finger_Sw_Status(FINGER_CLOSE);
	if(Cnt >= 100)
	{

	}
}
/********************************************************
	功能：		抓杯电磁铁打开
	输入参数：	需要检测的光电开关的编号
	返回值：		无
********************************************************/
void FingerOpen(PCT_NUMBER Pct_Num)
{
	u8 Cnt = 0;
	for(Cnt=0;Cnt<5;Cnt++)
	{
		Finger_Sw_Status(FINGER_OPEN);
		delay_ms(300);
		Finger_Sw_Status(FINGER_CLOSE);
		delay_ms(100);
		if(PCT_Get_Status(Pct_Num))
			continue;
		else if(!PCT_Get_Status(Pct_Num))
			break;
	}
	if(Cnt >= 5)
	{
		CanSendData(0x00,CMD_SINGLE_FINGEROPEN,FAULT_FINGER_OPEN);
		CurrFaultLevel = RERESET_LEVEL;  //停止
		//FingerEcttmStatus = FINISHED;  //不执行组合动作的时候注释掉这个，
	}
	else
	{
		//FingerEcttmStatus = FINISHED;
	}
}
/********************************************************
	功能：		抽屉电磁铁自检
	输入参数：	需要检测的光电开关得到编号
	返回值：		无
********************************************************/
void CabinSelfCheck(PCT_NUMBER Pct_Num)
{
	u16 Cnt = 0;
	CabinorEnable(CABIN_ELECTM_ENABLE);
	while(Cnt < 100)
	{
		delay_ms(50);
		Cnt ++;
		if(!PCT_Get_Status(Pct_Num))
		{
			break;
		}
	}
	CabinorEnable(CABIN_ELECTM_DISABLE);
	if(Cnt >= 100)
	{
		CanSendData(0x00,CMD_COM_OPEN_CABIN,FAULT_ENABLE_CABIN);
	}
}
/********************************************************
	功能：		使能抽屉电磁铁
	输入参数：	无
	返回值：		无
********************************************************/
void CabinEnable(void)
{
	CabinorEnable(CABIN_ELECTM_ENABLE);
	CabinEcttmStatus = FINISHED;
}
/********************************************************
	功能：		失能抽屉电磁铁
	输入参数：	无
	返回值：		无
********************************************************/
void CabinDisable(void)
{
	CabinorEnable(CABIN_ELECTM_DISABLE);
	CabinEcttmStatus = FINISHED;
}
/********************************************************
	功能：		X轴方向电机参数配置
	输入参数：	X轴方向上的动作枚举，X轴方向上总的脉冲数，X轴方向的运行方向
	返回值：		无
********************************************************/
void X_MotorParaConfig(MotionCmd_TypeDef X_Motion,u32 X_TargetPulse,MOTOR_DIR Dir)
{
	MotionParaConfig_TypeDef X_ConfigPara;
	switch(X_Motion)
	{
		case X_NO_MOTION:break;
		case X_TO_ORIGIN:   //X轴初始化复位操作
		{
			//Motor_Init_Motion(MOTOR_1,speed_up_table[MOTOR_1],speed_down_table[MOTOR_1]);
			X_ConfigPara.status = X_TO_ORIGIN;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		}break;
		case X_TO_CUP_DISH:
			X_ConfigPara.status = X_TO_CUP_DISH;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		case X_TO_HATCH_IN:
			X_ConfigPara.status = X_TO_HATCH_IN;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		case X_TO_HATCH_OUT:
			X_ConfigPara.status = X_TO_HATCH_OUT;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		case X_TO_MIX:
			X_ConfigPara.status = X_TO_MIX;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		case X_TO_WASTE1:
			X_ConfigPara.status = X_TO_WASTE1;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		case X_TO_WASTE2:
			X_ConfigPara.status = X_TO_WASTE2;
			X_ConfigPara.direction = Dir;
			X_ConfigPara.pct_number = PCT_NO;
			X_ConfigPara.pct_encoder_number = ENCODER_X_AXIS;
			break;
		default:break;
	}
	if(X_TargetPulse == 0)
	{
		motor_info[DEVICE_UNIT_X].finish = FINISHED;
		return;
	}
	else if(X_TargetPulse <= 25)  		//很微小的步数
	{
		X_ConfigPara.acc_pulse = 0;
		X_ConfigPara.dec_pulse = 0;
		X_ConfigPara.const_pulse = X_TargetPulse; 
		X_ConfigPara.multiple = 1;
		X_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&X_ConfigPara,DEVICE_UNIT_X,&mindist_up_table[DEVICE_UNIT_X][5],NULL);
	}
	else if(X_TargetPulse < ((X_LARGE_ACC_PULSE + X_LARGE_DEC_PULSE) + 200 )) //如果目标脉冲数小于加减速阶段的总的脉冲数之和的话，执行小距离的加减速控制
	{
		X_ConfigPara.acc_pulse = X_SHORT_ACC_PULSE;
		X_ConfigPara.dec_pulse = X_SHORT_DEC_PULSE;
		X_ConfigPara.const_pulse = X_TargetPulse - (X_SHORT_ACC_PULSE + X_SHORT_DEC_PULSE);
		X_ConfigPara.multiple = X_PER_FP_PULSE_NUM;
		X_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&X_ConfigPara,DEVICE_UNIT_X,mindist_up_table[DEVICE_UNIT_X],mindist_down_table[DEVICE_UNIT_X]);
	}
	else
	{
		X_ConfigPara.acc_pulse = X_LARGE_ACC_PULSE;
		X_ConfigPara.dec_pulse = X_LARGE_DEC_PULSE;
		X_ConfigPara.const_pulse = X_TargetPulse - (X_LARGE_ACC_PULSE + X_LARGE_DEC_PULSE);
		X_ConfigPara.multiple = X_PER_FP_PULSE_NUM;
		X_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&X_ConfigPara,DEVICE_UNIT_X,speed_up_table[DEVICE_UNIT_X],speed_down_table[DEVICE_UNIT_X]);
	}
}
/********************************************************
	功能：		Y轴方向上的的电机参数配置
	输入参数：	Y轴反向的动作枚举，Y轴反向上的总的脉冲数，Y轴运动方向
	返回值：		无
********************************************************/
void Y_MotorParaConfig(MotionCmd_TypeDef Y_Motion,u32 Y_TargetPulse,MOTOR_DIR Dir)
{
	MotionParaConfig_TypeDef Y_ConfigPara;
	switch(Y_Motion)
	{
		case Y_NO_MOTION:break;
		case Y_TO_ORIGIN:
		{
			//Motor_Init_Motion(MOTOR_2,speed_up_table[MOTOR_2],speed_down_table[MOTOR_2]);
			//return;
			Y_ConfigPara.status = Y_TO_ORIGIN;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			//函数调用
		}break;
		case Y_TO_CUP_DISH:
		{
			Y_ConfigPara.status = Y_TO_CUP_DISH;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
		}break;
		case Y_TO_HATCH_IN:
			Y_ConfigPara.status = Y_TO_HATCH_IN;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			break;
		case Y_TO_HATCH_OUT:
			Y_ConfigPara.status = Y_TO_HATCH_OUT;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			break;
		case Y_TO_MIX:
			Y_ConfigPara.status = Y_TO_MIX;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			break;
		case Y_TO_WASTE1:
			Y_ConfigPara.status = Y_TO_WASTE1;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			break;
		case Y_TO_WASTE2:
			Y_ConfigPara.status = Y_TO_WASTE2;
			Y_ConfigPara.direction = Dir;
			Y_ConfigPara.pct_number = PCT_NO;
			Y_ConfigPara.pct_encoder_number = ENCODER_Y_AXIS;
			break;
		default:break;
	}
	if(Y_TargetPulse == 0)
	{
		motor_info[DEVICE_UNIT_Y].finish = FINISHED;
		return;
	}
	else if(Y_TargetPulse <= 25)  		//很微小的步数
	{
		Y_ConfigPara.acc_pulse = 0;
		Y_ConfigPara.dec_pulse = 0;
		Y_ConfigPara.const_pulse = Y_TargetPulse; 
		Y_ConfigPara.multiple = 1;
		Y_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&Y_ConfigPara,DEVICE_UNIT_Y,&mindist_up_table[DEVICE_UNIT_Y][5],NULL);
	}
	else if(Y_TargetPulse < ((Y_LARGE_ACC_PULSE + Y_LARGE_DEC_PULSE) - 10))
	{
		Y_ConfigPara.acc_pulse = Y_SHORT_ACC_PULSE;
		Y_ConfigPara.dec_pulse = Y_SHORT_DEC_PULSE;
		Y_ConfigPara.const_pulse = Y_TargetPulse - (Y_SHORT_ACC_PULSE + Y_SHORT_DEC_PULSE);
		Y_ConfigPara.multiple = Y_PER_FP_PULSE_NUM;
		Y_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&Y_ConfigPara,DEVICE_UNIT_Y,mindist_up_table[DEVICE_UNIT_Y],mindist_down_table[DEVICE_UNIT_Y]);
	}
	else
	{
		Y_ConfigPara.acc_pulse = Y_LARGE_ACC_PULSE;
		Y_ConfigPara.dec_pulse = Y_LARGE_DEC_PULSE;
		Y_ConfigPara.const_pulse = Y_TargetPulse - (Y_LARGE_ACC_PULSE + Y_LARGE_DEC_PULSE);
		Y_ConfigPara.multiple = Y_PER_FP_PULSE_NUM;
		Y_ConfigPara.pct_zero_run_pulse = 0;
		MotorMotionConfig_Init(&Y_ConfigPara,DEVICE_UNIT_Y,speed_up_table[DEVICE_UNIT_Y],speed_down_table[DEVICE_UNIT_Y]);
	}
}
/********************************************************
	功能：		Z轴方向上的电机参数配置
	输入参数：	Z轴方向电机动作枚举，Z轴方向上的总的脉冲数
	返回值：		无
********************************************************/
void Z_MotorParaConfig(MotionCmd_TypeDef Z_Motion,u32 Z_TargetPulse,MOTOR_DIR Dir)
{
	MotionParaConfig_TypeDef Z_ConfigPara;
	switch(Z_Motion)
	{
		case Z_NO_MOTION:break;
		case Z_TO_ORIGIN:
		{
			//函数调用
			//Motor_Init_Motion(MOTOR_3,speed_up_table[MOTOR_3],speed_down_table[MOTOR_3]);
			//return;
			Z_ConfigPara.status = Z_TO_ORIGIN;
			Z_ConfigPara.direction = Dir;
			Z_ConfigPara.pct_number = PCT_NO;
		}break;
		case Z_TO_WASTE:
			Z_ConfigPara.status = Z_TO_CUPDISH;
			Z_ConfigPara.direction = Dir;
			Z_ConfigPara.pct_number = PCT_NO;
			break;
		case Z_TO_MIX:
			Z_ConfigPara.status = Z_TO_CUPDISH;
			Z_ConfigPara.direction = Dir;
			Z_ConfigPara.pct_number = PCT_NO;
			break;
		case Z_TO_HATCH:
			Z_ConfigPara.status = Z_TO_CUPDISH;
			Z_ConfigPara.direction = Dir;
			Z_ConfigPara.pct_number = PCT_NO;
		    break;
		case Z_TO_CUPDISH:
		{
			Z_ConfigPara.status = Z_TO_CUPDISH;
			Z_ConfigPara.direction = Dir;
			Z_ConfigPara.pct_number = PCT_NO;
		}break;
		default:break;
	}
	if(Z_TargetPulse == 0)
	{
		motor_info[DEVICE_UNIT_Z].finish = FINISHED;
		return;
	}
	 if(Z_TargetPulse < ((Z_LARGE_ACC_PULSE + Z_LARGE_DEC_PULSE) - 10))
	{
		Z_ConfigPara.acc_pulse = Z_SHORT_ACC_PULSE;
		Z_ConfigPara.dec_pulse = Z_SHORT_ACC_PULSE;
		Z_ConfigPara.const_pulse = Z_TargetPulse - (Z_SHORT_ACC_PULSE + Z_SHORT_ACC_PULSE);
		Z_ConfigPara.multiple = Z_PER_FP_PULSE_NUM;
		Z_ConfigPara.pct_zero_run_pulse = 0;
		Z_ConfigPara.pct_encoder_number = PCT_ENCODER_NO;
		MotorMotionConfig_Init(&Z_ConfigPara,DEVICE_UNIT_Z,mindist_up_table[DEVICE_UNIT_Z],mindist_down_table[DEVICE_UNIT_Z]);
	}
	else
	{
		Z_ConfigPara.acc_pulse = Z_LARGE_ACC_PULSE;
		Z_ConfigPara.dec_pulse = Z_LARGE_DEC_PULSE;
		Z_ConfigPara.const_pulse = Z_TargetPulse-(Z_LARGE_ACC_PULSE + Z_LARGE_DEC_PULSE);
		Z_ConfigPara.multiple = Z_PER_FP_PULSE_NUM;
		Z_ConfigPara.pct_zero_run_pulse = 0;
		Z_ConfigPara.pct_encoder_number = PCT_ENCODER_NO;
		MotorMotionConfig_Init(&Z_ConfigPara,DEVICE_UNIT_Z,speed_up_table[DEVICE_UNIT_Z],speed_down_table[DEVICE_UNIT_Z]);
	}
}
/********************************************************
	功能：		混匀器电机参数配置
	输入参数：	混匀器电机动作枚举，混匀器电机要运行的总的脉冲数，要运行的方向
	返回值：		无
********************************************************/
void MixMotor_ParaConfig(MotionCmd_TypeDef MixMoition,u32 Mix_TargetPulse,MOTOR_DIR Dir)
{
	MotionParaConfig_TypeDef Mix_ConfigPara;

	switch(MixMoition)
	{
		case MIX_TO_ORIGIN:
		{
			Mix_ConfigPara.status = MIX_TO_ORIGIN;
			Mix_ConfigPara.direction = Dir;
			Mix_ConfigPara.pct_number = PCT_NO;  //混匀盘光电开关
			return;
		}break;
		case MIX_START_RUNING:
		{
			Mix_ConfigPara.status = MIX_START_RUNING;
			Mix_ConfigPara.direction = Dir;
			Mix_ConfigPara.pct_number = PCT_NO;  //混匀盘光电开关
		}break;
		default:break;
	}
	if(MixMoition != MIX_NO_MOTION)
	{
		if(Mix_TargetPulse == 0)
		{
			motor_info[DEVICE_UNIT_MIX].finish = FINISHED;
			return;
		}
		else if(Mix_TargetPulse < ((MIX_LARGE_ACC_PULSE + MIX_LARGE_DEC_PULSE) - 10))
		{
			Mix_ConfigPara.acc_pulse = MIX_SHORT_ACC_PULSE;
			Mix_ConfigPara.dec_pulse = MIX_SHORT_DEC_PULSE;
			Mix_ConfigPara.const_pulse = Mix_TargetPulse - (MIX_SHORT_ACC_PULSE + MIX_SHORT_DEC_PULSE);
			Mix_ConfigPara.multiple = MIX_PER_FP_PULSE_NUM;
			Mix_ConfigPara.pct_zero_run_pulse = 0;
			Mix_ConfigPara.pct_encoder_number = PCT_ENCODER_NO;
			MotorMotionConfig_Init(&Mix_ConfigPara,DEVICE_UNIT_MIX,mindist_up_table[DEVICE_UNIT_MIX],mindist_down_table[DEVICE_UNIT_MIX]);
		}
		else
		{
			Mix_ConfigPara.acc_pulse = MIX_LARGE_ACC_PULSE;
			Mix_ConfigPara.dec_pulse = MIX_LARGE_DEC_PULSE;
			Mix_ConfigPara.const_pulse = Mix_TargetPulse-(MIX_LARGE_ACC_PULSE + MIX_LARGE_DEC_PULSE);
			Mix_ConfigPara.multiple = MIX_PER_FP_PULSE_NUM;
			Mix_ConfigPara.pct_zero_run_pulse = 0;
			Mix_ConfigPara.pct_encoder_number = PCT_ENCODER_NO;
			MotorMotionConfig_Init(&Mix_ConfigPara,DEVICE_UNIT_MIX,speed_up_table[DEVICE_UNIT_MIX],speed_down_table[DEVICE_UNIT_MIX]);
		}
	}
}
/********************************************************
	功能：		底层电机参数配置，包括加减速阶段的脉冲数的配置，加减速表的调用
	输入参数：	数据调用结构体，执行电机号，加速表链接指针，减速表链接指针
	返回值：		无
********************************************************/
void MotorMotionConfig_Init(MotionParaConfig_TypeDef *ConfigPara,DeviceUnit_TypeDef sel,u16 *AccTab,u16 *DecTab)
{
	motor_info[sel].status = (MOTOR_MOTION_INFO)ConfigPara->status;
	motor_info[sel].direction = ConfigPara->direction;
	motor_info[sel].acc_pulse = ConfigPara->acc_pulse;
	motor_info[sel].dec_pulse = ConfigPara->dec_pulse;
	motor_info[sel].const_pulse = ConfigPara->const_pulse;
	motor_info[sel].multiple =  ConfigPara->multiple;
	motor_info[sel].speed_up_buff = AccTab;//
	motor_info[sel].speed_down_buff = DecTab;//
	motor_info[sel].pct_number = ConfigPara->pct_number;
	motor_info[sel].pct_zero_run_pulse = ConfigPara->pct_zero_run_pulse;
	motor_info[sel].pct_encoder_number = ConfigPara->pct_encoder_number;

	Motor_Motion_Setup((MOTOR_CHIP_SELECT)sel); //强制转换为底层所用的电机状态类型
}
/********************************************************
	功能：		Z轴抓杯手指下降到指定的工作位置
	输入参数：	工作位状态枚举
	返回值：		无
********************************************************/
void Z_To_WorkStation(MotionCmd_TypeDef WorkStation,orUnloadCup_TypeDef orUnloadCup)
{
	CaptureFingerPulse_TypeDef AxisPulase;
	if(WorkStation == Z_TO_ORIGIN)
	{
		FingerPositionData.Target_Position.Fingrer_Position_Z = Mchine_Position.Machine_Relative_Oringin_Z;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);   //计算三个轴的目标脉冲数
		FingerPositionData.Curr_Position.Fingrer_Position_Z = Mchine_Position.Machine_Relative_Oringin_Z;
	}
	else if(WorkStation == Z_TO_CUPDISH)
	{
		FingerPositionData.Target_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.CupDish_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);   //计算三个轴的目标脉冲数
		FingerPositionData.Curr_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.CupDish_Position;
	}
	else if(WorkStation == Z_TO_HATCH)
	{
		FingerPositionData.Target_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Hatch_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(orUnloadCup == CONFIRM_UNLOAD_CUP)
			AxisPulase.Finger_Pulse_Z -= CAPTURE_UNLOAD_CUP_HEIGHT_DIF;
		FingerPositionData.Curr_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Hatch_Position;
	}
	else if(WorkStation == Z_TO_MIX)
	{  
		FingerPositionData.Target_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Mix_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(orUnloadCup == CONFIRM_UNLOAD_CUP)
			AxisPulase.Finger_Pulse_Z -= CAPTURE_UNLOAD_CUP_HEIGHT_DIF;
		FingerPositionData.Curr_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Mix_Position;
	}
	else if(WorkStation == Z_TO_WASTE)
	{
		FingerPositionData.Target_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Waste_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(orUnloadCup == CONFIRM_UNLOAD_CUP)
			AxisPulase.Finger_Pulse_Z -= CAPTURE_UNLOAD_CUP_HEIGHT_DIF;
		FingerPositionData.Curr_Position.Fingrer_Position_Z = WorkParaCombine.Z_Work_Para.Waste_Position;
	}
	//AxisPulase.Finger_Pulse_Z = AxisPulase.Finger_Pulse_Z;
	if(WorkStation != Z_NO_MOTION)
		DispatchUnitFun[DEVICE_UNIT_Z](WorkStation,AxisPulase.Finger_Pulse_Z,AxisPulase.Z_Dir);  //使能Z轴

}
/********************************************************
	功能：		XY轴运动到指定的工作位置
	输入参数：	Z轴方向动作状态枚举，Y轴方向动作状态枚举，运动到杯盘的某一个位置需要用到的行程脉冲数
	返回值：		无
********************************************************/
void XY_To_WorkStation(MotionCmd_TypeDef X_WorkStation,MotionCmd_TypeDef Y_WorkStation,u32 X_Position,u32 Y_Position)
{
	CaptureFingerPulse_TypeDef AxisPulase;

	if(X_WorkStation==X_TO_ORIGIN || Y_WorkStation==Y_TO_ORIGIN)
	{
		if(X_WorkStation==X_TO_ORIGIN)
			FingerPositionData.Target_Position.Fingrer_Position_X = Mchine_Position.Machine_Relative_Origin_X;
		if(Y_WorkStation==Y_TO_ORIGIN)
			FingerPositionData.Target_Position.Fingrer_Position_Y = Mchine_Position.Machine_Relative_Oringin_Y;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation==X_TO_ORIGIN)
			FingerPositionData.Curr_Position.Fingrer_Position_X = Mchine_Position.Machine_Relative_Origin_X;
		if(Y_WorkStation==Y_TO_ORIGIN)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = Mchine_Position.Machine_Relative_Oringin_Y;
	}
	else if(X_WorkStation==X_TO_CUP_DISH || Y_WorkStation==Y_TO_CUP_DISH)
	{
		if(X_WorkStation == X_TO_CUP_DISH)
			FingerPositionData.Target_Position.Fingrer_Position_X = X_Position;
		if(Y_WorkStation == Y_TO_CUP_DISH)
			FingerPositionData.Target_Position.Fingrer_Position_Y = Y_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_CUP_DISH)
			FingerPositionData.Curr_Position.Fingrer_Position_X = X_Position;
		if(Y_WorkStation == Y_TO_CUP_DISH)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = Y_Position;
	}
	else if(X_WorkStation==X_TO_HATCH_IN || Y_WorkStation==Y_TO_HATCH_IN)
	{
		if(X_WorkStation == X_TO_HATCH_IN)
			FingerPositionData.Target_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Hatch_In_Position;
		if(Y_WorkStation == Y_TO_HATCH_IN)
			FingerPositionData.Target_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Hatch_In_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_HATCH_IN)
			FingerPositionData.Curr_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Hatch_In_Position;
		if(Y_WorkStation == Y_TO_HATCH_IN)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Hatch_In_Position;
	}
	else if(X_WorkStation==X_TO_HATCH_OUT || Y_WorkStation==Y_TO_HATCH_OUT)
	{
		if(X_WorkStation == X_TO_HATCH_OUT)
			FingerPositionData.Target_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Hatch_Out_Position;
		if(Y_WorkStation == Y_TO_HATCH_OUT)
			FingerPositionData.Target_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Hatch_Out_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_HATCH_OUT)
			FingerPositionData.Curr_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Hatch_Out_Position;
		if(Y_WorkStation == Y_TO_HATCH_OUT)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Hatch_Out_Position;
	}
	else if(X_WorkStation==X_TO_MIX || Y_WorkStation==Y_TO_MIX)
	{
		if(X_WorkStation == X_TO_MIX)
			FingerPositionData.Target_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Mix_Position;
		if(Y_WorkStation == Y_TO_MIX)
			FingerPositionData.Target_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Mix_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_MIX)
			FingerPositionData.Curr_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Mix_Position;
		if(Y_WorkStation == Y_TO_MIX)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Mix_Position;
	}
	else if(X_WorkStation==X_TO_WASTE1 || Y_WorkStation==Y_TO_WASTE1)
	{
		if(X_WorkStation == X_TO_WASTE1)
			FingerPositionData.Target_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Waste1_Position;
		if(Y_WorkStation == Y_TO_WASTE1)
			FingerPositionData.Target_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Waste1_Position;
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_WASTE1)
			FingerPositionData.Curr_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Waste1_Position;
		if(Y_WorkStation == Y_TO_WASTE1)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Waste1_Position;
	}
	else if(X_WorkStation==X_TO_WASTE2 || Y_WorkStation==Y_TO_WASTE2)
	{
		if(X_WorkStation == X_TO_WASTE2)
			FingerPositionData.Target_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Waste2_Position;
		if(Y_WorkStation == Y_TO_WASTE2)
			FingerPositionData.Target_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Waste2_Position;   
		AxisPulase = CalRouteTotalPulse(&FingerPositionData);
		if(X_WorkStation == X_TO_WASTE2)
			FingerPositionData.Curr_Position.Fingrer_Position_X = WorkParaCombine.X_Work_Para.Waste2_Position;
		if(Y_WorkStation == Y_TO_WASTE2)
			FingerPositionData.Curr_Position.Fingrer_Position_Y = WorkParaCombine.Y_Work_Para.Waste2_Position;
	}
	if(X_WorkStation != X_NO_MOTION)
		DispatchUnitFun[DEVICE_UNIT_X](X_WorkStation,AxisPulase.Finger_Pulse_X,AxisPulase.X_Dir);  //使能Z轴
	if(Y_WorkStation != Y_NO_MOTION)
		DispatchUnitFun[DEVICE_UNIT_Y](Y_WorkStation,AxisPulase.Finger_Pulse_Y,AxisPulase.Y_Dir);  //使能Z轴
}
/********************************************************
	功能：		混匀器就开始混匀
	输入参数：	混匀器运行状态枚举
	返回值：		无
********************************************************/
void Mix_StartRun(MotionCmd_TypeDef Mix_WorkStation)
{
	if(Mix_WorkStation==MIX_START_RUNING || Mix_WorkStation!=MIX_NO_MOTION)
		DispatchUnitFun[DEVICE_UNIT_MIX](Mix_WorkStation,MixRunMaxPulse,REV);
}
/********************************************************
	功能：		判断抓杯是否完成
	输入参数：	无
	返回值：		无
********************************************************/
void CupCaptureStatusJudge(void)
{
	switch(CurrComMotion)
	{
		case COM_NO_MOTION:
		{
			if(PCT_Get_Status(PCT_FINGER) == 1)
				CaptureCupExecuteStatus = UNLOAD_CUP_TO_WASTE;
			else
				CaptureCupExecuteStatus = NO_EXECUTE;
		}break;
		case COM_CURR_TO_CAPTURE: //复位点故去抓杯
		case C0M_HATCH_TO_CUP:
		{
			if(PCT_Get_Status(PCT_FINGER) == 0)
				CaptureCupExecuteStatus = RE_CAPTURE_CUP;
			else
				CaptureCupExecuteStatus = NO_EXECUTE;
		}break;
		default:break;
	}
}
/********************************************************
	功能：		软件版本查询
	输入参数：	无
	返回值：		无
********************************************************/
void Go_SysSoftwareVersionCheck(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{

			CanSendData(CAN_MAIN_MACHINE_ID,CMD_VERSION_INQUIRE,VERSION_INQUIE);
			SendOtherStatusInfoFlag = FINISHED;
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:ComMotionFinishFlag = TRUE;break;
		case 0x0003:break;
		case 0x0007:break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}	
}
/********************************************************
	功能：		发送硬件状态信息
	输入参数：	无
	返回值：		无
********************************************************/
void Go_SendStatusInfo(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{

			ReportHardwaretatus();
			SendOtherStatusInfoFlag = FINISHED;
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:ComMotionFinishFlag = TRUE;break;
		case 0x0003:break;
		case 0x0007:break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}	
}
/********************************************************
	功能：		发送工作位校准参数
	输入参数：	无
	返回值：		无
********************************************************/
void Go_SendConfigPara(void)
{
	u8 Len;
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{

			Len = sizeof(Work_Para_TypeDef);
			CanSendWorkStationInfo(CAN_MAIN_MACHINE_ID,COM_REPORT_CONFIG_PARA,&Work_Para.Para_Version[0],Len);
			SendOtherStatusInfoFlag = FINISHED;
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:ComMotionFinishFlag = TRUE;break;
		case 0x0003:break;
		case 0x0007:break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}	
}
/********************************************************
	功能：		混匀器混匀动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Mix_Running(void)
{

	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:
			{
				XYZFiner_Motion_Sel.To_Mix_StartRun(MIX_START_RUNING);
				//CanSendData(0x00,CMD_COM_MIX_RUNNING,REC_ACK);
			}break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从当前位置到原点的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Curr_To_Origin(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0001:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_ORIGIN,Y_TO_ORIGIN,0,0);break;
		case 0x0003:ComMotionFinishFlag = TRUE;break;
		case 0x0007:break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点到抓杯点的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Curr_To_CaptureCup(void)
{
	u16 X_Offset,Y_Offset;
	GetCaptureCupOffset(CapturePara.CurrentCaptureNum,&X_Offset,&Y_Offset);
	if(CapturePara.CurrentCaptureNum>0 && CapturePara.CurrentCaptureNum<=112)
		CapturePara.CupDishNum = 3;
	else if(CapturePara.CurrentCaptureNum>112 && CapturePara.CurrentCaptureNum<=224)
		CapturePara.CupDishNum = 2;
	else if(CapturePara.CurrentCaptureNum>=224 && CapturePara.CurrentCaptureNum<=336)
		CapturePara.CupDishNum = 1;
	#if RUN_MODE == 1
		GetCaptureCupPosition(CapturePara.CupDishNum,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);  //已经校准好了是用的便宜位置脉冲数
	#else
		switch(CapturePara.CurrentCaptureNum)   //为经过校准时的位置参数脉冲数
		{
			case CUP_DISH1_LEFT_TOP_CUP_NUM:	
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish1_LeftTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish1_LeftTop_Position;
				break;
			case CUP_DISH1_RIGHT_TOP_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish1_RightTop_Position;
				break;
			case CUP_DISH1_LEFT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position;
				break;
			case CUP_DISH1_RIGHT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position;
				break;
			
			case CUP_DISH2_LEFT_TOP_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish2_LeftTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish2_LeftTop_Position;
				break;
			case CUP_DISH2_RIGHT_TOP_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish2_RightTop_Position;
				break;
			case CUP_DISH2_LEFT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish2_LeftBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position;
				break;
			case CUP_DISH2_RIGHT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position;
				break;
				
			case CUP_DISH3_LEFT_TOP_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish3_LeftTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish3_LeftTop_Position;
				break;
			case CUP_DISH3_RIGHT_TOP_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish3_RightTop_Position;
				break;
			case CUP_DISH3_LEFT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish3_LeftBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position;
				break;
			case CUP_DISH3_RIGHT_BOTTOM_CUP_NUM:
				CapturePara.X_Position = WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position;
				CapturePara.Y_Position = WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position;
				break;
			default:break;
		}
	#endif
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000: 
			{
				XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_CUP_DISH,Y_TO_CUP_DISH,CapturePara.X_Position,CapturePara.Y_Position);
				//CanSendData(0x00,CMD_COM_ORIGN_TO_CAPTUREP_POS,REC_ACK);
			}break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_CUPDISH,UNCONFIRM_UNLOAD_CUP);break;		//XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN);
		case 0X0003:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;  		//回到复位点
		case 0X0007:
			ComMotionFinishFlag = TRUE;
			CapturePara.CurrentCaptureNum +=1;
			if(CapturePara.CurrentCaptureNum == 336)
				CapturePara.CurrentCaptureNum = 1;
			break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点初始位到孵育盘内圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_HatchIn(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_IN,Y_TO_HATCH_IN,0,0);break;
		case 0x0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0007:ComMotionFinishFlag = TRUE;break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点到孵育盘外圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_HatchOut(void)
{
	//float X_Coord_Route,Y_Coord_Route;
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_OUT,Y_TO_HATCH_OUT,0,0);
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0007:ComMotionFinishFlag = TRUE;break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点到混匀盘的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_Mix(void)
{
	//float X_Coord_Route,Y_Coord_Route;
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_MIX,Y_TO_MIX,0,0);
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:ComMotionFinishFlag = TRUE;
			//XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			//XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0x0007:/*ComMotionFinishFlag = TRUE;*/break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点到废料桶1的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_Waste1(void)
{
	//float X_Coord_Route,Y_Coord_Route;
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE1,Y_TO_WASTE1,0,0);
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			break;
		case 0x0007:ComMotionFinishFlag = TRUE;break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从原点到废料桶2的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_Waste2(void)
{
	//float X_Coord_Route,Y_Coord_Route;
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_TO_WASTE2,0,0);
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0x0007:ComMotionFinishFlag = TRUE;break;
		case 0x000F:break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		开机时从原点到废料盘扔杯子
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Orign_To_Waste_Throw_Cup(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0x0000:
		{
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_TO_WASTE2,0,0);
			//CanSendData(0x00,CMD_COM_ORIGN_TO_HATCH,REC_ACK);
		}break;
		case 0x0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0x0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0x0007:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_ORIGIN,Y_TO_ORIGIN,0,0); break;
		case 0x000F:ComMotionFinishFlag = TRUE;break;
		case 0x001F:break;
		case 0x003F:break;
		case 0x007F:break;
		case 0x00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯点到孵育盘内圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Cup_To_HatchIn(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_IN,Y_TO_HATCH_IN,0,0);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,CONFIRM_UNLOAD_CUP);break;
		case 0X0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X0007:ComMotionFinishFlag = TRUE;break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯点到孵育盘外圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Cup_To_HatchOut(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_OUT,Y_TO_HATCH_OUT,0,0);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,CONFIRM_UNLOAD_CUP);break;
		case 0X0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X0007:ComMotionFinishFlag = TRUE;break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯到废料桶
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Cup_To_Wast1(void)
{
	u16 CombineStage = 0;
	u16 X_Offset;   //X方向上的脉冲
	u16 Y_Offset;    //Y方向上的脉冲
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE1,Y_TO_WASTE1,0,0);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,CONFIRM_UNLOAD_CUP);break;
		case 0X0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0007:
		{
			GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
			if(CurrCaptureCupNum <= 112)
			{
				GetCaptureCupPosition(3,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>112 && CurrCaptureCupNum <= 224)
			{
				GetCaptureCupPosition(2,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
			{
				GetCaptureCupPosition(1,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_CUP_DISH,Y_TO_CUP_DISH,CapturePara.X_Position,CapturePara.Y_Position);
		}break;
		case 0X000F:ComMotionFinishFlag = TRUE;break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯到废料桶
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Cup_To_Wast2(void)
{
	u16 CombineStage = 0;
	u16 X_Offset;   //X方向上的脉冲
	u16 Y_Offset;    //Y方向上的脉冲
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_TO_WASTE2,0,0);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X0007:
			GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
			if(CurrCaptureCupNum <= 112)
			{
				GetCaptureCupPosition(3,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>112 && CurrCaptureCupNum <= 224)
			{
				GetCaptureCupPosition(2,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
			{
				GetCaptureCupPosition(1,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_CUP_DISH,Y_TO_CUP_DISH,CapturePara.X_Position,CapturePara.Y_Position);break;
		case 0X000F:ComMotionFinishFlag = TRUE;break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯到废料桶
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Cup_To_Mix(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_MIX,Y_TO_MIX,0,0);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,CONFIRM_UNLOAD_CUP); break;
		case 0X0003:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			//delay_ms(UNLOAD_RISE_TIME);
			break;
		case 0X0007:ComMotionFinishFlag = TRUE;break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯到废料桶
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Mix_To_Wast1(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0003:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE1,Y_TO_WASTE1,0,0);break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0X000F:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从杯盘抓杯到废料桶
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Mix_To_Wast2(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0003:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_TO_WASTE2,0,0);break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
		case 0X000F:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从孵育盘到混匀盘的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Hatch_To_Mix(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP); break;
		case 0X0003:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_MIX,Y_TO_MIX,0,0);break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,CONFIRM_UNLOAD_CUP);break;
		case 0X000F:
		{
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
		}break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从混匀盘到孵育盘内圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Mix_To_HatchIn(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0003:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_IN,Y_TO_HATCH_IN,0,0);break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,CONFIRM_UNLOAD_CUP);break;
		case 0X000F:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从混匀盘到孵育盘外圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Mix_To_HatchOut(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0003:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_OUT,Y_TO_HATCH_OUT,0,0);break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,CONFIRM_UNLOAD_CUP);break;
		case 0X000F:
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
			break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从孵育盘到杯盘抓杯点的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Hatch_To_CupDish(void)
{
	u16 CombineStage = 0;
	u16 X_Offset;   //X方向上的脉冲
	u16 Y_Offset;    //Y方向上的脉冲
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:
		{
			GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
			if(CurrCaptureCupNum <= 112)
			{
				GetCaptureCupPosition(3,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>112 && CurrCaptureCupNum <= 224)
			{
				GetCaptureCupPosition(2,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
			{
				GetCaptureCupPosition(1,X_Offset,Y_Offset,&CapturePara.X_Position,&CapturePara.Y_Position);
			}
			XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_CUP_DISH,Y_TO_CUP_DISH,CapturePara.X_Position,CapturePara.Y_Position);
		}break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从孵育盘到废料盘的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Hatch_To_Waste(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,UNCONFIRM_UNLOAD_CUP);break;
		case 0X0001:
		{
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
		}break;
		case 0X0003:
		{
			ThrowCupNum ++;
			if(ThrowCupNum < WASTE_MAX_NUM)
			{
				XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE1,Y_TO_WASTE1,0,0);
			}
			else
			{
				XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_TO_WASTE2,0,0);
			}
		}break;
		case 0X0007:XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,CONFIRM_UNLOAD_CUP); break;
		case 0X000F:
		{
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);
		}break;
		case 0X001F:ComMotionFinishFlag = TRUE;break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从废料盘到混匀盘内圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Waste_To_HatchIn(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_IN,Y_TO_HATCH_IN,0,0);break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		从废料盘到混匀盘外圈的动作调度
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Waste_To_HatchOut(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_OUT,Y_TO_HATCH_OUT,0,0);break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		抓杯手开始卸杯
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Unload_Cup(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:
		{
			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,CONFIRM_UNLOAD_CUP);
			delay_ms(100);
			XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);
		}break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		打开抽屉电磁铁
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Open_Cabin(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:CabinDisable();break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		锁住抽屉电磁铁
	输入参数：	无
	返回值：		无
********************************************************/
void Go_Lock_Cabin(void)
{
	u16 CombineStage = 0;
	CombineStage = Combine_Stage.MotionStatus & 0xFFFF;
	switch(CombineStage)
	{
		case 0X0000:CabinEnable();break;
		case 0X0001:ComMotionFinishFlag = TRUE;break;
		case 0X0003:break;
		case 0X0007:break;
		case 0X000F:break;
		case 0X001F:break;
		case 0X003F:break;
		case 0X007F:break;
		case 0X00FF:break;
		case 0X01FF:break;
		case 0X03FF:break;
		case 0X07FF:break;
		case 0X0FFF:break;
		case 0X1FFF:break;
		case 0X3FFF:break;
		case 0X7FFF:break;
		case 0XFFFF:break;
		default:break;
	}
}
/********************************************************
	功能：		清楚动作列表的内容
	输入参数：	无
	返回值：		无
********************************************************/
void ClearMotionList(void)
{
	Combine_Stage.MotionStatus = 0X0000;
}
/********************************************************
	功能：		电机运行状态量的初始化
	输入参数：	无
	返回值：		无
********************************************************/
void StartMotorParaInit(void)
{
	motor_info[DEVICE_UNIT_X].finish = NO_FINISH;
	motor_info[DEVICE_UNIT_Y].finish = NO_FINISH;
	motor_info[DEVICE_UNIT_Z].finish = NO_FINISH;
	motor_info[DEVICE_UNIT_MIX].finish = NO_FINISH;
}
/********************************************************
	功能：		系统运行参数的初始化
	输入参数：	无
	返回值：		无
********************************************************/
void SysInit(void)
{
	StartMotorParaInit();  	//电机参数初始化复位
	
	SysParaInitConfig();		//轴坐标参数配置
	CupDishPropertyConfig();	//杯盘固有属性配置根据校准后的参数值
	CupCaptureStatusJudge();  	//手指抓杯状态判断
	LED_ON;
	delay_ms(400);
	LED_OFF;
	delay_ms(400);
	LED_ON;
}
static u8 CanCmdList = 0;
/********************************************************
	功能：		对各个电机轴的参数复位清零操作
	输入参数：	无
	返回值：		无
********************************************************/
void ResetParaInit(DeviceUnit_TypeDef Axis)
{
	switch(Axis)
	{
		case DEVICE_UNIT_X:
		{
			FingerPositionData.Curr_Position.Fingrer_Position_X = Mchine_Position.Machine_Relative_Origin_X;
			FingerPositionData.Target_Position.Fingrer_Position_X = 0;
		}break;
		case DEVICE_UNIT_Y:
		{
			FingerPositionData.Curr_Position.Fingrer_Position_Y = Mchine_Position.Machine_Relative_Oringin_Y;
			FingerPositionData.Target_Position.Fingrer_Position_Y = 0;
		}break;
		case DEVICE_UNIT_Z:
		{
			FingerPositionData.Curr_Position.Fingrer_Position_Z = Mchine_Position.Machine_Relative_Oringin_Z;
			FingerPositionData.Target_Position.Fingrer_Position_Z = 0;
		}break;
		case DEVICE_UNIT_MIX:
		{
			;
			//FingerPositionData.Curr_Coord.Fingrer_Coord_Z = Mchine_Position.Machine_Relative_Oringin_Z;
			//FingerPositionData.Target_Coord.Fingrer_Coord_Z = 0;
		}break;
		case DEVICE_TOTAL:
		{
			FingerPositionData.Curr_Position.Fingrer_Position_X = Mchine_Position.Machine_Relative_Origin_X;
			FingerPositionData.Curr_Position.Fingrer_Position_Y = Mchine_Position.Machine_Relative_Oringin_Y;
			FingerPositionData.Curr_Position.Fingrer_Position_Z = Mchine_Position.Machine_Relative_Oringin_Z;
			FingerPositionData.Target_Position.Fingrer_Position_X = 0;
			FingerPositionData.Target_Position.Fingrer_Position_Y = 0;
			FingerPositionData.Target_Position.Fingrer_Position_Z = 0;
		}break;
		default:break;
	}
	CurrComMotion = COM_NO_MOTION;
	ComMotionFinishFlag = FALSE;
	ClearMotionList();  //清楚  动作列表
}
extern DeviceUnit_TypeDef MotorTab[4];  //外部调用
/********************************************************
	功能：		复位是否完成检测,发送错误代码
	输入参数：	电机编号
	返回值：		无
********************************************************/
void ResetJudge(DeviceUnit_TypeDef MotorSel)
{
	switch(MotorSel)
	{
		case DEVICE_UNIT_X:
		{
			if(PCT_1_STATUS == 1)   	//已经触发,不做任何处理
				return;
			else   						//未触发光电开关
				Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[0],speed_up_table[MotorTab[0]],speed_down_table[MotorTab[0]]);
		}break;
		case DEVICE_UNIT_Y:
		{
			if(PCT_2_STATUS == 1)   	//已经触发,不做任何处理
				return;
			else   						//未触发光电开关
				Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[1],speed_up_table[MotorTab[1]],speed_down_table[MotorTab[1]]);
		}break;
		case DEVICE_UNIT_Z:
		{
			if(PCT_3_STATUS == 1)   	//已经触发,不做任何处理
				return;
			else   						//未触发光电开关
				Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[2],speed_up_table[MotorTab[2]],speed_down_table[MotorTab[2]]);
		}break;
		case DEVICE_UNIT_MIX:
			return;
			break;
		default:break;
	}
}
/********************************************************
	功能：		动作调度处理
	输入参数：	无
	返回值：		无
********************************************************/
void ComMotin_AssistHandle(void)
{
	u8 Data = 0;
	u8 i;
	static u8 Stage = 0;
	switch(CurrFaultLevel)
	{
		case LEVEL_LOWEST:  			//最低级别
		{
			if(motor_info[DEVICE_UNIT_Z].finish==FINISHED || motor_info[DEVICE_UNIT_MIX].finish==FINISHED ||
				(motor_info[DEVICE_UNIT_X].finish==FINISHED && motor_info[DEVICE_UNIT_Y].finish==FINISHED) ||
				/*FingerEcttmStatus==FINISHED ||*/ CabinEcttmStatus==FINISHED || SendOtherStatusInfoFlag==FINISHED || ComMotionFinishFlag == TRUE)
			{
				if(motor_info[DEVICE_UNIT_Z].finish == FINISHED)
					motor_info[DEVICE_UNIT_Z].finish = NO_FINISH;
				if(motor_info[DEVICE_UNIT_MIX].finish == FINISHED)
				{
					motor_info[DEVICE_UNIT_MIX].finish = NO_FINISH;
					//return;  //这里根据需要加入是否是并行还是串行动作
				}
				if(motor_info[DEVICE_UNIT_X].finish==FINISHED && motor_info[DEVICE_UNIT_Y].finish==FINISHED)
				{
					motor_info[DEVICE_UNIT_X].finish = NO_FINISH;
					motor_info[DEVICE_UNIT_Y].finish = NO_FINISH;
				}
				/*if(FingerEcttmStatus == FINISHED)   			//不执行组合动作的时候注释掉这个
					FingerEcttmStatus = NO_FINISH;*/
				if(CabinEcttmStatus == FINISHED)
					CabinEcttmStatus = NO_FINISH;
				if(SendOtherStatusInfoFlag==FINISHED)
					SendOtherStatusInfoFlag = NO_FINISH;
				if(CurrComMotion != COM_NO_MOTION)  			//没有动作指令执行的话则不进入
				{
					if(ComMotionFinishFlag == TRUE)  			//运行完毕后判断标志为是否已经置位，若是则进行下一步
					{
						ComMotionFinishFlag = FALSE;
						Stage = 0;
						ClearMotionList();  //清楚动作列表
						//CanSendMotionFinishCmd(0X00,CurrComMotion);//CAN总线发送动作完成指令
						//用来做协处理 
						CupCaptureStatusJudge();  //判断手指上有没有杯子
						if(CurrComMotion == COM_CURR_TO_ORIGIN)    	//当前动作为执行到原点复位动作
						{  
							for(i=0;i<4;i++)
								ResetJudge(MotorTab[i]);  		   	//复位检测处理
							ResetParaInit(DEVICE_TOTAL);
						}
						CurrComMotion = COM_NO_MOTION;  			//组合动作清除
					}
					else  //组合动作还没有执行完毕的话继续进行
					{
						Stage ++;
						if(Stage > 16)
						{
							Stage = 0;
							return;
						}
						switch(Stage)
						{
							case 1:Combine_Stage.MotionStage.Stage1 = 1;break;
							case 2:Combine_Stage.MotionStage.Stage2 = 1;break;
							case 3:Combine_Stage.MotionStage.Stage3 = 1;break;
							case 4:Combine_Stage.MotionStage.Stage4 = 1;break;
							case 5:Combine_Stage.MotionStage.Stage5 = 1;break;
							case 6:Combine_Stage.MotionStage.Stage6 = 1;break;
							case 7:Combine_Stage.MotionStage.Stage7 = 1;break;
							case 8:Combine_Stage.MotionStage.Stage8 = 1;break;
							case 9:Combine_Stage.MotionStage.Stage9 = 1;break;
							case 10:Combine_Stage.MotionStage.Stage10 = 1;break;
							case 11:Combine_Stage.MotionStage.Stage11 = 1;break;
							case 12:Combine_Stage.MotionStage.Stage12 = 1;break;
							case 13:Combine_Stage.MotionStage.Stage13 = 1;break;
							case 14:Combine_Stage.MotionStage.Stage14 = 1;break;
							case 15:Combine_Stage.MotionStage.Stage15 = 1;break;
							case 16:Combine_Stage.MotionStage.Stage16 = 1;break;
							default:break;
						}
						//CAN_Cmd_Combine_Motion[CurrComMotion]();  //再次执行组合动作里面的动作函数
						CAN_Cmd_Combine_Motion[CurrComMotion].CanMotion();
					}
				}
			}
		}break;
		case STOP_LEVLE:  			//停止级
		{
			
		}break;
		case PAUSE_LEVEL:  			//暂停级
		{
			
		}break;
		case RERESET_LEVEL:   		//复位级
		{
			CombineReset();
			ResetParaInit(DEVICE_TOTAL);
			CanCmdList = 0;
			CurrFaultLevel = LEVEL_LOWEST;
		}break;
		case WAIT_MOTION_FINSH:   	//等待当前动作完成后再处理级别
		{
			
		}break;
		default:break;
	}
}
/********************************************************
	功能：		CAN指令解析
	输入参数：	CAN数据包
	返回值：		解析结果
********************************************************/
void CAN_Cmd_Haandle(ExecuteCmd_TypeDf *ExecuteCmdData)
{
	u16 X_Offset;   			//用于单轴调试找试剂杯用
	u16 Y_Offset;
	if(CurrComMotion == COM_NO_MOTION)
	{
		if(ExecuteCmd.ExecuteCmdValid==1 || CaptureCupExecuteStatus!=NO_EXECUTE)  		//X和Y轴运行在相同的情况下
		{
			ExecuteCmd.ExecuteCmdValid = 0;  					//外部接口调用*/
			switch(CaptureCupExecuteStatus)
			{
				case RE_CAPTURE_CUP:
					CaptureCupExecuteStatus = NO_EXECUTE;
					ExecuteCmd.MotionCmd = CMD_COM_CURR_TO_CAPTUREP_POS;
					break;
				case UNLOAD_CUP_TO_WASTE:
					CaptureCupExecuteStatus = NO_EXECUTE;
					ExecuteCmd.MotionCmd = CMD_COM_STARTMACHINE_THROW_CUP;
					break;
				default:break;
			}
			for(u8 i=0;i<CAN_COM_MOTION_MAX;i++)
			{
				if(ExecuteCmd.MotionCmd == CAN_Cmd_Combine_Motion[i].CanCmd)
				{
					ExecuteCmd.MotionCmd = 0x00;
					CAN_Cmd_Combine_Motion[i].CanMotion();
					CurrComMotion = CAN_Cmd_Combine_Motion[i].CurrComMotion;
					break;
				}
			}
			switch(ExecuteCmd.MotionCmd)
			{
				case CMD_COM_FAULT_HANDLE_RESULT:   		//中位机故障处理反馈结果
				{
					switch(ExecuteCmd.CopoperationCmd_1)
					{
						case STOP_LEVLE:
						{
							CurrFaultLevel = STOP_LEVLE;
							//SaveCupDataToEEprom();  		//执行停机前的操作
						}break;
						case PAUSE_LEVEL:
						{
							CurrFaultLevel = PAUSE_LEVEL;
						}break;
						case RERESET_LEVEL:
						{
							CurrFaultLevel = RERESET_LEVEL;
						}break;
						case WAIT_MOTION_FINSH:
						{
							CurrFaultLevel = WAIT_MOTION_FINSH;
						}break;
						default:CurrFaultLevel = LEVEL_LOWEST;break;
					}
				}break;
				case CMD_SINGLE_X_RESET:
					Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[0],speed_up_table[MotorTab[0]],speed_down_table[MotorTab[0]]);
					ResetParaInit(DEVICE_UNIT_X);
					motor_info[DEVICE_UNIT_X].finish = NO_FINISH;
					break;
				case CMD_SINGLE_Y_RESET:
					Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[1],speed_up_table[MotorTab[1]],speed_down_table[MotorTab[1]]);
					ResetParaInit(DEVICE_UNIT_Y);
					motor_info[DEVICE_UNIT_Y].finish = NO_FINISH;
					break;
				case CMD_SINGLE_Z_RESET:
					Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[2],speed_up_table[MotorTab[2]],speed_down_table[MotorTab[2]]);
					ResetParaInit(DEVICE_UNIT_Z);
					motor_info[DEVICE_UNIT_Z].finish = NO_FINISH;
					break;
				case CMD_SINGLE_MIX_RESET:
					Motor_Init_Motion((MOTOR_CHIP_SELECT)MotorTab[3],speed_up_table[MotorTab[3]],speed_down_table[MotorTab[3]]);
					ResetParaInit(DEVICE_UNIT_MIX);
					motor_info[DEVICE_UNIT_MIX].finish = NO_FINISH;
					break;
				case CMD_SINGLE_X_TO_CUP:
					GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
					if(CurrCaptureCupNum <= 112)
						X_Offset = WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position + X_Offset*CUP_SPACE;
					else if(CurrCaptureCupNum>112 && CurrCaptureCupNum <= 224)
						X_Offset = WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position + X_Offset*CUP_SPACE;
					else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
						X_Offset = WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position + X_Offset*CUP_SPACE;
					XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_CUP_DISH,Y_NO_MOTION,X_Offset,0);
					break;
			   case CMD_SINGLE_X_TO_HATCHIN:	XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_IN,Y_NO_MOTION,0,0);break;
			   case CMD_SINGLE_X_TO_HATCHOUT:	XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_HATCH_OUT,Y_NO_MOTION,0,0);break;
			   case CMD_SINGLE_X_TO_MIX:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_MIX,Y_NO_MOTION,0,0);break;
			   case CMD_SINGLE_X_TO_WASTE1:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE1,Y_NO_MOTION,0,0);break;
			   case CMD_SINGLE_X_TO_WASTE2:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_TO_WASTE2,Y_NO_MOTION,0,0);break;
			   case CMD_SINGLE_Y_TO_HATCHIN:	XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_HATCH_IN,0,0);break;
			   case CMD_SINGLE_Y_TO_HATCHOUT:	XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_HATCH_OUT,0,0);break;
			   case CMD_SINGLE_Y_TO_MIX:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_MIX,0,0);break;
			   case CMD_SINGLE_Y_TO_WASTE1:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_WASTE1,0,0);break;
			   case CMD_SINGLE_Y_TO_WASTE2:		XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_WASTE2,0,0);break;
			   case CMD_SINGLE_Z_UP:			XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_ORIGIN,UNCONFIRM_UNLOAD_CUP);break;
			   case CMD_SINGLE_Z_TO_CUP:		XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_CUPDISH,UNCONFIRM_UNLOAD_CUP);break;
			   case CMD_SINGLE_Z_TO_HATCH:		XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_HATCH,UNCONFIRM_UNLOAD_CUP);break;
			   case CMD_SINGLE_Z_TO_MIX:		XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_MIX,UNCONFIRM_UNLOAD_CUP);break;
			   case CMD_SINGLE_Z_TO_WASTE:		XYZFiner_Motion_Sel.To_Z_To_WorkStation(Z_TO_WASTE,UNCONFIRM_UNLOAD_CUP);break;
			   case CMD_SINGLE_FINGEROPEN:		XYZFiner_Motion_Sel.To_Finger_Open(PCT_FINGER);break;
			   case CMD_SINGLE_Y_TO_CUP:
			   {
					GetCaptureCupOffset(CurrCaptureCupNum,&X_Offset,&Y_Offset);
					if(CurrCaptureCupNum <= 112)
						Y_Offset = WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position + Y_Offset*CUP_SPACE;
					else if(CurrCaptureCupNum>112 && CurrCaptureCupNum <= 224)
						Y_Offset = WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position + Y_Offset*CUP_SPACE;
					else if(CurrCaptureCupNum>224 && CurrCaptureCupNum<=336)
						Y_Offset = WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position + Y_Offset*CUP_SPACE;
					XYZFiner_Motion_Sel.To_XY_To_WorkStation(X_NO_MOTION,Y_TO_CUP_DISH,0,Y_Offset);
				}break;
				default:break;
			}
			ExecuteCmd.MotionCmd = 0;
		}
	}
}
#if DEBUG_MODE == 1
/********************************************************
	功能：		抓杯手工作位参数校准整体
	输入参数：	CAN数据帧
	返回值：		无
	数据格式:   command  buf[0]     buf[1]    buf[2]   buf[3]
                 指令   执行指令    动作指令
********************************************************/
void DebugMode(Package_Info *unpack_data)
{
	u8 Pct_Status = 0;
	if(!CanDataHandle)
	{
		CanDataHandle = 0XFF;				
		switch(unpack_data->command)
		{
			case CMD_SINGLE_AXIS_WASPY_ADJUST:  
			{
				switch(unpack_data->buff[0])  //执行指令
				{
					case 0x01: 
					{
						switch(unpack_data->buff[1])
						{
							case CMD_COM_MACHINE_WHOLE_RESET:    //复位
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_COMRESET,REC_ACK);
								//CombineReset();
								CAN_Cmd_Combine_Motion[COM_CURR_TO_ORIGIN].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_CURR_TO_ORIGIN].CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_COMRESET,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_CAPTUREP_POS:  		//原点到杯盘
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_CUP,REC_ACK);
								CapturePara.CurrentCaptureNum = (u16)(unpack_data->buff[2]<<8|unpack_data->buff[3]);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_CAPTURE].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_CAPTURE].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_CUP,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_HATCH_IN:  	//原点到孵育盘内圈
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_HATCHIN,REC_ACK);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_HATCHIN].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_HATCHIN].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_HATCHIN,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_HATCH_OUT:   //原点到孵育盘外圈
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_HATCHOUT,REC_ACK);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_HATCHOUT].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_HATCHOUT].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_HATCHOUT,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_MIX:     	//原点到混匀盘
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_MIX,REC_ACK);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_MIX].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_MIX].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_MIX,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_WASTE1:   	//原点到废料桶1
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_WASTE1,REC_ACK);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_WASTE1].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_WASTE1].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_WASTE1,FIN_ACK);
								break;
							case CMD_COM_ORIGN_TO_WASTE2:   	//原点到废料桶2
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_WASTE2,REC_ACK);
								CAN_Cmd_Combine_Motion[COM_ORIGN_TO_WASTE2].CanMotion();
								CurrComMotion = CAN_Cmd_Combine_Motion[COM_ORIGN_TO_WASTE2].CurrComMotion;
								DebugComMotion = CurrComMotion;
								CanSendData(CAN_MAIN_MACHINE_ID,(CanMotionDispatchCmd_TypeDef)DEBUG_CMD_ORIGIN_TO_WASTE2,FIN_ACK);
								break;
							default:break;
						}
					}break;
					case 0x02:  //位置微调
					{
						CanSendData(CAN_MAIN_MACHINE_ID,CMD_SINGLE_AXIS_WASPY_ADJUST,REC_ACK);
						Wispy_Adjust(unpack_data);
						CanSendData(CAN_MAIN_MACHINE_ID,CMD_SINGLE_AXIS_WASPY_ADJUST,FIN_ACK);
					}break;
					case 0x03: 	//确认参数1
					{
						switch(DebugComMotion)
						{
							case COM_ORIGN_TO_CAPTURE:
								switch(CapturePara.CurrentCaptureNum)
								{
									//杯架3
									case CUP_DISH3_LEFT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish3_LeftTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish3_LeftTop_Position += Debug_CorrVal.Y_OffsetVal;
										WorkParaCombine.Z_Work_Para.CupDish_Position += Debug_CorrVal.Z_OffsetVal;    //第一次校准后后面都不用在继续校准了
									}break;
									case CUP_DISH3_RIGHT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish3_RightTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish3_RightTop_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH3_LEFT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish3_LeftBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish3_LeftBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH3_RIGHT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish3_RightBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish3_RightBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									//杯架2
									case CUP_DISH2_LEFT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish2_LeftTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish2_LeftTop_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH2_RIGHT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish2_RightTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish2_RightTop_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH2_LEFT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish2_LeftBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish2_LeftBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH2_RIGHT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish2_RightBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish2_RightBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									//杯架1
									case CUP_DISH1_LEFT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish1_LeftTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish1_LeftTop_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH1_RIGHT_TOP_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish1_RightTop_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish1_RightTop_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH1_LEFT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish1_LeftBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish1_LeftBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
									case CUP_DISH1_RIGHT_BOTTOM_CUP_NUM:
									{
										WorkParaCombine.X_Work_Para.CupDish1_RightBottom_Position += Debug_CorrVal.X_OffsetVal;
										WorkParaCombine.Y_Work_Para.CupDish1_RightBottom_Position += Debug_CorrVal.Y_OffsetVal;
									}break;
								}
								break;
							case COM_ORIGN_TO_HATCHIN:
							{
								WorkParaCombine.X_Work_Para.Hatch_In_Position += Debug_CorrVal.X_OffsetVal;
								WorkParaCombine.Y_Work_Para.Hatch_In_Position += Debug_CorrVal.Y_OffsetVal;
								WorkParaCombine.Z_Work_Para.Hatch_Position += Debug_CorrVal.Z_OffsetVal;
							}break;
							case COM_ORIGN_TO_HATCHOUT:
							{
								WorkParaCombine.X_Work_Para.Hatch_Out_Position += Debug_CorrVal.X_OffsetVal;
								WorkParaCombine.Y_Work_Para.Hatch_Out_Position += Debug_CorrVal.Y_OffsetVal;
								WorkParaCombine.Z_Work_Para.Hatch_Position += Debug_CorrVal.Z_OffsetVal;
							}break;
							case COM_ORIGN_TO_MIX:
							{
								WorkParaCombine.X_Work_Para.Mix_Position += Debug_CorrVal.X_OffsetVal;
								WorkParaCombine.Y_Work_Para.Mix_Position += Debug_CorrVal.Y_OffsetVal;
								WorkParaCombine.Z_Work_Para.Mix_Position += Debug_CorrVal.Z_OffsetVal;
							}break;
							case COM_ORIGN_TO_WASTE1:
							{
								WorkParaCombine.X_Work_Para.Waste1_Position += Debug_CorrVal.X_OffsetVal;
								WorkParaCombine.Y_Work_Para.Waste1_Position += Debug_CorrVal.Y_OffsetVal;
								WorkParaCombine.Z_Work_Para.Waste_Position += Debug_CorrVal.Z_OffsetVal;
							}break;
							case COM_ORIGN_TO_WASTE2:
							{
								WorkParaCombine.X_Work_Para.Waste2_Position += Debug_CorrVal.X_OffsetVal;
								WorkParaCombine.Y_Work_Para.Waste2_Position += Debug_CorrVal.Y_OffsetVal;
								WorkParaCombine.Z_Work_Para.Waste_Position += Debug_CorrVal.Z_OffsetVal;
							}break;
							default:break;
						}
						memset(&Debug_CorrVal.X_OffsetVal,0,sizeof(DebugPara_TypeDef));  //清零修正结构数据
					}break;
					case 0x04:  //保存参数
					{
						CanSendData(CAN_MAIN_MACHINE_ID,CMD_REPORT_CONFIG_PARA,REC_ACK);
						SaveMachhineWorkStationToEeprom();
						CanSendData(CAN_MAIN_MACHINE_ID,CMD_REPORT_CONFIG_PARA,FIN_ACK);
					}break;
					default:break;
				}
			}break;
			default:break;
		}
		memset(unpack_data->buff,0,8);
	}
}
#endif
//模拟上位机动作调度指令
CanMotionDispatchCmd_TypeDef UperComputerCmdTab[] = 
{
	CMD_COM_CURR_TO_CAPTUREP_POS, 	//从复位点到抓杯点
	CMD_COM_CUP_TO_HATCH_IN,		//从抓杯点到到孵育盘内圈
	CMD_COM_CURR_TO_CAPTUREP_POS,	//从孵育盘到抓杯点
	CMD_COM_CUP_TO_HATCH_OUT,		//从抓杯点到孵育盘外圈
	CMD_COM_HATCH_TO_MIX,			//从孵育盘外圈到混匀盘
	CMD_COM_MIX_RUNNING,			//混匀器开始混匀
	CMD_COM_MIX_TO_HATCH_OUT,
	CMD_COM_ORIGN_TO_HATCH_IN,
	CMD_COM_HATCH_TO_MIX,
	CMD_COM_MIX_RUNNING,
	CMD_COM_MIX_TO_HATCH_IN,
	CMD_COM_HATCH_TO_WASTE,
	CMD_COM_WASTE_TO_HATCH_OUT,
	CMD_COM_HATCH_TO_WASTE,
	CMD_COM_CURR_TO_ORIGIN,			//从当前位到复位点
};
/********************************************************
	功能：		模拟上位机发指令
	输入参数：	无
	返回值：		无
********************************************************/
void AnalogUperComputerSendCmd(void)
{
	if(CurrComMotion==COM_NO_MOTION && CaptureCupExecuteStatus == NO_EXECUTE)
	{
		ExecuteCmd.ExecuteCmdValid = 1;
		ExecuteCmd.MotionCmd = UperComputerCmdTab[CanCmdList++];
		if(CanCmdList == 15)
		{
			CanCmdList = 0;
		}
	}
}
/********************************************************
	功能：		CAN数据解析
	输入参数：	无
	返回值：		无
********************************************************/
void CanProtocolAnalyze(Package_Info *unpack_data)
{
	if(!CanDataHandle)
	{
		CanDataHandle = 0XFF;
		ExecuteCmd.ExecuteCmdValid = 1;  //数据有效
		ExecuteCmd.MotionCmd = unpack_data->command;
		memcpy(&unpack_data->buff,&ExecuteCmd.CopoperationCmd_1,8);
	}
}
/********************************************************
	功能：		主运行函数部分
	输入参数：	无
	返回值：		无
********************************************************/
void MotionHandle(void)
{
	#if RUN_MODE == 1
		AnalogUperComputerSendCmd();
		CanProtocolAnalyze(&unpack_data);
		ComMotin_AssistHandle();
		CAN_Cmd_Haandle(&ExecuteCmd);
	#endif
	#if DEBUG_MODE == 1
		ComMotin_AssistHandle();
		DebugMode(&unpack_data);
	#endif
}























































