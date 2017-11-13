#include "sys.h"
#include "public.h"
#include "can.h"
#include "em_sw.h"
#include "Config.h"
typedef enum
{
	FALSE =0,
	TRUE = 1,
}Status_TypeDef;
//Z����ȥ�Ƿ����ж������ִ��
typedef enum
{
	CONFIRM_UNLOAD_CUP = 0,
	UNCONFIRM_UNLOAD_CUP
}orUnloadCup_TypeDef;
//���̵���ʱ����������
typedef struct 
{
	s16 X_OffsetVal;
	s16 Y_OffsetVal;
	s16 Z_OffsetVal;
	u32  MixRunningTime;					//����������ʱ��
}DebugPara_TypeDef;
//X�����������ò�����Ϣ
typedef enum
{
	X_NEGATIVE_LIMIT_POSITION = 0,
	X_FORWARD_LIMIT_POSITION,
	X_INITAL_POSTION,
}X_No_Config_Para_TypeDef;
#define X_NO_CONFIG_PARA_MAX   							3    //�����û��߲������ò����ĳ���
//X���������ò�����Ϣ
typedef enum
{
	X_CUPDISH1_LEFTBOTTOM_POSITION = 0, 			 
	X_CUPDISH1_LEFTTOP_POSITION, 			
	X_CUPDISH1_RIGHTBOTTOM_POSITION, 		
	X_CUPDISH1_RIGHTTOP_POSITION, 			
	X_CUPDISH2_LEFTBOTTOM_POSITION, 			
	X_CUPDISH2_LEFTTOP_POSITION, 			
	X_CUPDISH2_RIGHTBOTTOM_POSITION, 		
	X_CUPDISH2_RIGHTTOP_POSITION, 			
	X_CUPDISH3_LEFTBOTTOM_POSITION, 			
	X_CUPDISH3_LEFTTOP_POSITION, 			
	X_CUPDISH3_RIGHTBOTTOM_POSITION, 		
	X_CUPDISH3_RIGHTTOP_POSITION, 			
	X_HATCH_IN_POSITION, 					
	X_HATCH_OUT_POSITION, 					
	X_MIX_POSITIN, 							
	X_WASTE_1_POSITION, 						
	X_WASTE_2_POSITION, 						
}X_Config_Para_TypeDef;
#define X_CONFIG_PARA_MAX   							17
//Y�����������ò�����Ϣ
typedef enum
{
	Y_NEGATIVE_LIMIT_POSITION = 0,   					
	Y_FORWARD_LIMIT_POSITION, 			
	Y_INITAL_POSTION, 					
}Y_No_Config_Para_TypeDef;
#define Y_NO_CONFIG_PARA_MAX   							3
//Y���������ò�����Ϣ
typedef enum
{
	Y_CUPDISH1_LEFTBOTTOM_POSITION = 0, 		
	Y_CUPDISH1_LEFTTOP_POSITION, 			
	Y_CUPDISH1_RIGHTBOTTOM_POSITION, 	
	Y_CUPDISH1_RIGHTTOP_POSITION, 		
	Y_CUPDISH2_LEFTBOTTOM_POSITION, 		
	Y_CUPDISH2_LEFTTOP_POSITION, 		
	Y_CUPDISH2_RIGHTBOTTOM_POSITION, 		
	Y_CUPDISH2_RIGHTTOP_POSITION, 		
	Y_CUPDISH3_LEFTBOTTOM_POSITION, 		
	Y_CUPDISH3_LEFTTOP_POSITION, 			
	Y_CUPDISH3_RIGHTBOTTOM_POSITION,		
	Y_CUPDISH3_RIGHTTOP_POSITION, 			
	Y_HATCH_IN_POSITION, 				
	Y_HATCH_OUT_POSITION, 				
	Y_MIX_POSITIN, 						
	Y_WASTE_1_POSITION, 					
	Y_WASTE_2_POSITION, 					
}Y_Config_Para_TypeDef;
#define Y_CONFIG_PARA_MAX   							17
//Z�����������ò�����Ϣ
typedef enum
{
	Z_NEGATIVE_LIMIT_POSITION = 0,				
	Z_FORWARD_LIMIT_POSITION, 				
	Z_INIT_PPOSITION, 
}Z_No_Config_Para_TypeDef;
#define Z_NO_CONFIG_PARA_MAX  							3
//Z���������ò�����Ϣ
typedef enum
{
	Z_CUPDISH_POSITION = 0, 						
	Z_HATCH_POSITION, 						
	Z_MIX_POSITION, 							
	Z_WASTE_POSITION, 
}Z_Config_Para_TypeDef;
#define Z_CONFIG_PARA_MAX  								4
//���ȵ���������ò�����Ϣ
typedef enum
{
	MIX_INIT_POSITION = 0,
}Mix_No_Config_Para_TypeDef;
#define MIX_NO_CONFIG_PARA_MAX   						1
//���ȵ�������ò�����Ϣ
typedef enum
{
	MIX_TIME = 0, 							
	MIX_SPEED,
}Mix_Config_Para_TypeDef;
#define MIX_CONFIG_PARA_MAX  							2
//�����ò������ݽṹ
typedef struct
{
	s16 Normal_Value;   //��׼ֵ
	s16 Adjust_Value;   //У׼ֵ
}ConfigParaInfo_TypeDef;
//��������λ����Ӧ�����嵱��(��λ:Step)
typedef struct
{
	u8 Para_Version[5];
	 struct   						//X���������λ
	{
		u8 Name[5];
		u8 No_Config_Para_Len;
		#if X_NO_CONFIG_PARA_MAX
			s16 No_Config_Para_Info[X_NO_CONFIG_PARA_MAX];
		#endif
		u8 Config_Para_Len;
		#if X_CONFIG_PARA_MAX
			ConfigParaInfo_TypeDef Config_Para_Info[X_CONFIG_PARA_MAX];
		#endif
	}X_Motor_Para_Info;
	 struct   						//X���������λ
	{
		u8 Name[5];
		u8 No_Config_Para_Len;
		#if Y_NO_CONFIG_PARA_MAX
			s16 No_Config_Para_Info[Y_NO_CONFIG_PARA_MAX];
		#endif
		u8 Config_Para_Len;
		#if Y_CONFIG_PARA_MAX
			ConfigParaInfo_TypeDef Config_Para_Info[Y_CONFIG_PARA_MAX];
		#endif
	}Y_Motor_Para_Info;
	 struct   						//X���������λ
	{
		u8 Name[5];
		u8 No_Config_Para_Len;
		#if Z_NO_CONFIG_PARA_MAX
			s16 No_Config_Para_Info[Z_NO_CONFIG_PARA_MAX];
		#endif
		u8 Config_Para_Len;
		#if Z_CONFIG_PARA_MAX
			ConfigParaInfo_TypeDef Config_Para_Info[Z_CONFIG_PARA_MAX];
		#endif
	}Z_Motor_Para_Info;
	 struct   						//X���������λ
	{
		u8 Name[5];
		u8 No_Config_Para_Len;
		#if MIX_NO_CONFIG_PARA_MAX
			s16 No_Config_Para_Info[MIX_NO_CONFIG_PARA_MAX];
		#endif
		u8 Config_Para_Len;
		#if MIX_CONFIG_PARA_MAX
			u8 Config_Para_Info[MIX_CONFIG_PARA_MAX];
		#endif
	}Mix_Motor_Para_Info;
	u16 CaptureUnloadCupHeightDif;  //ץж���߶Ȳ�
	u16 crc_value;
}Work_Para_TypeDef;
//ʵ��У׼����ں�ֵ
typedef struct
{
	struct
	{
		s16 CupDish1_LeftBottom_Position;
		s16 CupDish1_LeftTop_Position;
		s16 CupDish1_RightBottom_Position;
		s16 CupDish1_RightTop_Position;
		s16 CupDish2_LeftBottom_Position;
		s16 CupDish2_LeftTop_Position;
		s16 CupDish2_RightBottom_Position;
		s16 CupDish2_RightTop_Position;
		s16 CupDish3_LeftBottom_Position;
		s16 CupDish3_LeftTop_Position;
		s16 CupDish3_RightBottom_Position;
		s16 CupDish3_RightTop_Position;
		s16 Hatch_In_Position;
		s16 Hatch_Out_Position;
		s16 Mix_Position;
		s16 Waste1_Position;
		s16 Waste2_Position;
	}X_Work_Para;
	struct
	{
		s16 CupDish1_LeftBottom_Position;
		s16 CupDish1_LeftTop_Position;
		s16 CupDish1_RightBottom_Position;
		s16 CupDish1_RightTop_Position;
		s16 CupDish2_LeftBottom_Position;
		s16 CupDish2_LeftTop_Position;
		s16 CupDish2_RightBottom_Position;
		s16 CupDish2_RightTop_Position;
		s16 CupDish3_LeftBottom_Position;
		s16 CupDish3_LeftTop_Position;
		s16 CupDish3_RightBottom_Position;
		s16 CupDish3_RightTop_Position;
		s16 Hatch_In_Position;
		s16 Hatch_Out_Position;
		s16 Mix_Position;
		s16 Waste1_Position;
		s16 Waste2_Position;
	}Y_Work_Para;
	struct
	{
		s16 CupDish_Position;
		s16 Hatch_Position;
		s16 Mix_Position;
		s16 Waste_Position;
	}Z_Work_Para;
	u16 crc_value;
}WorkParaCombine_TypeDef;
//����������Ȧ��
typedef enum
{
	OUT_CIRCLE = 0,  				//��Ȧ
	IN_CIRCLE = 1,	 				//��Ȧ
}HatchCircleLayer;
//ץ����ָ��ǰ�Ĺ���״̬
typedef enum
{
	CAPTURE_FINGER_IDLE = 0,  //����״̬
	CAPTURE_FINGER_CATCHED_CUP,//ץ������
	CAPTURE_FINGER_UNCATCHED_CUP,//δץ������
	CAPTURE_FINGER_THROW_AWAY_CUP,//�ӵ�����
	CAPTURE_FINGER_UNTHROW_AWAY_CUP,//δ�ӵ�����
}CaptureFinger_Status_TypeDef;
//�豸��
typedef enum
{
	DEVICE_UNIT_X 		= MOTOR_1,
	DEVICE_UNIT_Y 		= MOTOR_2,
	DEVICE_UNIT_Z 		= MOTOR_3,
	DEVICE_UNIT_MIX 	= MOTOR_4,  //���ȱõ��
	DEVICE_UNIT_FINGER 	= 4,     	//ץ����ָ�����
	DEVICE_UNIT_CABIN  	= 5,		//���뿪�������
	DEVICE_TOTAL       	= 6,    	//����ϵͳ
}DeviceUnit_TypeDef;
//�̵�Ԫ��ִ�ж���ö��
typedef enum
{
	Z_NO_MOTION=0,
	Z_WISPY_ADJUST,
	Z_TO_ORIGIN,
	Z_TO_CUPDISH,				//�����½�������
	Z_TO_HATCH,				//�����½���������
	Z_TO_MIX,					//�����½���������
	Z_TO_WASTE,				//�����½���������
	FINGER_NO_MOTION,
	FINGER_INIT,
	FINGER_OPEN,					//��ָ��
	FINGER_CLOSE,					//��ָ�ر�
	FINGER_UNLOAD_CUP,
	X_NO_MOTION,
	X_WISPY_ADJUST,
	X_TO_ORIGIN,
	X_TO_CUP_DISH,
	X_TO_HATCH_IN,
	X_TO_HATCH_OUT,
	X_TO_MIX,
	X_TO_WASTE1,
	X_TO_WASTE2,
	Y_NO_MOTION,
	Y_WISPY_ADJUST,
	Y_TO_ORIGIN,
	Y_TO_CUP_DISH,
	Y_TO_HATCH_IN,
	Y_TO_HATCH_OUT,
	Y_TO_MIX,
	Y_TO_WASTE1,
	Y_TO_WASTE2,
	MIX_NO_MOTION,
	MIX_TO_ORIGIN,
	MIX_START_RUNING,  				//�����̿�ʼ����
	CABIN_INIT,
	CABIN_ELECTM_ENABLE,     		//ʹ�ܳ�������
	CABIN_ELECTM_DISABLE,    		//��ʹ�ܳ�������
}MotionCmd_TypeDef;
#if DEBUG_MODE == 1
//����λ����ʱָ���
typedef enum
{
	DEBUG_CMD_COMRESET             = 0x00,
	DEBUG_CMD_ORIGIN_TO_CUP   	   = 0X01,
	DEBUG_CMD_ORIGIN_TO_HATCHIN	   = 0X02,
	DEBUG_CMD_ORIGIN_TO_HATCHOUT   = 0X03,
	DEBUG_CMD_ORIGIN_TO_MIX		   = 0X04,
	DEBUG_CMD_ORIGIN_TO_WASTE1	   = 0X05,
	DEBUG_CMD_ORIGIN_TO_WASTE2	   = 0X06,
}DebugCmdWorkStation_TypeDef;
#endif
//ץ��ִ��״̬
typedef enum
{
	NO_EXECUTE =  0,
	UNLOAD_CUP_TO_WASTE = 1,
	RE_CAPTURE_CUP = 2,
}CaptureCupStatus_TypeDef;
//��϶����������к�
typedef enum
{
	COM_NO_MOTION = 0,
	COM_VERSION_INQUIRE,
	COM_REPORT_HARD_STAUS,
	COM_REPORT_CONFIG_PARA,
	COM_CURR_TO_ORIGIN,
	COM_CURR_TO_CAPTURE,
	COM_ORIGN_TO_HATCHIN,
	COM_ORIGN_TO_HATCHOUT,
	COM_ORIGN_TO_MIX,
	COM_ORIGN_TO_WASTE1,
	COM_ORIGN_TO_WASTE2,
	C0M_CUP_TO_HATCHIN,
	C0M_CUP_TO_HATCHOUT,
	COM_CUPDISH_TO_WASTE1,
	COM_CUPDISH_TO_WASTE2,
	COM_CUPDISH_TO_MIX,
	COM_MIX_TO_WASTE1,
	COM_MIX_TO_WASTE2,
	COM_STARTMACHINE_THROW_CUP,  //��������б��ӵĻ�������
	C0M_HATCH_TO_CUP,
	C0M_HATCH_TO_MIX,
	C0M_MIX_TO_HATCHIN,
	C0M_MIX_TO_HATCHOUT,
	C0M_HATCH_TO_WASTE,
	C0M_WASTE_TO_HATCHIN,
	C0M_WASTE_TO_HATCHOUT,
	COM_OPEN_CABIN,
	COM_CLOSE_CABIN,
	COM_MIX_RUNNING,
	COM_UNLOAD_CUP,
}COM_ListTypeDef;
#define 	CAN_COM_MOTION_MAX       29   //CAN���ɵ��ȵĶ�������Ŀ
//CAN��������ָ��
typedef enum
{
	CMD_COM_NULL				 =  0X00,
	CMD_VERSION_INQUIRE          =  0X01, 
	CMD_REPORT_HARD_STAUS        =  0X02,
	CMD_REPORT_CONFIG_PARA		 =  0X03,
	CMD_COM_FAULT_HANDLE_RESULT  =  0X04,  			
	CMD_COM_MACHINE_WHOLE_RESET  =  0X10,  			
	CMD_COM_CURR_TO_ORIGIN       =  0X11, 			
	CMD_COM_CURR_TO_CAPTUREP_POS =  0X20,			
	CMD_COM_ORIGN_TO_HATCH_IN     = 0X21,
	CMD_COM_ORIGN_TO_HATCH_OUT    = 0X22,
	CMD_COM_ORIGN_TO_MIX		  = 0X23,
	CMD_COM_ORIGN_TO_WASTE1       = 0X24,
	CMD_COM_ORIGN_TO_WASTE2       = 0X25,
	CMD_COM_CUP_TO_HATCH_IN 	  = 0X26,
	CMD_COM_CUP_TO_HATCH_OUT      = 0X27,
	CMD_COM_HATCH_TO_CUP 		  = 0X28,
	CMD_COM_STARTMACHINE_THROW_CUP = 0X29,
	CMD_COM_HATCH_TO_MIX 		  = 0X2A,
	CMD_COM_MIX_TO_HATCH_IN 	  = 0X2B,
	CMD_COM_MIX_TO_HATCH_OUT 	  = 0X2C,
	CMD_COM_HATCH_TO_WASTE 		  = 0X2D,
	CMD_COM_WASTE_TO_HATCH_IN 	  = 0X2E,
	CMD_COM_WASTE_TO_HATCH_OUT	  = 0X2F,
	CMD_COM_CUPDISH_TO_WASTE1	  = 0X30,
	CMD_COM_CUPDISH_TO_WASTE2	  =	0X31,
	CMD_COM_CUPDISH_TO_MIX		  = 0X32,
	CMD_COM_MIX_TO_WASTE1         = 0x33,
	CMD_COM_MIX_TO_WASTE2         = 0x34,
	
	CMD_COM_OPEN_CABIN      	  = 0X35,
	CMD_COM_CLOSE_CABIN     	  = 0X36,
	CMD_COM_MIX_RUNNING           = 0X40,
	CMD_COM_UNLOAD_CUP            = 0X41,
	CMD_SINGLE_X_RESET			  =	0X50,
	CMD_SINGLE_X_TO_CUP           = 0X51,
	CMD_SINGLE_X_TO_HATCHIN       = 0X52,
	CMD_SINGLE_X_TO_HATCHOUT      = 0X53,
	CMD_SINGLE_X_TO_MIX           = 0X54,
	CMD_SINGLE_X_TO_WASTE1        = 0X55,
	CMD_SINGLE_X_TO_WASTE2        = 0X56,
	CMD_SINGLE_Y_RESET			  =	0X60,
	CMD_SINGLE_Y_TO_CUP           = 0X61,
	CMD_SINGLE_Y_TO_HATCHIN       = 0X62,
	CMD_SINGLE_Y_TO_HATCHOUT      = 0X63,
	CMD_SINGLE_Y_TO_MIX           = 0X64,
	CMD_SINGLE_Y_TO_WASTE1        = 0X65,
	CMD_SINGLE_Y_TO_WASTE2        = 0X66,
	CMD_SINGLE_Z_RESET			  =	0X70,
	CMD_SINGLE_Z_UP				  = 0x71,
	CMD_SINGLE_Z_TO_CUP           = 0X72,
	CMD_SINGLE_Z_TO_HATCH         = 0X73,
	CMD_SINGLE_Z_TO_MIX           = 0X74,
	CMD_SINGLE_Z_TO_WASTE         = 0X75,
	CMD_SINGLE_MIX_RESET		  = 0X80,
	CMD_SINGLE_FINGEROPEN         = 0X90,
	CMD_SINGLE_AXIS_WASPY_ADJUST  = 0XF3
}CanMotionDispatchCmd_TypeDef;
#define 	CAN_SINGLE_MOTION_MAX       37
//ִ��ָ��
typedef struct
{
	u8 ExecuteCmdValid;     //ִ��ָ����Ч
	CanMotionDispatchCmd_TypeDef MotionCmd;  //����ָ��
	u8 CopoperationCmd_1;
	u8 CopoperationCmd_2;
	u8 CopoperationCmd_3;
	u8 CopoperationCmd_4;
	u8 CopoperationCmd_5;
	u8 CopoperationCmd_6;
	u8 CopoperationCmd_7;
	u8 CopoperationCmd_8;
}ExecuteCmd_TypeDf;
//������У׼���ʵ�ʹ��в���
typedef struct
{
	u16 Cup1_X_Actual_Space;
	u16 Cup1_Y_Actual_Space;
	
	u16 Cup2_X_Actual_Space;
	u16 Cup2_Y_Actual_Space;
	
	u16 Cup3_X_Actual_Space;
	u16 Cup3_Y_Actual_Space;
}CupDishCalibrationProperty_TypeDef;
//Ӳ��״̬�ṹ
typedef struct
{
	union
	{
		u16 AllStatus;
		struct
		{
			u8 Pct_X:		1;
			u8 Pct_Y:		1;
			u8 Pct_Z:		1;
			u8 Pct_Mix:		1;
			u8 Pct_Finger:	1;
			u8 Pct_Waste1:	1;
			u8 Pct_Waste2:	1;
			u8 Pct_Cabin:	1;
			u8 Ect_Finger:	1;
			u8 EctCabin_:	1;
			u8 Ect_Reserve:	6;
		}UnitStatus;
	}HardStatus;   					//Ӳ��״̬
	struct
	{
		u16 CurrCaptureCupNum;  	//��ǰץȡ�ı�����
		u16 CurrThrowCupNum;    	//��ǰ�ӵ��ı�����
		COM_ListTypeDef CurrExecuteMotion;  //��ǰִ�еĶ���
	}ExecutePara;
}SysStatusInquiry_TypeDef;  			//ϵͳ״̬��ѯ
//��λ�������Ĺ��ϵļ����Ƿ�ֹͣ��ǰ�������Ǽ�����һ���Ķ�������ִ��
typedef enum
{
	LEVEL_LOWEST        = 0X00,         //������ͣ����Լ���ִ��
	STOP_LEVLE 			= 0X01,  	 	//ֹͣ
	PAUSE_LEVEL 		= 0X02,			//��ͣ
	RERESET_LEVEL 		= 0X03,			//���¸�λ
	WAIT_MOTION_FINSH 	= 0X04,  		//�ȴ���ǰ�������
	
}FaultLevel_TypeDef;
//�������б�����
typedef enum
{
	REC_ACK = 0,
	REC_NACK,
	FIN_ACK,
	VERSION_INQUIE,
	CONFIG_PARA,
	FAULT_RESET_X_AXIS,
	FAULT_RESET_Y_AXIS,
	FAULT_RESET_Z_AXIS,
	FAULT_RESET_MIX,
	FAULT_ORIGIN_TO_CAPTURECUP,
	FAULT_ORIGIN_TO_HATCHIN,
	FAULT_ORIGIN_TO_HATCHOUT,
	
	FAULT_CUP_TO_HATCHIN,
	FAULT_CUP_TO_HATCHOUT,
	FAULT_HATCH_TO_CUP,
	FAULT_HATCH_TO_MIX,
	FAULT_MIX_TO_HATCHIN,
	FAULT_MIX_TO_HATCHOUT,
	FAULT_HATCH_TO_WASTE,
	FAULT_WASTE_TO_HATCHIN,
	FAULT_WASTE_TO_HATCHOUT,
	FAULT_ENABLE_CABIN,
	FAULT_MIX_RUNNING,
	FAULT_X_TO_CAPTURE,
	FAULT_X_TO_HATCHIN,
	FAULT_X_TO_HATCHOUT,
	FAULT_X_TO_MIX,
	FAULT_X_TO_WASTE1,
	FAULT_X_TO_WASTE2,
	FAULT_Y_TO_CAPTURE,
	FAULT_Y_TO_HATCHIN,
	FAULT_Y_TO_HATCHOUT,
	FAULT_Y_TO_MIX,
	FAULT_Y_TO_WASTE1,
	FAULT_Y_TO_WASTE2,
	FAULT_Z_TO_CUP,
	FAULT_Z_TO_HATCH,
	FAULT_Z_TO_MIX,
	FAULT_Z_TO_WASTE,
	FAULT_FINGER_OPEN,
}AnswerCode_TypeDef;
#define FAULT_TYPE_MAX_NUM  39
//�����б����
typedef struct
{
	AnswerCode_TypeDef Fault_Type;
	char *FaultText;
}FaultTypeCmdTab_TypeDef;
//XYZ��ץ����ָ�����״̬��������˶������������
typedef union
{
	u16 MotionStatus;
	struct
	{
		u8 Stage1:1;
		u8 Stage2:1;
		u8 Stage3:1;
		u8 Stage4:1;
		u8 Stage5:1;
		u8 Stage6:1;
		u8 Stage7:1;
		u8 Stage8:1;
		u8 Stage9:1;
		u8 Stage10:1;
		u8 Stage11:1;
		u8 Stage12:1;
		u8 Stage13:1;
		u8 Stage14:1;
		u8 Stage15:1;
		u8 Stage16:1;
	}MotionStage;
}CombineStage;
//���ϴ����������ݽṹ
//ռ���ֽ���;8Byte
/*typedef union
{
	u8 FaultBuf[8];
	struct
	{
		u32 FaultDetc1;
		u32 FaultDetc2;
	}FaultDetect;
}FaultBuff_TypeDef;*/
//���ϴ�������ṹ��
//�������¼���ϴιػ�������һ��ץ�����λ��
typedef struct
{
	u8 CupDishNum;
	u16 CurrentCaptureNum;  			//��ǰ�Ѿ�ץȡ�ı�����
	u32 X_Position;
	u32 Y_Position;
}CapturePara_TpyeDef;
//�������ԭ������
typedef struct
{
	int32_t  Machine_Absolute_Origin_X;  	//����ԭ������(��е����ϵ)
	int32_t  Machine_Absolute_Origin_Y;
	int32_t  Machine_Absolute_Origin_Z;
	int32_t	 Machine_Relative_Origin_X;     //���ԭ������(��������ϵ)
	int32_t  Machine_Relative_Oringin_Y;
	int32_t  Machine_Relative_Oringin_Z;
}Machine_Position_TypeDef;
//ץ����ָ����
typedef struct
{
	struct
	{
	    int32_t Fingrer_Position_X;
		int32_t Fingrer_Position_Y;
		int32_t Fingrer_Position_Z;
	}Curr_Position;  					//��ǰ��������
	struct
	{
		int32_t Fingrer_Position_X;
		int32_t Fingrer_Position_Y;
		int32_t Fingrer_Position_Z;
	}Target_Position;
}CaptureFinger_Position_TypeDef;
//ץ����ָ���е�Ŀ��λ�ü��������������
typedef struct
{
	MOTOR_DIR X_Dir;  					//������
	MOTOR_DIR Y_Dir;
	MOTOR_DIR Z_Dir;
	u32 Finger_Pulse_X;  				//����
	u32 Finger_Pulse_Y;
	u32 Finger_Pulse_Z;
}CaptureFingerPulse_TypeDef;
//�������ò�������
typedef struct
{
	MotionCmd_TypeDef status;			//����˶�״̬
	MOTOR_DIR direction;				//����˶�����
	u32 acc_pulse;						//��Q�ٶ��ж��ٸ�����
	u32 dec_pulse;						//���ٶ��ж��ٸ�����
	u32 const_pulse;					//���ٶ��ж��ٸ�����
	u8 multiple;						//����Ƶ���ж��ٸ�����
	PCT_NUMBER pct_number;
	u16 pct_zero_run_pulse;
	PCT_ENCODER_NUMBER pct_encoder_number;
}MotionParaConfig_TypeDef;

typedef void (*ExecuteMotionCANCmd_TypeDef)(void);  	//CANָ����϶���
typedef struct   		//Canָ������Ƚṹ
{
	CanMotionDispatchCmd_TypeDef CanCmd;  //ָ���
	ExecuteMotionCANCmd_TypeDef  CanMotion;
	COM_ListTypeDef CurrComMotion;
	
}CanCmdMotionDispatch_TypeDef;
typedef void (*DispatchUnitExecuteFun_TypeDef[])(MotionCmd_TypeDef Axis_Motion,u32 ExecutePara,MOTOR_DIR Dir); 
typedef void (*FingerOpen_TypeDef)(PCT_NUMBER Pct_Num);  //ץ����ָ��
typedef void (*Z_To_WorkStation_TypeDef)(MotionCmd_TypeDef WorkStation,orUnloadCup_TypeDef orUnloadCup);//Z���½�������λ
typedef void (*XY_To_WorkStation_TypeDef)(MotionCmd_TypeDef X_WorkStation,MotionCmd_TypeDef Y_WorkStation,u32 X_Position,u32 Y_Position);//XY�����е�����λ
typedef void (*Mix_StartRun_TypeDef)(MotionCmd_TypeDef Mix_WorkStation);
typedef struct
{
	FingerOpen_TypeDef  To_Finger_Open;
	Z_To_WorkStation_TypeDef To_Z_To_WorkStation;
	XY_To_WorkStation_TypeDef To_XY_To_WorkStation;
	Mix_StartRun_TypeDef To_Mix_StartRun;
}XYZFingerMotion_TypeDef;
void SaveCupDataToEEprom(void);
void SaveMachhineWorkStationToEeprom(void);
void SaveBeltPara(void);
void BeltParaCorr(DeviceUnit_TypeDef MotorSel,u8 TrnsmuDir);
void SysParaInitConfig(void);
void CupDishPropertyConfig(void);
void SysInit(void);
void WorkPositionParaConfig(void);

//void MotionParaCopnfig(MotionParaConfig_TypeDef *ConfigPara,MOTOR_CHIP_SELECT sel);
void MotorMotionConfig_Init(MotionParaConfig_TypeDef *ConfigPara,DeviceUnit_TypeDef sel,u16 *AccTab,u16 *DecTab);

void CanSendWorkStationInfo(u8 DestId,CanMotionDispatchCmd_TypeDef ExecuteCmd,u8 *buf,u16 buf_len);
void ReportHardwaretatus(void);
void ResetParaInit(DeviceUnit_TypeDef Axis);
void ResetJudge(DeviceUnit_TypeDef MotorSel);
void AxisCoordConfig(void);
void GetCaptureCupOffset(u16 CurrCaptureCupNum,u16 *XOffsetCoord,u16 *YOffsetCoord);
void GetCaptureCupPosition(u8 CupNum,u16 X_Offset,u16 Y_Offset,u32 *X_Offset_Posiotion,u32 *Y_Offset_Position);
CaptureFingerPulse_TypeDef CalRouteTotalPulse(CaptureFinger_Position_TypeDef *Finger_Position);

void Z_MotorParaConfig(MotionCmd_TypeDef Z_Motion,u32 Z_TargetPulse,MOTOR_DIR Dir);
void Y_MotorParaConfig(MotionCmd_TypeDef Y_Motion,u32 Y_TargetPulse,MOTOR_DIR Dir);
void X_MotorParaConfig(MotionCmd_TypeDef X_Motion,u32 X_TargetPulse,MOTOR_DIR Dir);
void MixMotor_ParaConfig(MotionCmd_TypeDef MixMoition,u32 Mix_TargetPulse,MOTOR_DIR Dir);

void CabinorEnable(MotionCmd_TypeDef ElectmStatus);
void FingerOpen(PCT_NUMBER Pct_Num);
void Z_To_WorkStation(MotionCmd_TypeDef WorkStation,orUnloadCup_TypeDef orUnloadCup);
void XY_To_WorkStation(MotionCmd_TypeDef X_WorkStation,MotionCmd_TypeDef Y_WorkStation,u32 X_Position,u32 Y_Position);
void Mix_StartRun(MotionCmd_TypeDef Mix_WorkStation);

void Wispy_Adjust(Package_Info *unpack_data);
void CalMixMaxPoint(u8 AllTime);

void ComMotin_AssistHandle(void);
void ClearMotionList(void);
void FaultDetect(void);
void CupCaptureStatusJudge(void);


//CAN������϶�������
void Go_SysSoftwareVersionCheck(void);
void Go_SendConfigPara(void);
void Go_SendStatusInfo(void);
void Go_Mix_Running(void);
void Go_Curr_To_CaptureCup(void);
void Go_Orign_To_Waste_Throw_Cup(void);
void Go_Orign_To_HatchIn(void);
void Go_Orign_To_HatchOut(void);
void Go_Orign_To_Mix(void);
void Go_Orign_To_Waste1(void);
void Go_Orign_To_Waste2(void);
void Go_Curr_To_Origin(void);

void Go_Cup_To_HatchIn(void);
void Go_Cup_To_HatchOut(void);
void Go_Cup_To_Wast1(void);
void Go_Cup_To_Wast2(void);
void Go_Cup_To_Mix(void);
void Go_Mix_To_Wast1(void);
void Go_Mix_To_Wast2(void);

void Go_Hatch_To_Mix(void);
void Go_Mix_To_HatchIn(void);
void Go_Mix_To_HatchOut(void);
void Go_Hatch_To_CupDish(void);
void Go_Hatch_To_Waste(void);
void Go_Waste_To_HatchIn(void);
void Go_Waste_To_HatchOut(void);
void Go_Unload_Cup(void);

void Go_Open_Cabin(void);
void Go_Lock_Cabin(void);

void Debug_Go_Orign_To_Capture(void);
void Debug_Go_Orign_To_HatchIn(void);
void Debug_Go_Orign_To_HatchOut(void);
void Debug_Go_Orign_To_Mix(void);
void Debug_Go_Orign_To_Wast1(void);
void Debug_Go_Orign_To_Wast2(void);

void StartMotorParaInit(void);

void MotionHandle(void);

void CAN_Cmd_Haandle(ExecuteCmd_TypeDf *ExecuteCmdData);

//void EEpromTest(void);






































