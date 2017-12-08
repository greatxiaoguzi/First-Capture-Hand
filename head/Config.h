#include "sys.h"
#include "public.h"
#include "can.h"
#include "em_sw.h"
//程序运行模式
#define RUN_MODE     									1   			//运行模式
#define DEBUG_MODE   									0 			//调试模式

#define CONFIG_PARA_OPTION                              0
//EEPROM地址
#define EEPROM_FOR_CUP_DATA_START_ADDR          		0X0000    		//杯盘号，当前抓杯数，当前杯位坐标，混匀器混匀时间等起始地址
#define EEPROM_FOR_WORK_PARA_START_ADDR 				0X001E    		//30 byte机器工作位坐标起始地址
//应答码及版本号
#define TEXT_REC_ACK 									"ACK"
#define TEXT_REC_NACK 									"NACK"
#define TEXT_FIN_ACK 									"OK"
#define TEXT_VERSION_INQUIE 							"V1.0.000"

#define TEXT_FAULT_RESET_TOTAL 							"ERROR000"
#define TEXT_FAULT_RESET_X_AXIS 						"ERROR001"
#define TEXT_FAULT_RESET_Y_AXIS 						"ERROR002"
#define TEXT_FAULT_RESET_Z_AXIS 						"ERROR003"
#define TEXT_FAULT_RESET_MIX 							"ERROR004"
#define TEXT_FAULT_ORIGIN_TO_CAPTURECUP 				"ERROR005"
#define TEXT_FAULT_ORIGIN_TO_HATCHIN 					"ERROR006"
#define TEXT_FAULT_ORIGIN_TO_HATCHOUT 					"ERROR007"
#define TEXT_FAULT_CUP_TO_HATCHIN 						"ERROR008"
#define TEXT_FAULT_CUP_TO_HATCHOUT 						"ERROR009"
#define TEXT_FAULT_HATCH_TO_CUP 						"ERROR010"
#define TEXT_FAULT_HATCH_TO_MIX 						"ERROR011"
#define TEXT_FAULT_MIX_TO_HATCHIN 						"ERROR012"
#define TEXT_FAULT_MIX_TO_HATCHOUT 						"ERROR013"
#define TEXT_FAULT_HATCH_TO_WASTE 						"ERROR014"
#define TEXT_FAULT_WASTE_TO_HATCHIN 					"ERROR015"
#define TEXT_FAULT_WASTE_TO_HATCHOUT 					"ERROR016"
#define TEXT_FAULT_ENABLE_CABIN 						"ERROR017"
#define TEXT_FAULT_MIX_RUNNING 							"ERROR018"
#define TEXT_FAULT_X_TO_CAPTURE 						"ERROR100"
#define TEXT_FAULT_X_TO_HATCHIN 						"ERROR101"
#define TEXT_FAULT_X_TO_HATCHOUT 						"ERROR102"
#define TEXT_FAULT_X_TO_MIX 							"ERROR103"
#define TEXT_FAULT_X_TO_WASTE1 							"ERROR104"
#define TEXT_FAULT_X_TO_WASTE2 							"ERROR105"
#define TEXT_FAULT_Y_TO_CAPTURE 						"ERROR201"
#define TEXT_FAULT_Y_TO_HATCHIN 						"ERROR202"
#define TEXT_FAULT_Y_TO_HATCHOUT						"ERROR203"
#define TEXT_FAULT_Y_TO_MIX 							"ERROR204"
#define TEXT_FAULT_Y_TO_WASTE1 							"ERROR205"
#define TEXT_FAULT_Y_TO_WASTE2 							"ERROR206"
#define TEXT_FAULT_Z_TO_CUP 							"ERROR302"
#define TEXT_FAULT_Z_TO_HATCH 							"ERROR303"
#define TEXT_FAULT_Z_TO_MIX 							"ERROR304"
#define TEXT_FAULT_Z_TO_WASTE 							"ERROR305"
#define TEXT_FAULT_FINGER_OPEN 							"ERROR306"
#define TEXT_FAULT_CAPTURE_FAIL							"ERROR401"

/********************************************************************************************/
//非动作指令
/**********************************************************************************************/

#define CAPTURE_UNLOAD_CUP_HIGH_DIFFERENCE          	100   		//抓卸杯高度差值补偿

#define MOTOR_FIXED_STEP        						200         //电机固定的一圈的STEP个数
#define X_LEVEL											8			//X后方向上细分数
#define Y_LEVEL											8			//Y轴方向上细分数
#define Z_LEVEL											8			//Z轴方向上细分数
#define MIX_LEVEL										8			//混匀器电机细分数
#define CUP_TOTAL_NUM     								336  		//杯子总数量
#define WASTE_MAX_NUM     								168   		//一个垃圾桶最大可容纳的杯子数量
#define CUP_SPACE               						12   		//杯间距(单位：mm)
#define X_DIR_CUP_NUM           						8     		//X方向杯子数量(单位：个)
#define Y_DIR_CUP_NUM									14    		//Y方向杯子数量(单位：个)

/*********************************************************/
#define CanDataHandle     								can_flag

//第一抓杯手所用  所有电磁阀宏替换
#define  SW_FINGER_ENABLE  								EM_SW_OUT(EM_SW_CH_A,EM_SW_STATUS_ON)   //SW_CtrlPara(Para) 使能手指电磁铁(后面可以用参数控制调节函数来替换)
#define  SW_FINGER_DISABLE 								EM_SW_OUT(EM_SW_CH_A,EM_SW_STATUS_OFF)  //SW_CtrlPara(Para) 失能手指电磁铁(后面可以用参数控制调节函数来替换)
#define  SW_CABIN_ENABLE     							EM_SW_OUT(EM_SW_CH_B,EM_SW_STATUS_ON)   //使能抽屉电磁铁
#define  SW_CABIN_DISABLE    							EM_SW_OUT(EM_SW_CH_B,EM_SW_STATUS_OFF)  //失能抽屉电磁铁

//第一抓杯手所用光电开关宏替换
#define  PCT_X_AXIS      								PCT1
#define  PCT_Y_AXIS      								PCT2
#define  PCT_Z_AXIS      								PCT3
#define  PCT_MIX      									PCT4
#define  PCT_FINGER         							PCT5
#define  PCT_WASTE1       								PCT6
#define  PCT_WASTE2       								PCT7
#define  PCT_CABIN         								PCT8

#define  ENCODER_X_AXIS         						PCT_ENCODER_NO
#define  ENCODER_Y_AXIS									PCT_ENCODER_NO
/********************************************************/
#define UNLOAD_RISE_TIME          						100 		//下降后到卸杯上升时间的时间间隔
#define CAPTURE_UNLOAD_CUP_HEIGHT_DIF                   800         //抓卸杯高度差，单位(脉冲)
#define Z_PCT_ZERO_RUN_PULSE      						(8*30)    	//触发到光电开关后需要减速运行的脉冲数 
#define Y_PCT_ZERO_RUN_PULSE      						(8*11)
#define X_PCT_ZERO_RUN_PULSE      						(8*11)
#define MIX_PCT_ZERO_RUN_PULSE    						(8*20)
#define Z_PER_FP_PULSE_NUM        						8      		//每个频率点要走得到脉冲数
#define Y_PER_FP_PULSE_NUM        						8 
#define X_PER_FP_PULSE_NUM        						8 
#define MIX_PER_FP_PULSE_NUM      						8  

//杯架1校准杯号
#define CUP_DISH1_LEFT_TOP_CUP_NUM						225
#define CUP_DISH1_RIGHT_TOP_CUP_NUM						238
#define CUP_DISH1_LEFT_BOTTOM_CUP_NUM					323
#define CUP_DISH1_RIGHT_BOTTOM_CUP_NUM					336
//杯架2校准杯号
#define CUP_DISH2_LEFT_TOP_CUP_NUM						113	
#define CUP_DISH2_RIGHT_TOP_CUP_NUM						126
#define CUP_DISH2_LEFT_BOTTOM_CUP_NUM					211
#define CUP_DISH2_RIGHT_BOTTOM_CUP_NUM					224
//杯架3校准杯号
#define CUP_DISH3_LEFT_TOP_CUP_NUM						1
#define CUP_DISH3_RIGHT_TOP_CUP_NUM						14
#define CUP_DISH3_LEFT_BOTTOM_CUP_NUM					99
#define CUP_DISH3_RIGHT_BOTTOM_CUP_NUM 					112

#define Z_LARGE_ACC_PULSE 		  						464    		//加速阶段要执行的脉冲数
#define Z_LARGE_DEC_PULSE		  						464    		//减速阶段要执行的脉冲数
#define Y_LARGE_ACC_PULSE 		  						472    		//加速阶段要执行的脉冲数
#define Y_LARGE_DEC_PULSE		  						472    		//减速阶段要执行的脉冲数
#define X_LARGE_ACC_PULSE 		  						472    		//加速阶段要执行的脉冲数
#define X_LARGE_DEC_PULSE		  						472    		//减速阶段要执行的脉冲数
#define MIX_LARGE_ACC_PULSE       						472    		//混匀盘加速阶段需要执行的脉冲数
#define MIX_LARGE_DEC_PULSE		  						472    		//混匀盘减速阶段需要执行的脉冲数

#define Z_SHORT_ACC_PULSE 		  						72    		//加速阶段要执行的脉冲数
#define Z_SHORT_DEC_PULSE		  						72    		//减速阶段要执行的脉冲数
#define Y_SHORT_ACC_PULSE 		  						88    		//加速阶段要执行的脉冲数
#define Y_SHORT_DEC_PULSE		  						88    		//减速阶段要执行的脉冲数
#define X_SHORT_ACC_PULSE 		  						88    		//加速阶段要执行的脉冲数
#define X_SHORT_DEC_PULSE		  						88    		//减速阶段要执行的脉冲数
#define MIX_SHORT_ACC_PULSE       						96    		//混匀盘加速阶段需要执行的脉冲数
#define MIX_SHORT_DEC_PULSE		  						96 

#define MIX_DEFAULT_MAX_PULSE     						3000		//混匀电机最大运行的脉冲数(最后相当于时间)

#define CAN_MAIN_MACHINE_ID       						0X00		//CAN主机ID好


















































































