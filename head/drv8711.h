#ifndef __DRV8711_H_
#define __DRV8711_H_

#include "spi.h"
#include "public.h"

#define DUMMY		0x00

#define R_SENSE		0.05			//DRV8711²ÉÑùµç×èÖµ£¬µ¥Î»:Å·Ä·

//DRV8711¶ÁÐ´Î»¿ØÖÆ
#define DRV_WRITE					(u16)(0 << 15)
#define DRV_READ					(u16)(1 << 15)

//¼Ä´æÆ÷µØÖ·
#define DRV_CTRL_ADDR				(u16)(0 << 12)
#define DRV_TORQUE_ADDR				(u16)(1 << 12)
#define DRV_OFF_ADDR				(u16)(2 << 12)
#define DRV_BLANK_ADDR				(u16)(3 << 12)
#define DRV_DECAY_ADDR				(u16)(4 << 12)
#define DRV_STALL_ADDR				(u16)(5 << 12)
#define DRV_DRIVER_ADDR				(u16)(6 << 12)
#define DRV_STATUS_ADDR				(u16)(7 << 12)

//CTRL¼Ä´æÆ÷
#define DRV_CTRL_ENABLE				(u16)(1 << 0)
#define DRV_CTRL_DISABLE			(u16)(0 << 0)		//Ä¬ÈÏ×´Ì¬

#define DRV_CTRL_DIR_FORWARD		(u16)(0 << 1)
#define DRV_CTRL_DIR_INVERT			(u16)(1 << 1)		//Ä¬ÈÏ×´Ì¬

#define DRV_CTRL_RSTEP_NO_ACTION	(u16)(0 << 2)		//Ä¬ÈÏ×´Ì¬
#define DRV_CTRL_RSTEP_ADVANCE		(u16)(1 << 2)

#define DRV_CTRL_MODE_FULL_STP		(u16)(0 << 3)
#define DRV_CTRL_MODE_2_STP			(u16)(1 << 3)
#define DRV_CTRL_MODE_4_STP			(u16)(2 << 3)		//Ä¬ÈÏ×´Ì¬
#define DRV_CTRL_MODE_8_STP			(u16)(3 << 3)
#define DRV_CTRL_MODE_16_STP		(u16)(4 << 3)
#define DRV_CTRL_MODE_32_STP		(u16)(5 << 3)
#define DRV_CTRL_MODE_64_STP		(u16)(6 << 3)
#define DRV_CTRL_MODE_128_STP		(u16)(7 << 3)
#define DRV_CTRL_MODE_256_STP		(u16)(8 << 3)
#define DRV_LEVEL_FULL				DRV_CTRL_MODE_FULL_STP
#define DRV_LEVEL_2					DRV_CTRL_MODE_2_STP
#define DRV_LEVEL_4					DRV_CTRL_MODE_4_STP
#define DRV_LEVEL_8					DRV_CTRL_MODE_8_STP
#define DRV_LEVEL_16				DRV_CTRL_MODE_16_STP
#define DRV_LEVEL_32				DRV_CTRL_MODE_32_STP
#define DRV_LEVEL_64				DRV_CTRL_MODE_64_STP
#define DRV_LEVEL_128				DRV_CTRL_MODE_128_STP
#define DRV_LEVEL_256				DRV_CTRL_MODE_256_STP

#define DRV_CTRL_STALL_DETECT_IN	(u16)(0 << 7)		//Ä¬ÈÏ×´Ì¬
#define DRV_CTRL_STALL_DETECT_EXT	(u16)(1 << 7)

#define DRV_CTRL_GAIN_5				(u16)(0 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_CTRL_GAIN_10			(u16)(1 << 8)
#define DRV_CTRL_GAIN_20			(u16)(2 << 8)
#define DRV_CTRL_GAIN_40			(u16)(3 << 8)
#define DRV_GAIN_5					DRV_CTRL_GAIN_5
#define DRV_GAIN_10					DRV_CTRL_GAIN_10
#define DRV_GAIN_20					DRV_CTRL_GAIN_20
#define DRV_GAIN_40					DRV_CTRL_GAIN_40

#define DRV_CTRL_DEAD_TIME_400ns	(u16)(0 << 10)
#define DRV_CTRL_DEAD_TIME_450ns	(u16)(1 << 10)
#define DRV_CTRL_DEAD_TIME_650ns	(u16)(2 << 10)
#define DRV_CTRL_DEAD_TIME_850ns	(u16)(3 << 10)		//Ä¬ÈÏ×´Ì¬

//TORQUE¼Ä´æÆ÷
#define DRV_TOQUE_DEFAULT			(u16)(0xff << 0)

#define DRV_TORQUE_SMPLTH_50us		(u16)(0 << 8)
#define DRV_TORQUE_SMPLTH_100us		(u16)(1 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_TORQUE_SMPLTH_200us		(u16)(2 << 8)
#define DRV_TORQUE_SMPLTH_300us		(u16)(3 << 8)
#define DRV_TORQUE_SMPLTH_400us		(u16)(4 << 8)
#define DRV_TORQUE_SMPLTH_600us		(u16)(5 << 8)
#define DRV_TORQUE_SMPLTH_800us		(u16)(6 << 8)
#define DRV_TORQUE_SMPLTH_1000us	(u16)(7 << 8)

//OFF ¼Ä´æÆ÷
#define DRV_OFF_TOFF_DEFAULT		(u16)(0x30 << 0)

#define DRV_OFF_PWN_MODE_IN_INDEXER	(u16)(0 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_OFF_PWN_MODE_BY_INDEXER	(u16)(1 << 8)

//BALNK¼Ä´æÆ÷
#define DRV_BLANK_TBLANK_DEFAULT	(u16)(0x80 << 0)

#define DRV_BLANK_ABT_DISABLE		(u16)(0 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_BLANK_ABT_ENABLE		(u16)(1 << 8)

//DECAY¼Ä´æÆ÷
#define DRV_DECAY_TDECAY_DEFAULT	(u16)(0x10 << 0)

#define DRV_DECAY_DECMOD_SLOW		(u16)(0 << 8)
#define DRV_DECAY_DECMOD_1			(u16)(1 << 8)		//Ä¬ÈÏ×´Ì¬£¬µçÁ÷Ôö¼ÓÊ±ÂýË¥¼õ£¬µçÁ÷¼õÐ¡Ê±»ìºÏË¥¼õ
#define DRV_DECAY_DECMOD_FAST		(u16)(2 << 8)
#define DRV_DECAY_DECMOD_MIX		(u16)(3 << 8)
#define DRV_DECAY_DECMOD_2			(u16)(4 << 8)		//µçÁ÷Ôö¼ÓÊ±ÂýË¥¼õ£¬µçÁ÷¼õÐ¡Ê±×Ô¶¯»ìºÏË¥¼õ
#define DRV_DECAY_DECMOD_AUTO_MIX	(u16)(5 << 8)
#define DRV_DECAY_SLOW				DRV_DECAY_DECMOD_SLOW
#define DRV_DECAY_1					DRV_DECAY_DECMOD_1
#define DRV_DECAY_FAST				DRV_DECAY_DECMOD_FAST
#define DRV_DECAY_MIX				DRV_DECAY_DECMOD_MIX
#define DRV_DECAY_2					DRV_DECAY_DECMOD_2
#define DRV_DECAY_AUTO_MIX			DRV_DECAY_DECMOD_AUTO_MIX

//STALL¼Ä´æÆ÷
#define DRV_STALL_SDTHR_DEFAULT		(u16)(0x40 << 0)

#define DRV_STALL_SDCNT_1_STEP		(u16)(0 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_STALL_SDCNT_2_STEP		(u16)(1 << 8)
#define DRV_STALL_SDCNT_4_STEP		(u16)(2 << 8)
#define DRV_STALL_SDCNT_8_STEP		(u16)(3 << 8)

#define DRV_STALL_VDIV_32			(u16)(0 << 10)		//Ä¬ÈÏ×´Ì¬
#define DRV_STALL_VDIV_16			(u16)(1 << 10)
#define DRV_STALL_VDIV_8			(u16)(2 << 10)
#define DRV_STALL_VDIV_4			(u16)(3 << 10)

//DRIVE¼Ä´æÆ÷
#define DRV_DRIVER_OCPTH_250mV		(u16)(0 < 0)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_OCPTH_500mV		(u16)(1 < 0)
#define DRV_DRIVER_OCPTH_750mV		(u16)(2 < 0)
#define DRV_DRIVER_OCPTH_1000mV		(u16)(3 < 0)

#define DRV_DRIVER_OCPDEG_1us		(u16)(0 < 2)
#define DRV_DRIVER_OCPDEG_2us		(u16)(1 < 2)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_OCPDEG_4us		(u16)(2 < 2)
#define DRV_DRIVER_OCPDEG_8us		(u16)(3 < 2)

#define DRV_DRIVER_TDRIVEN_250ns	(u16)(0 << 4)
#define DRV_DRIVER_TDRIVEN_500ns	(u16)(1 << 4)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_TDRIVEN_1us		(u16)(2 << 4)
#define DRV_DRIVER_TDRIVEN_2us		(u16)(3 << 4)

#define DRV_DRIVER_TDRIVEP_250ns	(u16)(0 << 6)
#define DRV_DRIVER_TDRIVEP_500ns	(u16)(1 << 6)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_TDRIVEP_1us		(u16)(2 << 6)
#define DRV_DRIVER_TDRIVEP_2us		(u16)(3 << 6)

#define DRV_DRIVER_IDRIVEN_100mA	(u16)(0 << 8)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_IDRIVEN_200mA	(u16)(1 << 8)
#define DRV_DRIVER_IDRIVEN_300mA	(u16)(2 << 8)
#define DRV_DRIVER_IDRIVEN_400mA	(u16)(3 << 8)

#define DRV_DRIVER_IDRIVEP_50mA		(u16)(0 << 10)		//Ä¬ÈÏ×´Ì¬
#define DRV_DRIVER_IDRIVEP_100mA	(u16)(1 << 10)
#define DRV_DRIVER_IDRIVEP_150mA	(u16)(2 << 10)
#define DRV_DRIVER_IDRIVEP_200mA	(u16)(3 << 10)

void DRV8711_Init(void);
void DRV8711_Self_Lock(MOTOR_CHIP_SELECT sel);
void DRV8711_Dir_Forward(MOTOR_CHIP_SELECT sel);
void DRV8711_Dir_Invert(MOTOR_CHIP_SELECT sel);
MOTOR_DIR DRV8711_Get_Direction(MOTOR_CHIP_SELECT sel);
void DRV8711_Set_Current(MOTOR_CHIP_SELECT sel, u16 mA);
void DRV8711_Enable(MOTOR_CHIP_SELECT sel);
void DRV8711_Disable(MOTOR_CHIP_SELECT sel);
void DRV8711_Set_Index_Level(MOTOR_CHIP_SELECT sel, u16 step);
void DRV8711_Set_Decay(MOTOR_CHIP_SELECT sel, u16 mode);
void DRV8711_Set_Current(MOTOR_CHIP_SELECT sel, u16 mA);
u16 DRV8711_Get_Index_Level(MOTOR_CHIP_SELECT sel);

#endif

