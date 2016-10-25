/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			ServoUnit.h
** 最后修订日期:		2009-03-23
** 最后版本:			1.0
** 描述:				舵机命令执行器，用于解析执行舵机功能模块的指令，统领PPM与利用DynamixelProtocol协议栈的UART接口。
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-19
** 版本:				1.0
** 描述:				执行的具体功能函数的参数传递与返回值的封装。
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			律晔
** 修订日期:			2009-10-28
** 版本:				2.0
** 描述:				将总线操作部分从舵机操作函数中分离出来
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			
** 修订日期:	    
** 版本:		    
** 描述:            
**
*************************************************************************************************************************/
#ifndef SERVOUNIT_H_
#define SERVOUNIT_H_

#include "ConfigTypes.h"

#include "Drivers/Timer.h"
#include "Drivers/Led.h"
#include "Drivers/Ppm.h"
#include "Drivers/Uart.h"

#include "Lib/DynamixelProtocol.h"
#include "Lib/Mapping.h"

#include "Shell/TtlBusUnit.h"


#define		SERVOUNIT_GLOBALS

#ifndef   SERVOUNIT_GLOBALS
     #define SERVOUNIT_EXT     
#else 
     #define SERVOUNIT_EXT  extern
#endif 



/*************************************************************************************************************************
                                                       模式控制字定义
*************************************************************************************************************************/
#define SERVO_MODE_SERVO				0x00
#define SERVO_MODE_MOTO					0x01

#define SERVO_EN						0x01			// 使能
#define SERVO_DIS						0x00			// 不使能

#define SERVO_PPM_ID					0x0E
/*************************************************************************************************************************
                                                         结构体定义
*************************************************************************************************************************/


typedef struct
{
	PPM_CONTROL_STRUCT*			p_ppm_control;

	uint16						last_position[BSU_MAX_SYNC_DEVICE_LENGTH];
	uint16						last_velocity[BSU_MAX_SYNC_DEVICE_LENGTH];

	uint8						node_mapping[32];

	tUint8Uint8Puint8*			pTest;
	tUint8Uint8Uint8*			pSetMode;
	tUint8Uint8Uint8*			pSetTorqueEnable;

	tUint8Uint8Uint16Uint16*	pSetPosition;
	tUint8Uint8Uint16Uint16*	pSetRegPosition;
	tUint8Uint8Uint16Uint16*	pSetSyncPosition;

	tUint8Uint8Uint8Uint16*		pSetVelocity;
	tUint8Uint8Uint8Uint16*		pSetRegVelocity;
	tUint8Uint8Uint8Uint16*		pSetSyncVelocity;

	tUint8Uint8Uint16Uint16*	pSetAngleLimit;
	tUint8Uint8Uint8*			pSetId;

	tUint8Uint8Puint16*			pGetPosition;
	tUint8Uint8Puint16*			pGetLoad;
	tUint8Uint8Puint16Puint16*	pGetAngleLimit;

	tUint8Void*					pAction;
	tUint8Void*					pActionSync;
	tVoidPuint8Uint8*			pInitNodeBuffer;

	tUint8Uint8Uint16Uint16Puint16Puint16* pCalcVelocity;
}SERVO_CONTROL_STRUCT;


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
SERVOUNIT_EXT SERVO_CONTROL_STRUCT servo_control;

/*************************************************************************************************************************
                                                         函数声明
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
