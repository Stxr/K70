/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			TtlBusUnit.h
** 最后修订日期:		2009-03-23
** 最后版本:			1.0
** 描述:				使用DynamixelProtocol协议栈的UART接口的操作方法接口
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-19
** 版本:				1.0
** 描述:				使用DynamixelProtocol协议栈的UART接口的操作方法接口
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
#ifndef TTLBUSUNIT_H_
#define TTLBUSUNIT_H_

#include "ConfigTypes.h"

#include "Drivers/Timer.h"
#include "Drivers/Led.h"
#include "Drivers/Ppm.h"
#include "Drivers/Uart.h"

#include "Lib/DynamixelProtocol.h"
#include "Lib/Mapping.h"

#define		TTLBUSUNIT_GLOBALS

#ifndef		TTLBUSUNIT_GLOBALS
     #define TTLBUSUNIT_EXT
#else 
     #define TTLBUSUNIT_EXT  extern
#endif 


/*************************************************************************************************************************
                                                         结构体定义
*************************************************************************************************************************/


typedef struct
{
	BSU_STACK_STR_STRUCT				str_bsu_stack;
	BSU_SYNC_SEND_STRUCT				str_bsu_sync;
	UART_CONTROL_STRUCT*				p_uart_control;
	
	uint8*								p_device_mapping;
	uint8								device_amount;
	uint8								sync_update_state;

	tUint8Puint8Uint8*					pInitSync;
	tUint8Uint8Uint8Uint16Uint16*		pWriteRegWord2;
	tUint8Uint8Uint8Uint16Uint16*		pWriteWord2;
	tUint8Uint8Uint8Uint16*				pWriteRegWord;
	tUint8Uint8Uint8Uint16*				pWriteWord;
	tUint8Uint8Uint8Uint8*				pWriteByte;
	tVoidUint8Uint8*					pWriteSyncControl;
	tUint8Uint8Uint16Uint16*			pWriteSyncWord2;
	tVoidVoid*							pActionSync;
	tUint8Void*							pAction;
	tUint8Uint8Uint8Puint16*			pReadWord;
	tUint8Uint8Uint8Puint16Puint16*		pReadWord2;
	tUint8Uint8Puint8*					pPing;

	uint8 								state;					// 节点状态，该寄存器反映成功访问的最后一个节点的节点状态
}TTLBUS_CONTROL_STRUCT;


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
TTLBUSUNIT_EXT TTLBUS_CONTROL_STRUCT ttlbus_control;

/*************************************************************************************************************************
                                                         函数声明
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
