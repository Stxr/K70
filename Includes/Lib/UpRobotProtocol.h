/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			UpRobotProtocol.h
** 最后修订日期:		2009-03-16
** 最后版本:			1.0
** 描述:				博创兴盛机器人技术有限公司定义的机器人控制UART通讯协议的协议栈
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-16
** 版本:				1.0
** 描述:				用于生成控制结构体并写发送缓冲；
**					处理UART接收缓冲区的数据，生成状态结构体；
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
*************************************************************************************************************************/
#ifndef UPROBOTPROTOCOL_H_
#define UPROBOTPROTOCOL_H_


#include "ConfigTypes.h"
#include "Drivers/Uart.h"
#include "Drivers/Led.h"

#define		UPROBOTPROTOCOL_GLOBALS

#ifndef   UPROBOTPROTOCOL_GLOBALS
     #define UPROBOTPROTOCOL_EXT     
#else 
     #define UPROBOTPROTOCOL_EXT  extern
#endif 




/*************************************************************************************************************************
**                                                    协议相关控制字
*************************************************************************************************************************/
#define UPROBOT_START1 							0x55
#define UPROBOT_START2 							0xAA


#define UPROBOT_MSSR_MAX_PARAMETER_LENGTH		0x96			// 最长参数定义
																// 由于数传电台的数据包长度为150bit，所以一次发送长度
																// 需要小于15byte，所以数据区长度不能大于15 - 7 = 8
#define UPROBOT_MRSS_MAX_PARAMETER_LENGTH		0x07



/*************************************************************************************************************************
**                                                     结构体定义
*************************************************************************************************************************/
typedef struct
{
	uint8 device_id;											// 设备ID
	uint8 functional_unit;										// 功能单元代号
	uint8 method_code;											// 方法代号
	uint8 state_byte;											// 状态标志字
	uint8 parameter_length;										// 参数长度

	uint8 parameter[UPROBOT_MRSS_MAX_PARAMETER_LENGTH];			// 参数数组
}UPROBOT_MRSS_STRUCT;											// UpRobot主机接收从机发送结构体


typedef struct
{
	uint8 device_id;											// 设备ID
	uint8 functional_unit;										// 功能单元代号
	uint8 method_code;											// 方法代号
	
	uint8 parameter_length;										// 参数长度
	uint8 parameter[UPROBOT_MSSR_MAX_PARAMETER_LENGTH];			// 参数数组
}UPROBOT_MSSR_STRUCT;											// UpRobot主机发送从机接收结构体


typedef uint8 (tUpRobotSlaveRxPacket)(UPROBOT_MSSR_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control);
typedef uint8 (tUpRobotSlaveTxPacket)(UPROBOT_MRSS_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control);
typedef uint8 (tUpRobotMasterTxPacket)(UPROBOT_MSSR_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control);
typedef uint8 (tUpRobotMasterRxPacket)(UPROBOT_MRSS_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control);


typedef struct
{
	tUpRobotMasterRxPacket* pUpRobotMasterRxPacket;				// 函数指针定义
	tUpRobotMasterTxPacket* pUpRobotMasterTxPacket;
	tUpRobotSlaveRxPacket* pUpRobotSlaveRxPacket;				// 函数指针定义
	tUpRobotSlaveTxPacket* pUpRobotSlaveTxPacket;
}UPROBOT_CONTROL_STRUCT;


typedef struct
{
	UPROBOT_MSSR_STRUCT str_slave_receive;						// 建立接收结构体
	UPROBOT_MRSS_STRUCT str_slave_send;							// 建立发送结构体
	UPROBOT_MRSS_STRUCT str_master_receive;						// 建立接收结构体
	UPROBOT_MSSR_STRUCT str_master_send;						// 建立发送结构体
}UPROBOT_STACK_STR_STRUCT;

/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
UPROBOTPROTOCOL_EXT UPROBOT_CONTROL_STRUCT uprobot_control;



#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
