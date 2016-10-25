/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			DynamixelProtocol.h
** 最后修订日期:		2009-03-11
** 最后版本:			1.0
** 描述:				Robotis公司定义的Dynamixel总线伺服器的协议的协议栈
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-11
** 版本:				1.0
** 描述:				 用于生成控制结构体并写发送缓冲；
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
#ifndef DYNAMIXELPROTOCOL_H_
#define DYNAMIXELPROTOCOL_H_


#include "ConfigTypes.h"

#include "ConfigTypes.h"
#include "Drivers/Uart.h"
#include "Drivers/Led.h"


#define		DYNAMIXELPROTOCOL_GLOBALS

#ifndef   DYNAMIXELPROTOCOL_GLOBALS
     #define DYNAMIXELPROTOCOL_EXT     
#else 
     #define DYNAMIXELPROTOCOL_EXT  extern
#endif 


/*************************************************************************************************************************
**                                                    Instruction
*************************************************************************************************************************/
#define INST_PING 						0x01
#define INST_READ 						0x02
#define INST_WRITE 						0x03
#define INST_REG_WRITE 					0x04
#define INST_ACTION 					0x05
#define INST_RESET 						0x06
#define INST_DIGITAL_RESET 				0x07
#define INST_SYSTEM_READ 				0x0C
#define INST_SYSTEM_WRITE 				0x0D
#define INST_SYNC_WRITE 				0x83
#define INST_SYNC_REG_WRITE 			0x84


/*************************************************************************************************************************
**                                                    协议相关字
*************************************************************************************************************************/
#define BSU_START1 						0xFF
#define BSU_START2 						0xFF


#define BSU_MAX_PARAMETER_LENGTH		0x0F				//最长参数定义

#define BSU_BROADCASTING_ID				0xFE

#define BSU_MAX_SYNC_DEVICE_LENGTH		0x20
#define BSU_MAX_SYNC_PARAMETER_LENGTH	0x04


/*************************************************************************************************************************
**                                                     结构体定义
*************************************************************************************************************************/
typedef struct SBsuSyncSend
{
	uint8 update_amount;												// 设备数据更新数量
	uint8 update_mapping[BSU_MAX_SYNC_DEVICE_LENGTH];					// 设备数据更新映射表

	uint8 first_address;												// 首地址
	uint8 parameter_length;												// 参数长度

	uint8 write_parameter[BSU_MAX_SYNC_DEVICE_LENGTH][BSU_MAX_SYNC_PARAMETER_LENGTH];	// 写寄存器数据
}BSU_SYNC_SEND_STRUCT;													// Dynamixel同步发送控制结构体
/*************************************************************************************************************************
** 输入id号，查询id_mapping映射表device_amount次得到缓冲区号，缓冲区号与update_mapping映射表中的缓冲区号对比，
** 比较循环update_amount次，如果比较匹配，则update_amount不更新，否则将update_mapping[update_amount]储存缓冲
** 区号，update_amount++；将输入的参数写入write_parameter[缓冲区号][];
*************************************************************************************************************************/

typedef struct SBsuSend
{
	uint8 device_id;										// 设备ID
	uint8 instruction_byte;									// 控制字
	uint8 first_address;									// 首地址

	uint8 read_length;										// 读寄存器个数

	uint8 parameter_length;									// 参数长度
	uint8 write_parameter[BSU_MAX_PARAMETER_LENGTH];		// 写寄存器数据
}BSU_SEND_STRUCT;											// Dynamixel发送控制结构体


typedef struct SBsuReceive
{
	uint8 device_id;										// 设备ID
	uint8 error_byte;										// 错误标志字
	uint8 parameter_length;									// 参数长度

	uint8 receive_parameter[BSU_MAX_PARAMETER_LENGTH];		// 寄存器数据
}BSU_RECEIVE_STRUCT;										// Dynamixel接收控制结构体


typedef uint8 (tRxBsuPacket)(BSU_RECEIVE_STRUCT *p_bsu_receive, UART_CONTROL_STRUCT* p_uart_control);
typedef uint8 (tTxBsuPacket)(BSU_SEND_STRUCT* p_bsu_send, UART_CONTROL_STRUCT* p_uart_control);
typedef uint8 tTxBsuSyncPacket(BSU_SYNC_SEND_STRUCT* p_sync_send, const uint8* p_id_mapping, UART_CONTROL_STRUCT* p_uart_control);

typedef struct SBsuControl
{
	tRxBsuPacket* pRxPacket;
	tTxBsuPacket* pTxPacket;
	tTxBsuSyncPacket* pTxSyncPacket;
}BSU_CONTROL_STRUCT;										// Dynamixel控制结构体


typedef struct SBsuStackStr
{
	BSU_RECEIVE_STRUCT bsu_receive;
	BSU_SEND_STRUCT bsu_send;
}BSU_STACK_STR_STRUCT;										// Dynamixel发送接收缓冲结构体


/*************************************************************************************************************************
**                                                     结构体声明
*************************************************************************************************************************/
DYNAMIXELPROTOCOL_EXT BSU_CONTROL_STRUCT bsu_control;


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
