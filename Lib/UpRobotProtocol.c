/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			UpRobotProtocol.c
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
#include "Lib/UpRobotProtocol.h"


#define UPROBOT_RX_TIMEOUT_COUNT 		1400


/*************************************************************************************************************************
** 函数名称:			UpRobotSlaveRxPacket
**
** 函数描述:			UpRobot从机字段解析(Parser)
**                      
**                      
**					                 
** 输入变量:			UPROBOT_RECEIVE_SLAVE_STRUCT* p_uprobot_receive;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotSlaveRxPacket(UPROBOT_MSSR_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control)
{	
	uint8 count = 0;													// 循环次数计数器
	uint32 timeout_count = 0;											// 等待超时计数器

	uint8 temp_buffer = 0;												// 数据缓存
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (UPROBOT_MSSR_MAX_PARAMETER_LENGTH + 7); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())					// 如果接收数据不成功(FALSE)则开始计时
		{
			timeout_count++;
			if(timeout_count > UPROBOT_RX_TIMEOUT_COUNT)
			{
				return ERR_OVERFLOW;									// 等待超时
			}
		}
		
		(p_uart_control->pReceiveByte)(&temp_buffer);

		switch(count)
		{
			case 1:														// 匹配启动字1
			{
				if(UPROBOT_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:														// 匹配启动字2
			{		
				if(UPROBOT_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:														// 设备ID号
			{
				p_uprobot_receive->device_id = temp_buffer;
				break;
			}
			case 4:														// 数据包长度
			{
				p_uprobot_receive->parameter_length = temp_buffer ;
				break;
			}
			case 5:														// 功能单元编号
			{
				p_uprobot_receive->functional_unit = temp_buffer;
				break;
			}
			case 6:														// 方法编号
			{
				p_uprobot_receive->method_code = temp_buffer;
				break;
			}
			default:
			{
				if(count < (7 + p_uprobot_receive->parameter_length))
				{
					p_uprobot_receive->parameter[count - 7] = temp_buffer;
				}
				else
				{
					check_sum = UPROBOT_START1;							// 计算检验和
					check_sum += UPROBOT_START2;
					check_sum += p_uprobot_receive->device_id;			
					check_sum += p_uprobot_receive->parameter_length;
					check_sum += p_uprobot_receive->functional_unit;
					check_sum += p_uprobot_receive->method_code;
					for(check_sum_count = 0; check_sum_count < (p_uprobot_receive->parameter_length); check_sum_count++)
					{
						check_sum += p_uprobot_receive -> parameter[check_sum_count];
					}
					
					if(check_sum == temp_buffer)
					{
						return ERR_OK;									// 无错误
					}
					else
					{
						//return ERR_CRC;									// 校验码错误
						return ERR_OK;
					}
				}
				break;	
			}			
		}	 
	}
	
	return ERR_MATH;													// 无效缓冲数据溢出
}


/*************************************************************************************************************************
** 函数名称:			UpRobotSlaveTxPacket
**
** 函数描述:			UpRobot从机数据包的组包(To String)、发送
**                      
**                      
**					                 
** 输入变量:			UPROBOT_MRSS_STRUCT* p_uprobot_send; UART_CONTROL_STRUCT* p_uart_control
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotSlaveTxPacket(UPROBOT_MRSS_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START1);									// 开始位1、2
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START2);
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->device_id);						// 设备ID
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter_length);					// 参数长度
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->functional_unit);					// 功能单元代号
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->method_code);						// 方法代号
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->state_byte);						// 状态标志
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter[count]);				// 写数据组
	}
	
	check_sum = UPROBOT_START1;
	check_sum += UPROBOT_START2;
	check_sum += p_uprobot_send->device_id;
	check_sum += p_uprobot_send->parameter_length;
	check_sum += p_uprobot_send->functional_unit;
	check_sum += p_uprobot_send->method_code;
	check_sum += p_uprobot_send->state_byte;
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		check_sum += p_uprobot_send->parameter[count];									// 校验和计算
	}

	(p_uart_control->pWriteTxBuffer)(check_sum);
	
	p_uart_control->pSendTxBuffer();													// 发送缓冲数据
	
	return ERR_OK;
}
	
	
/*************************************************************************************************************************
** 函数名称:			UpRobotMasterRxPacket
**
** 函数描述:			UpRobot主机字段解析(Parser)
**                                       
**					                 
** 输入变量:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotMasterRxPacket(UPROBOT_MRSS_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control)
{	
	uint8 count = 0;														// 循环次数计数器
	uint32 timeout_count = 0;												// 等待超时计数器

	uint8 temp_buffer = 0;													// 数据缓存
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (UPROBOT_MRSS_MAX_PARAMETER_LENGTH + 8); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())					// 如果接收数据不成功(FALSE)则开始计时
		{
			timeout_count++;
			if(timeout_count > UPROBOT_RX_TIMEOUT_COUNT)
			{
				return ERR_OVERFLOW;									// 等待超时
			}
		}
		
		cli();
		(p_uart_control->pReceiveByte)(&temp_buffer);
		sei();

		switch(count)
		{
			case 1:															// 匹配启动字1
			{
				if(UPROBOT_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:															// 匹配启动字2
			{		
				if(UPROBOT_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:															// 设备ID号
			{
				p_uprobot_receive->device_id = temp_buffer;
				break;
			}
			case 4:															// 数据包长度
			{
				p_uprobot_receive->parameter_length = temp_buffer ;
				break;
			}
			case 5:															// 状态字
			{
				p_uprobot_receive->state_byte  = temp_buffer;
				break;
			}
			default:
			{
				if(count < (6 + p_uprobot_receive->parameter_length))
				{
					p_uprobot_receive->parameter[count - 7] = temp_buffer;
				}
				else
				{
					check_sum = UPROBOT_START1;								// 计算检验和
					check_sum += UPROBOT_START2;
					check_sum += p_uprobot_receive->device_id;			
					check_sum += p_uprobot_receive->parameter_length;
					check_sum += p_uprobot_receive->state_byte;
					for(check_sum_count = 0; check_sum_count < (p_uprobot_receive->parameter_length); check_sum_count++)
					{
						check_sum += p_uprobot_receive -> parameter[check_sum_count];
					}
					
					if(check_sum == temp_buffer)
					{
						return ERR_OK;										// 无错误
					}
					else
					{
						return ERR_CRC;										// 校验码错误
					return ERR_OK;
					}
				}
				break;	
			}			
		}	 
	}
	
	
	return ERR_MATH;														// 无效缓冲数据溢出
}


/*************************************************************************************************************************
** 函数名称:			UpRobotMasterTxPacket
**
** 函数描述:			UpRobot主机数据包的组包(To String)、发送
**                                       
**					                 
** 输入变量:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotMasterTxPacket(UPROBOT_MSSR_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START1);									// 开始位1、2
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START2);
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->device_id);						// 设备ID
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter_length);					// 参数长度
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->functional_unit);					// 功能模块编号
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->method_code);						// 方法编号
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter[count]);				// 写数据组
	}
	
	check_sum = UPROBOT_START1;
	check_sum += UPROBOT_START2;
	check_sum += p_uprobot_send->device_id;
	check_sum += p_uprobot_send->parameter_length;
	check_sum += p_uprobot_send->functional_unit;
	check_sum += p_uprobot_send->method_code;
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		check_sum += p_uprobot_send->parameter[count];									// 校验和计算
	}

	(p_uart_control->pWriteTxBuffer)(check_sum);
	
	p_uart_control->pSendTxBuffer();													// 发送缓冲数据
	
	return ERR_OK;
}


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
UPROBOT_CONTROL_STRUCT uprobot_control = 
{
	.pUpRobotMasterRxPacket = UpRobotMasterRxPacket,
	.pUpRobotMasterTxPacket = UpRobotMasterTxPacket,
	.pUpRobotSlaveRxPacket = UpRobotSlaveRxPacket,
	.pUpRobotSlaveTxPacket = UpRobotSlaveTxPacket,
};


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
