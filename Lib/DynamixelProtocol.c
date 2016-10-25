/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			DynamixelProtocol.c
** 最后修订日期:		2009-03-11
** 最后版本:			1.0
** 描述:				Robotis公司定义的Dynamixel总线伺服器的协议的协议栈
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-11
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
#include "Lib/DynamixelProtocol.h"


/*************************************************************************************************************************
** 函数名称:			TxBsuSyncPacket
**
** 函数描述:			发送总线伺服单元的同步控制数据包的组包(To String)、发送
**
**
** 输入变量:			BSU_SYNC_SEND_STRUCT* p_sync_send, uint8 p_id_mapping, UART_CONTROL_STRUCT* p_uart_control
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 TxBsuSyncPacket(BSU_SYNC_SEND_STRUCT* p_sync_send, const uint8* p_id_mapping, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 id_count = 0;
	uint8 parameter_count = 0;

	uint8 temp_value = 0;
	uint8 check_sum = 0;


	if (0 == p_sync_send->update_amount)											// 清除更新标志
	{
		return ERR_VALUE;
	}

	(p_uart_control->pWriteTxBuffer)(BSU_START1);									// 开始位1、2
	(p_uart_control->pWriteTxBuffer)(BSU_START2);
	(p_uart_control->pWriteTxBuffer)(BSU_BROADCASTING_ID);							// 设备ID

	check_sum = BSU_BROADCASTING_ID;

	temp_value = ((p_sync_send->parameter_length + 1) *  p_sync_send->update_amount) + 4;

	(p_uart_control->pWriteTxBuffer)(temp_value);									// 数据长度

	check_sum += temp_value;

	(p_uart_control->pWriteTxBuffer)(INST_SYNC_WRITE);								// 控制字

	check_sum += INST_SYNC_WRITE;

	(p_uart_control->pWriteTxBuffer)(p_sync_send->first_address);					// 寄存器首地址

	check_sum += p_sync_send->first_address;

	(p_uart_control->pWriteTxBuffer)(p_sync_send->parameter_length);				// 同步控制的参数个数

	check_sum += p_sync_send->parameter_length;

	for (id_count = 0; id_count < (p_sync_send->update_amount); id_count++)
	{
		temp_value = *(p_id_mapping + p_sync_send->update_mapping[id_count]);
		(p_uart_control->pWriteTxBuffer)(temp_value);								// ID
		check_sum += temp_value;

		for (parameter_count = 0; parameter_count < (p_sync_send->parameter_length); parameter_count++)
		{
			temp_value = p_sync_send->write_parameter[p_sync_send->update_mapping[id_count]][parameter_count];
			(p_uart_control->pWriteTxBuffer)(temp_value);							// 参数
			check_sum += temp_value;
		}
	}

	check_sum = ~check_sum;															// 校验和计算

	(p_uart_control->pWriteTxBuffer)(check_sum);									// 校验和

	p_sync_send->update_amount = 0;													// 清除更新标志

	(p_uart_control->pSendTxBuffer)();												// 发送缓冲数据

	return ERR_OK;
}


/*************************************************************************************************************************
** 函数名称:			RxBsuPacket
**
** 函数描述:			字段解析(Parser)
**
**
**
** 输入变量:			BSU_RECEIVE_CONTROL_STRUCT *p_bsu_receive;
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
static uint8 RxBsuPacket(BSU_RECEIVE_STRUCT *p_bsu_receive, UART_CONTROL_STRUCT* p_uart_control)
{
	#define BSU_RX_TIMEOUT_COUNT 		700							// 最长等待1ms
	
	uint8 count = 0;												// 循环次数计数器
	uint32 timeout_count = 0;										// 等待超时计数器
	
	uint8 temp_buffer = 0;											// 数据缓存
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (BSU_MAX_PARAMETER_LENGTH + 6); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())		// 如果接收数据不成功(FALSE)则开始计时
		{
			timeout_count++;
			if(timeout_count > BSU_RX_TIMEOUT_COUNT)
			{
				return ERR_OVERFLOW;
			}
		}
		
//		cli();
		p_uart_control->pReceiveByte(&temp_buffer);
//		sei();

		switch(count)
		{
			case 1:													// 匹配启动字1
			{
				if(BSU_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:													// 匹配启动字2
			{		
				if(BSU_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:													// 设备ID号
			{
				p_bsu_receive -> device_id = temp_buffer;
				break;
			}
			case 4:													// 数据包长度
			{
				p_bsu_receive -> parameter_length = temp_buffer - 2;
				
				if(BSU_MAX_PARAMETER_LENGTH < p_bsu_receive -> parameter_length)
				{
					return ERR_RANGE;
				}
				
				break;
			}
			case 5:													// 错误状态字
			{
				p_bsu_receive -> error_byte = temp_buffer;
				break;
			}
			default:
			{
				if(count < (6 + p_bsu_receive -> parameter_length))
				{
					p_bsu_receive -> receive_parameter[count - 6] = temp_buffer;
				}
				else
				{
					check_sum = p_bsu_receive -> device_id;			// 计算检验和
					check_sum += (p_bsu_receive -> parameter_length + 2);
					check_sum += p_bsu_receive -> error_byte;
					for(check_sum_count = 0; check_sum_count < (p_bsu_receive -> parameter_length); check_sum_count++)
					{
						check_sum += p_bsu_receive -> receive_parameter[check_sum_count];
					}
					check_sum = ~check_sum;
					if(check_sum != temp_buffer)
					{
						return ERR_CRC;
					}
					else
					{
						return ERR_OK;
					}
				}
				break;
			}
		}
	}
	return ERR_MATH;
}


/*************************************************************************************************************************
** 函数名称:			TxBsuPacket
**
** 函数描述:			发送总线伺服单元的控制数据包的组包(To String)、发送
**                      
**                      
**					                 
** 输入变量:			BSU_SEND_CONTROL_STRUCT *p_bsu_send;
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
static uint8 TxBsuPacket(BSU_SEND_STRUCT* p_bsu_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(BSU_START1);											// 开始位1、2
	(p_uart_control->pWriteTxBuffer)(BSU_START2);
	(p_uart_control->pWriteTxBuffer)(p_bsu_send->device_id);								// 设备ID
	
	switch (p_bsu_send->instruction_byte)
	{
		case INST_PING:
		{
			(p_uart_control->pWriteTxBuffer)(0x02);											// 读取操作时，数据长度为4，不会变化
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);
			
			check_sum = p_bsu_send->device_id;
			check_sum += 0x02;
			check_sum += p_bsu_send->instruction_byte;
			check_sum = ~check_sum;															// 校验和计算
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// 校验和
			
			break;
		}
		case INST_READ:
		{
			(p_uart_control->pWriteTxBuffer)(0x04);											// 读取操作时，数据长度为4，不会变化
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// 寄存器首地址
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->read_length);						// 要读取的数据长度
			
			check_sum = p_bsu_send->device_id;
			check_sum += 0x04;
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;
			check_sum += p_bsu_send->read_length;
			check_sum = ~check_sum;															// 校验和计算
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// 校验和
			
			break;
		}
		case INST_WRITE:
		{
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->parameter_length + 3);				// 数据长度

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);					// 控制字

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// 寄存器首地址
			
			check_sum = p_bsu_send->device_id;
			check_sum += (p_bsu_send->parameter_length + 3);
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;

			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				(p_uart_control->pWriteTxBuffer)(p_bsu_send->write_parameter[count]);		// 写数据组
			}
			
			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				check_sum += p_bsu_send->write_parameter[count];							// 校验和计算
			}

			check_sum = ~check_sum;															// 校验和计算
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// 校验和
			
			break;
		}
		case INST_REG_WRITE:
		{
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->parameter_length + 3);				// 数据长度

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);					// 控制字

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// 寄存器首地址
			
			check_sum = p_bsu_send->device_id;
			check_sum += (p_bsu_send->parameter_length + 3);
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;

			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				(p_uart_control->pWriteTxBuffer)(p_bsu_send->write_parameter[count]);		// 写数据组
			}
			
			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				check_sum += p_bsu_send->write_parameter[count];		// 校验和计算
			}

			check_sum = ~check_sum;										// 校验和计算
			
			(p_uart_control->pWriteTxBuffer)(check_sum);				// 校验和
			
			break;
		}
		case INST_ACTION:
		{
			(p_uart_control->pWriteTxBuffer)(0x02);						// ACTION操作时，数据长度为2，不会变化
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);

			check_sum = p_bsu_send->device_id;							// 校验和计算
			check_sum += (p_bsu_send->instruction_byte + 0x02);
			check_sum = ~check_sum;

			(p_uart_control->pWriteTxBuffer)(check_sum);				// 校验和
			
			break;
		}
		default:
		{
			return ERR_VALUE;
			break;
		}
	}
	

	(p_uart_control->pSendTxBuffer)();									// 发送缓冲数据

	
	return ERR_OK;
}




/*************************************************************************************************************************
**                                                     结构体定义
*************************************************************************************************************************/
BSU_CONTROL_STRUCT bsu_control = 
{
	.pTxPacket = TxBsuPacket,
	.pRxPacket = RxBsuPacket,
	.pTxSyncPacket = TxBsuSyncPacket,
};


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
