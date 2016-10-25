/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			TtlBusUnit.c
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
#include "Shell/TtlBusUnit.h"


/*************************************************************************************************************************
** 函数名称:			BsuRegWriteWord2
**
** 函数描述:			向指定总线节点写两个整型变量。
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
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusRegWord2(uint8 id, uint8 adress, uint16 first_word, uint16 second_word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_REG_WRITE;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.parameter_length = 0x04;

	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[0], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[1], first_word);
	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[2], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[3], second_word);

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
				temp_state = ERR_OK;
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuWriteWord2
**
** 函数描述:
**
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
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusWord2(uint8 id, uint8 adress, uint16 first_word, uint16 second_word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_WRITE;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.parameter_length = 0x04;

	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[0], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[1], first_word);
	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[2], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[3], second_word);

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
				temp_state = ERR_OK;
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuRegWriteWord
**
** 函数描述:			向节点写一个16位字
**
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
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusRegWord(uint8 id, uint8 adress, uint16 word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_REG_WRITE;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.parameter_length = 0x02;

	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[0], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[1], word);

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
				temp_state = ERR_OK;
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuRegWriteWord
**
** 函数描述:			向节点写一个16位字
**
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
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusWord(uint8 id, uint8 adress, uint16 word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_WRITE;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.parameter_length = 0x02;

	Out16(ttlbus_control.str_bsu_stack.bsu_send.write_parameter[0], ttlbus_control.str_bsu_stack.bsu_send.write_parameter[1], word);

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
				temp_state = ERR_OK;
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuWriteByte
**
** 函数描述:			向节点写一个字节。
**
**
**
** 输入变量:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** 返回值:				uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:				律晔
** 创建日期:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusByte(uint8 id, uint8 adress, uint8 byte)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_WRITE;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.parameter_length = 0x01;

	ttlbus_control.str_bsu_stack.bsu_send.write_parameter[0] = byte;

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
				temp_state = ERR_OK;
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			WriteBsuSyncControl
**
** 函数描述:			同步写控制命令，在此命令中设置同步写的首地址及数据长度
**
**
** 输入变量:			uint8 adress; uint8 length;
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
static void WriteTtlBusSyncControl(uint8 adress, uint8 length)
{
	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_sync.first_address = adress;
	ttlbus_control.str_bsu_sync.parameter_length = length;


	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);
}


/*************************************************************************************************************************
** 函数名称:			WriteBsuSyncWord2
**
** 函数描述:			同步写控制命令，在此命令中设置同步写的参数
**
**
** 输入变量:			uint8 id; uint8 word1; uint8 word2;
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
static uint8 WriteTtlBusSyncWord2(uint8 id, uint16 first_word, uint16 second_word)
{
	uint8 count = 0;
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	temp_state = mapping_control.pInverse1DimArray(ttlbus_control.p_device_mapping, ttlbus_control.device_amount, id, &temp_value);

	if (TRUE == temp_state)
	{
		for (count = 0; count < ttlbus_control.str_bsu_sync.update_amount; count++)
		{
			if (temp_value == ttlbus_control.str_bsu_sync.update_mapping[count])
			{
				temp_state = FALSE;
				break;
			}
		}

		if (TRUE == temp_state)
		{
			ttlbus_control.str_bsu_sync.update_mapping[ttlbus_control.str_bsu_sync.update_amount] = temp_value;
			ttlbus_control.str_bsu_sync.update_amount++;
		}

		Out16(ttlbus_control.str_bsu_sync.write_parameter[temp_value][0], ttlbus_control.str_bsu_sync.write_parameter[temp_value][1], first_word);
		Out16(ttlbus_control.str_bsu_sync.write_parameter[temp_value][2], ttlbus_control.str_bsu_sync.write_parameter[temp_value][3], second_word);

		temp_state = ERR_OK;
	}
	else
	{
		temp_state = ERR_VALUE;
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			ActionBsuSync
**
** 函数描述:			同步写控制命令，在此命令中向设备发送数据
**
**
** 输入变量:			uint8 id; uint8 word1; uint8 word2;
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
static void ActionTtlBusSync(void)
{
	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	bsu_control.pTxSyncPacket(&ttlbus_control.str_bsu_sync, ttlbus_control.p_device_mapping, ttlbus_control.p_uart_control);

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);
}


/*************************************************************************************************************************
** 函数名称:			BsuAction
**
** 函数描述:			使寄存器指令开始执行
**
**
**
** 输入变量:			uint8 id;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionTtlBus(void)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = BSU_BROADCASTING_ID;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_ACTION;

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuReadWord
**
** 函数描述:			从指定节点读取一个整型值;
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ReadTtlBusWord(uint8 id, uint8 adress, uint16* value)
{
	uint8 temp_state = ERR_OK;
	uint8 temp_length = 0;
	uint8 count = 0;
	uint8 temp_parameter[2];

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_READ;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.read_length = 0x02;

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);


	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				temp_length = ttlbus_control.str_bsu_stack.bsu_receive.parameter_length;

				if(2 == temp_length)
				{
					ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;

					for(count = 0; count < temp_length; count++)
					{
						temp_parameter[count] = ttlbus_control.str_bsu_stack.bsu_receive.receive_parameter[count];
					}

					In16(*value, temp_parameter[0], temp_parameter[1]);

					temp_state = ERR_OK;
				}
				else
				{
					temp_state = ERR_VALUE;
				}
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuReadWord
**
** 函数描述:			 从指定节点读取一个整型值;
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ReadTtlBusWord2(uint8 id, uint8 adress, uint16* first_word, uint16* second_word)
{
	uint8 temp_state = ERR_OK;
	uint8 temp_length = 0;
	uint8 count = 0;
	uint8 temp_parameter[4];

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_READ;
	ttlbus_control.str_bsu_stack.bsu_send.first_address = adress;
	ttlbus_control.str_bsu_stack.bsu_send.read_length = 0x04;

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			if(ttlbus_control.str_bsu_stack.bsu_receive.device_id == ttlbus_control.str_bsu_stack.bsu_send.device_id)
			{
				temp_length = ttlbus_control.str_bsu_stack.bsu_receive.parameter_length;

				if(4 == temp_length)
				{
					ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;

					for(count = 0; count < temp_length; count++)
					{
						temp_parameter[count] = ttlbus_control.str_bsu_stack.bsu_receive.receive_parameter[count];
					}

					In16(*first_word, temp_parameter[0], temp_parameter[1]);
					In16(*second_word, temp_parameter[2], temp_parameter[3]);

					temp_state = ERR_OK;
				}
				else
				{
					temp_state = ERR_VALUE;
				}
			}
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			BsuPing
**
** 函数描述:			PING节点，当只连接一个舵机时，可用本方法进行广播，确定节点ID。
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
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 PingTtlBus(uint8 id, uint8* back_id)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

	ttlbus_control.str_bsu_stack.bsu_send.device_id = id;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_PING;

	ttlbus_control.p_uart_control->pClearReceiveBuffer();

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	if(ERR_OK == temp_state)
	{
		temp_state = (bsu_control.pRxPacket)(&ttlbus_control.str_bsu_stack.bsu_receive, ttlbus_control.p_uart_control);
		if(ERR_OK == temp_state)
		{
			*back_id = ttlbus_control.str_bsu_stack.bsu_receive.device_id;
			ttlbus_control.state = ttlbus_control.str_bsu_stack.bsu_receive.error_byte;
			temp_state = ERR_OK;
		}
	}

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 使能系统中断

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			InitTtlbusSyncMode
**
** 函数描述:			同步写控制模式初始化
**
**
** 输入变量:			uint8* p_mapping, uint8 amount
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
static uint8 InitTtlBusSync(uint8* p_mapping, uint8 amount)
{
	if (amount > 32)
	{
		return FALSE;
	}

	ttlbus_control.p_device_mapping = p_mapping;
	ttlbus_control.str_bsu_sync.update_amount = 0;
	ttlbus_control.device_amount = amount;
	ttlbus_control.sync_update_state = TRUE;				// TRUE时允许访问Control命令

	return TRUE;
}


/*************************************************************************************************************************
													控制结构体声明
*************************************************************************************************************************/
TTLBUS_CONTROL_STRUCT ttlbus_control =
{
	.p_uart_control = &uart0_control,

	.pInitSync = InitTtlBusSync,
	.pWriteRegWord2 = WriteTtlBusRegWord2,
	.pWriteWord2 = WriteTtlBusWord2,
	.pWriteRegWord = WriteTtlBusRegWord,
	.pWriteWord = WriteTtlBusWord,
	.pWriteByte = WriteTtlBusByte,
	.pWriteSyncControl = WriteTtlBusSyncControl,
	.pWriteSyncWord2 = WriteTtlBusSyncWord2,
	.pActionSync = ActionTtlBusSync,
	.pAction = ActionTtlBus,
	.pReadWord = ReadTtlBusWord,
	.pReadWord2 = ReadTtlBusWord2,
	.pPing = PingTtlBus,


	.state = ERR_OK,
};


/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
