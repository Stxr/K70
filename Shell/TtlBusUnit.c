/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			TtlBusUnit.c
** ����޶�����:		2009-03-23
** ���汾:			1.0
** ����:				ʹ��DynamixelProtocolЭ��ջ��UART�ӿڵĲ��������ӿ�
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-19
** �汾:				1.0
** ����:				ʹ��DynamixelProtocolЭ��ջ��UART�ӿڵĲ��������ӿ�
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			����
** �޶�����:			2009-10-28
** �汾:				2.0
** ����:				�����߲������ִӶ�����������з������
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
*************************************************************************************************************************/
#include "Shell/TtlBusUnit.h"


/*************************************************************************************************************************
** ��������:			BsuRegWriteWord2
**
** ��������:			��ָ�����߽ڵ�д�������ͱ�����
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusRegWord2(uint8 id, uint8 adress, uint16 first_word, uint16 second_word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuWriteWord2
**
** ��������:
**
**
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusWord2(uint8 id, uint8 adress, uint16 first_word, uint16 second_word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuRegWriteWord
**
** ��������:			��ڵ�дһ��16λ��
**
**
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusRegWord(uint8 id, uint8 adress, uint16 word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuRegWriteWord
**
** ��������:			��ڵ�дһ��16λ��
**
**
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusWord(uint8 id, uint8 adress, uint16 word)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuWriteByte
**
** ��������:			��ڵ�дһ���ֽڡ�
**
**
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:				uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:				����
** ��������:			2009-03-14
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusByte(uint8 id, uint8 adress, uint8 byte)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);								// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			WriteBsuSyncControl
**
** ��������:			ͬ��д��������ڴ�����������ͬ��д���׵�ַ�����ݳ���
**
**
** �������:			uint8 adress; uint8 length;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void WriteTtlBusSyncControl(uint8 adress, uint8 length)
{
	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

	ttlbus_control.str_bsu_sync.first_address = adress;
	ttlbus_control.str_bsu_sync.parameter_length = length;


	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);
}


/*************************************************************************************************************************
** ��������:			WriteBsuSyncWord2
**
** ��������:			ͬ��д��������ڴ�����������ͬ��д�Ĳ���
**
**
** �������:			uint8 id; uint8 word1; uint8 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 WriteTtlBusSyncWord2(uint8 id, uint16 first_word, uint16 second_word)
{
	uint8 count = 0;
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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
** ��������:			ActionBsuSync
**
** ��������:			ͬ��д��������ڴ����������豸��������
**
**
** �������:			uint8 id; uint8 word1; uint8 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ActionTtlBusSync(void)
{
	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

	bsu_control.pTxSyncPacket(&ttlbus_control.str_bsu_sync, ttlbus_control.p_device_mapping, ttlbus_control.p_uart_control);

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);
}


/*************************************************************************************************************************
** ��������:			BsuAction
**
** ��������:			ʹ�Ĵ���ָ�ʼִ��
**
**
**
** �������:			uint8 id;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionTtlBus(void)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

	ttlbus_control.str_bsu_stack.bsu_send.device_id = BSU_BROADCASTING_ID;
	ttlbus_control.str_bsu_stack.bsu_send.instruction_byte = INST_ACTION;

	temp_state = (bsu_control.pTxPacket)(&ttlbus_control.str_bsu_stack.bsu_send, ttlbus_control.p_uart_control);

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuReadWord
**
** ��������:			��ָ���ڵ��ȡһ������ֵ;
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ReadTtlBusWord(uint8 id, uint8 adress, uint16* value)
{
	uint8 temp_state = ERR_OK;
	uint8 temp_length = 0;
	uint8 count = 0;
	uint8 temp_parameter[2];

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuReadWord
**
** ��������:			 ��ָ���ڵ��ȡһ������ֵ;
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ReadTtlBusWord2(uint8 id, uint8 adress, uint16* first_word, uint16* second_word)
{
	uint8 temp_state = ERR_OK;
	uint8 temp_length = 0;
	uint8 count = 0;
	uint8 temp_parameter[4];

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			BsuPing
**
** ��������:			PING�ڵ㣬��ֻ����һ�����ʱ�����ñ��������й㲥��ȷ���ڵ�ID��
**
**
** �������:			uint8 id; uint8 adress; uint16 word1; uint16 word2;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 PingTtlBus(uint8 id, uint8* back_id)
{
	uint8 temp_state = ERR_OK;

	timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

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

	timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ʹ��ϵͳ�ж�

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			InitTtlbusSyncMode
**
** ��������:			ͬ��д����ģʽ��ʼ��
**
**
** �������:			uint8* p_mapping, uint8 amount
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
	ttlbus_control.sync_update_state = TRUE;				// TRUEʱ�������Control����

	return TRUE;
}


/*************************************************************************************************************************
													���ƽṹ������
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
**														�ļ�����
*************************************************************************************************************************/
