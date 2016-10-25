/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			DynamixelProtocol.c
** ����޶�����:		2009-03-11
** ���汾:			1.0
** ����:				Robotis��˾�����Dynamixel�����ŷ�����Э���Э��ջ
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-11
** �汾:				1.0
** ����:				�������ɿ��ƽṹ�岢д���ͻ��壻
**					����UART���ջ����������ݣ�����״̬�ṹ�壻
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			
** �޶�����:		
** �汾:			
** ����:		    
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			
** �޶�����:	    
** �汾:		    
** ����:            
**
*************************************************************************************************************************/
#include "Lib/DynamixelProtocol.h"


/*************************************************************************************************************************
** ��������:			TxBsuSyncPacket
**
** ��������:			���������ŷ���Ԫ��ͬ���������ݰ������(To String)������
**
**
** �������:			BSU_SYNC_SEND_STRUCT* p_sync_send, uint8 p_id_mapping, UART_CONTROL_STRUCT* p_uart_control
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
static uint8 TxBsuSyncPacket(BSU_SYNC_SEND_STRUCT* p_sync_send, const uint8* p_id_mapping, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 id_count = 0;
	uint8 parameter_count = 0;

	uint8 temp_value = 0;
	uint8 check_sum = 0;


	if (0 == p_sync_send->update_amount)											// ������±�־
	{
		return ERR_VALUE;
	}

	(p_uart_control->pWriteTxBuffer)(BSU_START1);									// ��ʼλ1��2
	(p_uart_control->pWriteTxBuffer)(BSU_START2);
	(p_uart_control->pWriteTxBuffer)(BSU_BROADCASTING_ID);							// �豸ID

	check_sum = BSU_BROADCASTING_ID;

	temp_value = ((p_sync_send->parameter_length + 1) *  p_sync_send->update_amount) + 4;

	(p_uart_control->pWriteTxBuffer)(temp_value);									// ���ݳ���

	check_sum += temp_value;

	(p_uart_control->pWriteTxBuffer)(INST_SYNC_WRITE);								// ������

	check_sum += INST_SYNC_WRITE;

	(p_uart_control->pWriteTxBuffer)(p_sync_send->first_address);					// �Ĵ����׵�ַ

	check_sum += p_sync_send->first_address;

	(p_uart_control->pWriteTxBuffer)(p_sync_send->parameter_length);				// ͬ�����ƵĲ�������

	check_sum += p_sync_send->parameter_length;

	for (id_count = 0; id_count < (p_sync_send->update_amount); id_count++)
	{
		temp_value = *(p_id_mapping + p_sync_send->update_mapping[id_count]);
		(p_uart_control->pWriteTxBuffer)(temp_value);								// ID
		check_sum += temp_value;

		for (parameter_count = 0; parameter_count < (p_sync_send->parameter_length); parameter_count++)
		{
			temp_value = p_sync_send->write_parameter[p_sync_send->update_mapping[id_count]][parameter_count];
			(p_uart_control->pWriteTxBuffer)(temp_value);							// ����
			check_sum += temp_value;
		}
	}

	check_sum = ~check_sum;															// У��ͼ���

	(p_uart_control->pWriteTxBuffer)(check_sum);									// У���

	p_sync_send->update_amount = 0;													// ������±�־

	(p_uart_control->pSendTxBuffer)();												// ���ͻ�������

	return ERR_OK;
}


/*************************************************************************************************************************
** ��������:			RxBsuPacket
**
** ��������:			�ֶν���(Parser)
**
**
**
** �������:			BSU_RECEIVE_CONTROL_STRUCT *p_bsu_receive;
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
static uint8 RxBsuPacket(BSU_RECEIVE_STRUCT *p_bsu_receive, UART_CONTROL_STRUCT* p_uart_control)
{
	#define BSU_RX_TIMEOUT_COUNT 		700							// ��ȴ�1ms
	
	uint8 count = 0;												// ѭ������������
	uint32 timeout_count = 0;										// �ȴ���ʱ������
	
	uint8 temp_buffer = 0;											// ���ݻ���
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (BSU_MAX_PARAMETER_LENGTH + 6); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())		// ����������ݲ��ɹ�(FALSE)��ʼ��ʱ
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
			case 1:													// ƥ��������1
			{
				if(BSU_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:													// ƥ��������2
			{		
				if(BSU_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:													// �豸ID��
			{
				p_bsu_receive -> device_id = temp_buffer;
				break;
			}
			case 4:													// ���ݰ�����
			{
				p_bsu_receive -> parameter_length = temp_buffer - 2;
				
				if(BSU_MAX_PARAMETER_LENGTH < p_bsu_receive -> parameter_length)
				{
					return ERR_RANGE;
				}
				
				break;
			}
			case 5:													// ����״̬��
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
					check_sum = p_bsu_receive -> device_id;			// ��������
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
** ��������:			TxBsuPacket
**
** ��������:			���������ŷ���Ԫ�Ŀ������ݰ������(To String)������
**                      
**                      
**					                 
** �������:			BSU_SEND_CONTROL_STRUCT *p_bsu_send;
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
static uint8 TxBsuPacket(BSU_SEND_STRUCT* p_bsu_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(BSU_START1);											// ��ʼλ1��2
	(p_uart_control->pWriteTxBuffer)(BSU_START2);
	(p_uart_control->pWriteTxBuffer)(p_bsu_send->device_id);								// �豸ID
	
	switch (p_bsu_send->instruction_byte)
	{
		case INST_PING:
		{
			(p_uart_control->pWriteTxBuffer)(0x02);											// ��ȡ����ʱ�����ݳ���Ϊ4������仯
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);
			
			check_sum = p_bsu_send->device_id;
			check_sum += 0x02;
			check_sum += p_bsu_send->instruction_byte;
			check_sum = ~check_sum;															// У��ͼ���
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// У���
			
			break;
		}
		case INST_READ:
		{
			(p_uart_control->pWriteTxBuffer)(0x04);											// ��ȡ����ʱ�����ݳ���Ϊ4������仯
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// �Ĵ����׵�ַ
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->read_length);						// Ҫ��ȡ�����ݳ���
			
			check_sum = p_bsu_send->device_id;
			check_sum += 0x04;
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;
			check_sum += p_bsu_send->read_length;
			check_sum = ~check_sum;															// У��ͼ���
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// У���
			
			break;
		}
		case INST_WRITE:
		{
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->parameter_length + 3);				// ���ݳ���

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);					// ������

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// �Ĵ����׵�ַ
			
			check_sum = p_bsu_send->device_id;
			check_sum += (p_bsu_send->parameter_length + 3);
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;

			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				(p_uart_control->pWriteTxBuffer)(p_bsu_send->write_parameter[count]);		// д������
			}
			
			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				check_sum += p_bsu_send->write_parameter[count];							// У��ͼ���
			}

			check_sum = ~check_sum;															// У��ͼ���
			
			(p_uart_control->pWriteTxBuffer)(check_sum);									// У���
			
			break;
		}
		case INST_REG_WRITE:
		{
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->parameter_length + 3);				// ���ݳ���

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);					// ������

			(p_uart_control->pWriteTxBuffer)(p_bsu_send->first_address);					// �Ĵ����׵�ַ
			
			check_sum = p_bsu_send->device_id;
			check_sum += (p_bsu_send->parameter_length + 3);
			check_sum += p_bsu_send->instruction_byte;
			check_sum += p_bsu_send->first_address;

			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				(p_uart_control->pWriteTxBuffer)(p_bsu_send->write_parameter[count]);		// д������
			}
			
			for(count = 0; count < (p_bsu_send->parameter_length); count++)
			{
				check_sum += p_bsu_send->write_parameter[count];		// У��ͼ���
			}

			check_sum = ~check_sum;										// У��ͼ���
			
			(p_uart_control->pWriteTxBuffer)(check_sum);				// У���
			
			break;
		}
		case INST_ACTION:
		{
			(p_uart_control->pWriteTxBuffer)(0x02);						// ACTION����ʱ�����ݳ���Ϊ2������仯
			(p_uart_control->pWriteTxBuffer)(p_bsu_send->instruction_byte);

			check_sum = p_bsu_send->device_id;							// У��ͼ���
			check_sum += (p_bsu_send->instruction_byte + 0x02);
			check_sum = ~check_sum;

			(p_uart_control->pWriteTxBuffer)(check_sum);				// У���
			
			break;
		}
		default:
		{
			return ERR_VALUE;
			break;
		}
	}
	

	(p_uart_control->pSendTxBuffer)();									// ���ͻ�������

	
	return ERR_OK;
}




/*************************************************************************************************************************
**                                                     �ṹ�嶨��
*************************************************************************************************************************/
BSU_CONTROL_STRUCT bsu_control = 
{
	.pTxPacket = TxBsuPacket,
	.pRxPacket = RxBsuPacket,
	.pTxSyncPacket = TxBsuSyncPacket,
};


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
