/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			UpRobotProtocol.c
** ����޶�����:		2009-03-16
** ���汾:			1.0
** ����:				������ʢ�����˼������޹�˾����Ļ����˿���UARTͨѶЭ���Э��ջ
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-16
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
#include "Lib/UpRobotProtocol.h"


#define UPROBOT_RX_TIMEOUT_COUNT 		1400


/*************************************************************************************************************************
** ��������:			UpRobotSlaveRxPacket
**
** ��������:			UpRobot�ӻ��ֶν���(Parser)
**                      
**                      
**					                 
** �������:			UPROBOT_RECEIVE_SLAVE_STRUCT* p_uprobot_receive;
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
static uint8 UpRobotSlaveRxPacket(UPROBOT_MSSR_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control)
{	
	uint8 count = 0;													// ѭ������������
	uint32 timeout_count = 0;											// �ȴ���ʱ������

	uint8 temp_buffer = 0;												// ���ݻ���
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (UPROBOT_MSSR_MAX_PARAMETER_LENGTH + 7); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())					// ����������ݲ��ɹ�(FALSE)��ʼ��ʱ
		{
			timeout_count++;
			if(timeout_count > UPROBOT_RX_TIMEOUT_COUNT)
			{
				return ERR_OVERFLOW;									// �ȴ���ʱ
			}
		}
		
		(p_uart_control->pReceiveByte)(&temp_buffer);

		switch(count)
		{
			case 1:														// ƥ��������1
			{
				if(UPROBOT_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:														// ƥ��������2
			{		
				if(UPROBOT_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:														// �豸ID��
			{
				p_uprobot_receive->device_id = temp_buffer;
				break;
			}
			case 4:														// ���ݰ�����
			{
				p_uprobot_receive->parameter_length = temp_buffer ;
				break;
			}
			case 5:														// ���ܵ�Ԫ���
			{
				p_uprobot_receive->functional_unit = temp_buffer;
				break;
			}
			case 6:														// �������
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
					check_sum = UPROBOT_START1;							// ��������
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
						return ERR_OK;									// �޴���
					}
					else
					{
						//return ERR_CRC;									// У�������
						return ERR_OK;
					}
				}
				break;	
			}			
		}	 
	}
	
	return ERR_MATH;													// ��Ч�����������
}


/*************************************************************************************************************************
** ��������:			UpRobotSlaveTxPacket
**
** ��������:			UpRobot�ӻ����ݰ������(To String)������
**                      
**                      
**					                 
** �������:			UPROBOT_MRSS_STRUCT* p_uprobot_send; UART_CONTROL_STRUCT* p_uart_control
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
static uint8 UpRobotSlaveTxPacket(UPROBOT_MRSS_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START1);									// ��ʼλ1��2
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START2);
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->device_id);						// �豸ID
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter_length);					// ��������
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->functional_unit);					// ���ܵ�Ԫ����
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->method_code);						// ��������
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->state_byte);						// ״̬��־
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter[count]);				// д������
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
		check_sum += p_uprobot_send->parameter[count];									// У��ͼ���
	}

	(p_uart_control->pWriteTxBuffer)(check_sum);
	
	p_uart_control->pSendTxBuffer();													// ���ͻ�������
	
	return ERR_OK;
}
	
	
/*************************************************************************************************************************
** ��������:			UpRobotMasterRxPacket
**
** ��������:			UpRobot�����ֶν���(Parser)
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
** ��������:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotMasterRxPacket(UPROBOT_MRSS_STRUCT* p_uprobot_receive, UART_CONTROL_STRUCT* p_uart_control)
{	
	uint8 count = 0;														// ѭ������������
	uint32 timeout_count = 0;												// �ȴ���ʱ������

	uint8 temp_buffer = 0;													// ���ݻ���
	uint8 check_sum = 0;
	uint8 check_sum_count = 0;
	
	for(count = 1; count <= (UPROBOT_MRSS_MAX_PARAMETER_LENGTH + 8); count++)
	{
		timeout_count = 0;
		
		while(!p_uart_control->pTestReceiveBuffer())					// ����������ݲ��ɹ�(FALSE)��ʼ��ʱ
		{
			timeout_count++;
			if(timeout_count > UPROBOT_RX_TIMEOUT_COUNT)
			{
				return ERR_OVERFLOW;									// �ȴ���ʱ
			}
		}
		
		cli();
		(p_uart_control->pReceiveByte)(&temp_buffer);
		sei();

		switch(count)
		{
			case 1:															// ƥ��������1
			{
				if(UPROBOT_START1 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 2:															// ƥ��������2
			{		
				if(UPROBOT_START2 != temp_buffer)
				{
					count = 0;
				}
				break;
			}
			case 3:															// �豸ID��
			{
				p_uprobot_receive->device_id = temp_buffer;
				break;
			}
			case 4:															// ���ݰ�����
			{
				p_uprobot_receive->parameter_length = temp_buffer ;
				break;
			}
			case 5:															// ״̬��
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
					check_sum = UPROBOT_START1;								// ��������
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
						return ERR_OK;										// �޴���
					}
					else
					{
						return ERR_CRC;										// У�������
					return ERR_OK;
					}
				}
				break;	
			}			
		}	 
	}
	
	
	return ERR_MATH;														// ��Ч�����������
}


/*************************************************************************************************************************
** ��������:			UpRobotMasterTxPacket
**
** ��������:			UpRobot�������ݰ������(To String)������
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
** ��������:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 UpRobotMasterTxPacket(UPROBOT_MSSR_STRUCT* p_uprobot_send, UART_CONTROL_STRUCT* p_uart_control)
{
	uint8 count = 0;
	uint8 check_sum = 0;
	
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START1);									// ��ʼλ1��2
	(p_uart_control->pWriteTxBuffer)(UPROBOT_START2);
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->device_id);						// �豸ID
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter_length);					// ��������
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->functional_unit);					// ����ģ����
	(p_uart_control->pWriteTxBuffer)(p_uprobot_send->method_code);						// �������
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		(p_uart_control->pWriteTxBuffer)(p_uprobot_send->parameter[count]);				// д������
	}
	
	check_sum = UPROBOT_START1;
	check_sum += UPROBOT_START2;
	check_sum += p_uprobot_send->device_id;
	check_sum += p_uprobot_send->parameter_length;
	check_sum += p_uprobot_send->functional_unit;
	check_sum += p_uprobot_send->method_code;
	
	for(count = 0; count < (p_uprobot_send->parameter_length); count++)
	{
		check_sum += p_uprobot_send->parameter[count];									// У��ͼ���
	}

	(p_uart_control->pWriteTxBuffer)(check_sum);
	
	p_uart_control->pSendTxBuffer();													// ���ͻ�������
	
	return ERR_OK;
}


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/
UPROBOT_CONTROL_STRUCT uprobot_control = 
{
	.pUpRobotMasterRxPacket = UpRobotMasterRxPacket,
	.pUpRobotMasterTxPacket = UpRobotMasterTxPacket,
	.pUpRobotSlaveRxPacket = UpRobotSlaveRxPacket,
	.pUpRobotSlaveTxPacket = UpRobotSlaveTxPacket,
};


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
