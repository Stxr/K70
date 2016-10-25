/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			VirtualUart.c
** ����޶�����:		2009-08-20
** ���汾:			1.0
** ����:				����Uart�ı�׼��ɽӿڣ�����Uart2��SPIģ����������ڿ���Fractal�ӻ�ģ���Uart�ӿڡ�
** 					��������Uart���ṩ�����ݽṹ��Ҳ��������������߼�Uart���ڵ��Ի��߼�ģ����ͨѶ��
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-08-20
** �汾:				1.0
** ����:				�����豸�ĳ�ʼ�������������������õ�
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
#include "Drivers/VirtualUart.h"


/*************************************************************************************************************************
** ��������:			SetUart2BaudRate
**
** ��������:			�ú�����������UART2�Ĳ�����
**					                 
** �������:			mode(ģʽ��UART_BAUDRATE_1X��UART_BAUDRATE_2X), baud(������)
** ����ֵ:			void;
**
** ʹ�ú����:		None
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetUart2BaudRate(uint8 mode, uint32 baud)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// Լ2.5ms

	uint16 count = 0;
//	uint8 temp_length = 0;

	uint32 temp_baud = 0;

	uint8 temp_value = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// �ȴ��������
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// �ȴ�����VIRTUALUART_TIMEOUT_COUNT����ѯ���ں󣬳�ʱ��
																		// �����������
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_BAUD;

//	temp_length = 5;

//	spi_control.send_length = temp_length;

	buffer_queue_control.pAdd(&spi_control.str_send_queue, mode);

	temp_baud = baud;

	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud�ĵ�8λ
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud���е�8λ
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud���и�8λ
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud�ĸ�8λ
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// ���Ͷ���׼����ɣ�׼������
}


/*************************************************************************************************************************
** ��������:			SendUart2byte
**
** ��������:			�ú������ڴ�uart2��������
**					                 
** �������:			val;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SendUart2Byte(uint8 val)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// Լ2.5ms

	uint16 count = 0;
//	uint8 temp_length = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// �ȴ��������
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// �ȴ�����VIRTUALUART_TIMEOUT_COUNT����ѯ���ں󣬳�ʱ��
																		// �����������
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_SEND;

//	temp_length = 1;

//	spi_control.send_length = temp_length;

	buffer_queue_control.pAdd(&spi_control.str_send_queue, val);

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// ���Ͷ���׼����ɣ�׼������
}


/*************************************************************************************************************************
** ��������:			WriteUart2TxBuffer
**
** ��������:			������д��UART2�ķ��ͻ������
**					                 
** �������:			val;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void WriteUart2TxBuffer(uint8 val)
{
	buffer_queue_control.pAdd(&uart2_control.str_tx_buffer, val);
}


/*************************************************************************************************************************
** ��������:			SentUart2TxBuffer
**
** ��������:			�����ͻ�������е����ݷ���
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SendUart2TxBuffer(void)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// Լ2.5ms

	uint16 count = 0;
//	uint8 short_count = 0;
	uint8 temp_value = 0;
//	uint8 temp_length = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// �ȴ��������
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// �ȴ�����VIRTUALUART_TIMEOUT_COUNT����ѯ���ں󣬳�ʱ��
																		// �����������
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_SEND;

//	temp_length = buffer_queue_control.pCheckLength(&uart2_control.str_tx_buffer);	// ͳ����Ҫ���͵����ݳ���

//	spi_control.send_length = temp_length;

//	for (short_count = 0; short_count < temp_length; short_count++)
//	{
//		buffer_queue_control.pOut(&uart2_control.str_tx_buffer, &temp_value);
//		buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);
//	}

	while (uart2_control.pReceiveByte(&temp_value))
	{
		spi_control.pWriteSendBuffer(temp_value);
	}

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// ���Ͷ���׼����ɣ�׼������
}


/*************************************************************************************************************************
** ��������:			ReceiveUart2Byte
**
** ��������:			�ú������ڴ�uart2��������
**					                 
** �������:			val;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ReceiveUart2Byte(uint8 *val)
{
	if((buffer_queue_control.pOut)(&uart2_control.str_rx_buffer, val))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** ��������:			TestReceiveUart2
**
** ��������:			 �ú��������жϽ��ջ������Ƿ�������ݣ����ڷ���TURE�������ڷ���FALSE
**					                 
** �������:			void;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 TestReceiveUart2(void)
{
	if((buffer_queue_control.pTestEmpty)(&uart2_control.str_rx_buffer))
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}


/*************************************************************************************************************************
** ��������:			InitUart2
**
** ��������:			uart2��ʼ������
**                      
**                      
**					                 
** �������:			void
** ����ֵ:			void
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			InitBufferQueue; Out16;
**
** ������:			����
** ��������:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitUart2(void)
{
	SetReg(uart2_control.state , 0);
	
	if (!GetRegBit(spi_control.state, INIT_COMPLETE))
	{
		SetRegMask(uart2_control.state, ERR_MASK, ERR_DEPENDENCE);							// �ô������ͱ�־λ
		return;
	}

	buffer_queue_control.pInit(&uart2_control.str_rx_buffer, 64);							// ��ʼ��������
	buffer_queue_control.pInit(&uart2_control.str_tx_buffer, 64);
	
	if (!GetRegBit(uart2_control.str_rx_buffer.state, INIT_COMPLETE)
		|| !GetRegBit(uart2_control.str_rx_buffer.state, INIT_COMPLETE))					// �����пռ��Ƿ����ɹ�
	{
		SetRegMask(uart2_control.state, ERR_MASK, ERR_NOTAVAIL);							// �ô������ͱ�־λ
		return;
	}
	
	// ����

	uart2_control.pSetBaudRate = SetUart2BaudRate;											// ��ʼ�����������ú���
	uart2_control.pSendByte = SendUart2Byte;												// ��ʼ�����ͺ���
	uart2_control.pReceiveByte = ReceiveUart2Byte;											// ��ʼ�����պ���
	uart2_control.pWriteTxBuffer = WriteUart2TxBuffer;										// ��ʼ���뻺����к���
	uart2_control.pSendTxBuffer = SendUart2TxBuffer;										// ��ʼ�����ͻ�����к���
	uart2_control.pTestReceiveBuffer = TestReceiveUart2;									// ��ʼ�����Խ��ջ��庯��

	SetRegBit(uart2_control.state, INIT_COMPLETE);											// ��λ��ʼ����ɱ�־
}


/*************************************************************************************************************************
                                                         �ṹ������
*************************************************************************************************************************/
UART_CONTROL_STRUCT uart2_control = { .pInit = InitUart2 };


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
