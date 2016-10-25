/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			VirtualUart.c
** 最后修订日期:		2009-08-20
** 最后版本:			1.0
** 描述:				虚拟Uart的标准编成接口，虚拟Uart2与SPI模块关联，用于控制Fractal从机模块的Uart接口。
** 					按照虚拟Uart所提供的数据结构，也可以虚拟出其他逻辑Uart用于调试或逻辑模块间的通讯。
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-08-20
** 版本:				1.0
** 描述:				包括设备的初始化、缓冲区的用量设置等
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
#include "Drivers/VirtualUart.h"


/*************************************************************************************************************************
** 函数名称:			SetUart2BaudRate
**
** 函数描述:			该函数用于设置UART2的波特率
**					                 
** 输入变量:			mode(模式，UART_BAUDRATE_1X或UART_BAUDRATE_2X), baud(波特率)
** 返回值:			void;
**
** 使用宏或常量:		None
** 使用全局变量:	    None;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetUart2BaudRate(uint8 mode, uint32 baud)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// 约2.5ms

	uint16 count = 0;
//	uint8 temp_length = 0;

	uint32 temp_baud = 0;

	uint8 temp_value = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// 等待传输结束
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// 等待超过VIRTUALUART_TIMEOUT_COUNT个查询周期后，超时退
																		// 出，报告错误
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_BAUD;

//	temp_length = 5;

//	spi_control.send_length = temp_length;

	buffer_queue_control.pAdd(&spi_control.str_send_queue, mode);

	temp_baud = baud;

	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud的低8位
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud的中低8位
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud的中高8位
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	temp_baud = temp_baud >> 8;
	temp_value = (uint8)(temp_baud & 0x000000FF);						// baud的高8位
	buffer_queue_control.pAdd(&spi_control.str_send_queue, temp_value);

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// 发送队列准备完成，准备发送
}


/*************************************************************************************************************************
** 函数名称:			SendUart2byte
**
** 函数描述:			该函数用于从uart2发送数据
**					                 
** 输入变量:			val;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SendUart2Byte(uint8 val)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// 约2.5ms

	uint16 count = 0;
//	uint8 temp_length = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// 等待传输结束
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// 等待超过VIRTUALUART_TIMEOUT_COUNT个查询周期后，超时退
																		// 出，报告错误
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_SEND;

//	temp_length = 1;

//	spi_control.send_length = temp_length;

	buffer_queue_control.pAdd(&spi_control.str_send_queue, val);

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// 发送队列准备完成，准备发送
}


/*************************************************************************************************************************
** 函数名称:			WriteUart2TxBuffer
**
** 函数描述:			将数据写入UART2的发送缓冲队列
**					                 
** 输入变量:			val;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void WriteUart2TxBuffer(uint8 val)
{
	buffer_queue_control.pAdd(&uart2_control.str_tx_buffer, val);
}


/*************************************************************************************************************************
** 函数名称:			SentUart2TxBuffer
**
** 函数描述:			将发送缓冲队列中的数据发送
**					                 
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SendUart2TxBuffer(void)
{
	#define VIRTUALUART_TIMEOUT_COUNT			1800					// 约2.5ms

	uint16 count = 0;
//	uint8 short_count = 0;
	uint8 temp_value = 0;
//	uint8 temp_length = 0;

	count = 0;

	while (SPI_SEND_STATE_WAITUPDATE != spi_control.send_state)			// 等待传输结束
	{
		count++;
		if (count > VIRTUALUART_TIMEOUT_COUNT)
		{
			SetRegMask(uart2_control.state, ERR_MASK, ERR_OVERFLOW);
			return;														// 等待超过VIRTUALUART_TIMEOUT_COUNT个查询周期后，超时退
																		// 出，报告错误
		}
	}

	spi_control.send_state = SPI_SEND_STATE_UPDATING;

	spi_control.send_device = SPI_DEVICE_UART_SEND;

//	temp_length = buffer_queue_control.pCheckLength(&uart2_control.str_tx_buffer);	// 统计需要发送的数据长度

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

	spi_control.send_state = SPI_SEND_STATE_WAITSEND;					// 发送队列准备完成，准备发送
}


/*************************************************************************************************************************
** 函数名称:			ReceiveUart2Byte
**
** 函数描述:			该函数用于从uart2接收数据
**					                 
** 输入变量:			val;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
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
** 函数名称:			TestReceiveUart2
**
** 函数描述:			 该函数用于判断接收缓冲区是否存在数据，存在返回TURE，不存在返回FALSE
**					                 
** 输入变量:			void;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
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
** 函数名称:			InitUart2
**
** 函数描述:			uart2初始化函数
**                      
**                      
**					                 
** 输入变量:			void
** 返回值:			void
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			InitBufferQueue; Out16;
**
** 创建人:			律晔
** 创建日期:			2009-08-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitUart2(void)
{
	SetReg(uart2_control.state , 0);
	
	if (!GetRegBit(spi_control.state, INIT_COMPLETE))
	{
		SetRegMask(uart2_control.state, ERR_MASK, ERR_DEPENDENCE);							// 置错误类型标志位
		return;
	}

	buffer_queue_control.pInit(&uart2_control.str_rx_buffer, 64);							// 初始化缓冲区
	buffer_queue_control.pInit(&uart2_control.str_tx_buffer, 64);
	
	if (!GetRegBit(uart2_control.str_rx_buffer.state, INIT_COMPLETE)
		|| !GetRegBit(uart2_control.str_rx_buffer.state, INIT_COMPLETE))					// 检查队列空间是否分配成功
	{
		SetRegMask(uart2_control.state, ERR_MASK, ERR_NOTAVAIL);							// 置错误类型标志位
		return;
	}
	
	// 代码

	uart2_control.pSetBaudRate = SetUart2BaudRate;											// 初始化波特率设置函数
	uart2_control.pSendByte = SendUart2Byte;												// 初始化发送函数
	uart2_control.pReceiveByte = ReceiveUart2Byte;											// 初始化接收函数
	uart2_control.pWriteTxBuffer = WriteUart2TxBuffer;										// 初始化入缓冲队列函数
	uart2_control.pSendTxBuffer = SendUart2TxBuffer;										// 初始化发送缓冲队列函数
	uart2_control.pTestReceiveBuffer = TestReceiveUart2;									// 初始化测试接收缓冲函数

	SetRegBit(uart2_control.state, INIT_COMPLETE);											// 置位初始化完成标志
}


/*************************************************************************************************************************
                                                         结构体声明
*************************************************************************************************************************/
UART_CONTROL_STRUCT uart2_control = { .pInit = InitUart2 };


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
