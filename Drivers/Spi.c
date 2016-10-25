/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Spi.c
** 最后修订日期:		2009-03-30
** 最后版本:			1.0
** 描述:				Spi的标准编成接口
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-30
** 版本:				1.0
** 描述:				包括设备的初始化、波特率的调整、开关中断、缓冲区的用量设置等
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
#include "Drivers/Spi.h"


/*************************************************************************************************************************
** 函数名称:			EnableSpiInterrupt
**
** 函数描述:			使能SPI传输完成中断
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
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void EnableSpiInterrupt(void)
{
	SetRegBit(SPI_SPCR, SPIE);
}


/*************************************************************************************************************************
** 函数名称:			DisableSpiInterrupt
**
** 函数描述:			屏蔽SPI传输完成中断
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
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void DisableSpiInterrupt(void)
{
	ClrRegBit(SPI_SPCR, SPIE);
}


/*************************************************************************************************************************
** 函数名称:			SetSpiClockRate
**
** 函数描述:			设置Spi时钟频率，该频率有系统时钟分频得出，对从机模式无效。
** 					参数为时钟频率（分频值）；
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
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetSpiClockRate(uint8 clock_rate)
{
	uint8 temp_value = 0;

	temp_value = (clock_rate >> 2) & 0x01;				// 取 Bit2;

	if (TRUE == temp_value)
	{
		SetRegBit(SPI_SPSR, SPI2X);
	}
	else
	{
		ClrRegBit(SPI_SPSR, SPI2X);
	}

	temp_value = clock_rate & 0x03;						// 取Bit0、Bit1;

	SetRegMask(SPI_SPCR, SPI_SPCR_SPR_MASK, temp_value << SPI_SPCR_SPR_OFFSET);
}

/*************************************************************************************************************************
** 函数名称:			SetSpiClockMode
**
** 函数描述:			设置Spi时钟模式，主机和从机的模式匹配时才能够正常通讯；
** 					参数为时钟极性；时钟相位；
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
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetSpiClockMode(uint8 polarity, uint8 phase)
{
	uint8 temp_value = 0;

	temp_value = (phase + (polarity << 1)) & 0x03;				// 取低两位

	SetRegMask(SPI_SPCR, SPI_SPCR_CLOCKMODE_MASK, temp_value << SPI_SPCR_CLOCKMODE_OFFSET);
}

/*************************************************************************************************************************
** 函数名称:			SetSpiMode
**
** 函数描述:			设置Spi工作模式，可以设置为主机或从机；
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
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetSpiMode(uint8 mode)
{
	if (SPI_MODE_MASTER == mode)
	{
		ClrRegBit(SPI_PORT, SPI_MOSI_BIT);
		SetRegBit(SPI_PORT, SPI_MISO_BIT);
		ClrRegBit(SPI_PORT, SPI_SCK_BIT);
		SetRegBit(SPI_PORT, SPI_SS_BIT);

		SetRegBit(SPI_DDR, SPI_MOSI_BIT);
		ClrRegBit(SPI_DDR, SPI_MISO_BIT);
		SetRegBit(SPI_DDR, SPI_SCK_BIT);
		SetRegBit(SPI_DDR, SPI_SS_BIT);

		SetRegBit(SPI_SPCR, MSTR);
	}
	else
	{
		ClrRegBit(SPI_PORT, SPI_MOSI_BIT);										// 初始化端口
		SetRegBit(SPI_PORT, SPI_MISO_BIT);
		ClrRegBit(SPI_PORT, SPI_SCK_BIT);
		SetRegBit(SPI_PORT, SPI_SS_BIT);

		ClrRegBit(SPI_DDR, SPI_MOSI_BIT);
		SetRegBit(SPI_DDR, SPI_MISO_BIT);
		ClrRegBit(SPI_DDR, SPI_SCK_BIT);
		ClrRegBit(SPI_DDR, SPI_SS_BIT);

		ClrRegBit(SPI_SPCR, MSTR);
	}
}


/*************************************************************************************************************************
** 函数名称:			StartTransport
**
** 函数描述:			该函数用于向SPI移位寄存器中写入一个值，开始与从机交换数据。
** 					!注意! 该函数只能在SPI中断使能的情况下使用，本主要服务于Fractal系统两个硬件模块间的数据传输方法，该方法也在本文件中定义。
**
** 输入变量:			uint8 val;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void StartTransport(uint8 val)
{
	SetReg(SPI_SPDR, val);
}


/*************************************************************************************************************************
** 函数名称:			WriteSpiSendBuffer
**
** 函数描述:			写Spi发送缓冲
**
** 输入变量:			uint8;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-09-11
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void WriteSpiSendBuffer(uint8 val)
{
	buffer_queue_control.pAdd(&spi_control.str_send_queue, val);
}


/*************************************************************************************************************************
** 函数名称:			ProcessSendMsg
**
** 函数描述:			通讯中对数据包的处理过程之一，用于检测数据缓冲的当前状态，进行数据的发送。
** 					!注意! 该函数只能在SPI中断使能的情况下使用。
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
** 创建日期:			2009-08-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ProcessSendMsg(void)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	if (SPI_SEND_STATE_WAITSEND == spi_control.send_state)
	{
		spi_control.send_state = SPI_SEND_STATE_SENDING;
		spi_control.send_length = buffer_queue_control.pCheckLength(&spi_control.str_send_queue);

		temp_value = (spi_control.send_length << SPI_DATA_LENGTH_OFFSET & SPI_DATA_LENGTH_MASK)
					| ((spi_control.send_device << SPI_DATA_DEVICE_OFFSET) & SPI_DATA_DEVICE_MASK);
	}
	else if (SPI_SEND_STATE_SENDING == spi_control.send_state)
	{
		temp_state = buffer_queue_control.pOut(&spi_control.str_send_queue, &temp_value);

		if(FALSE == temp_state)
		{
			SetRegMask(spi_control.state, ERR_MASK, ERR_UNDERRUN);						// 置错误类型标志位
			return;
		}

		temp_state = buffer_queue_control.pTestEmpty(&spi_control.str_send_queue);		// 测试队列是否为空

		if (TRUE == temp_state)
		{
			spi_control.send_state = SPI_SEND_STATE_WAITUPDATE;
		}
	}
	else if (SPI_RECEIVE_STATE_RECEIVING == spi_control.receive_state)
	{
		temp_value = SPI_SEND_FILLER;
	}
	else
	{
		return;
	}

	ClrRegBit(SPI_PORT, SPI_SS_BIT);													// 主机SS拉低从机SS
	while (FALSE != GetRegBit(SPI_PIN, SPI_SS_BIT));									// 检查PIN寄存器的值，如果没有拉低则等待

	SetReg(SPI_SPDR, temp_value);														// 开始传输，返回等待数据传输完成中断发生
}

/*************************************************************************************************************************
** 函数名称:			ProcessReceiveMsg
**
** 函数描述:			通讯中对数据包的处理过程之一，用于检测数据缓冲的当前状态，进行数据的接收处理。
** 					!注意! 该函数只能在SPI中断使能的情况下使用。该函数依赖VirtualUart模块、SystemUnit模块、AudioPluse模块的初始化
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
** 创建日期:			2009-08-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ProcessReceiveMsg(void)
{
	uint8 temp_value = 0;

	temp_value = GetReg(SPI_SPDR);

	if (SPI_RECEIVE_STATE_RECEIVING == spi_control.receive_state)
	{
		if (SPI_RECEIVE_CONTROL_WORD == spi_control.receive_index)
		{
			spi_control.receive_length = temp_value & SPI_DATA_LENGTH_MASK;
			spi_control.receive_device = (temp_value & SPI_DATA_DEVICE_MASK) >> SPI_DATA_DEVICE_OFFSET;

			spi_control.receive_index = SPI_RECEIVE_PACKET;
		}
		else
		{
			switch (spi_control.receive_device)
			{
				// 根据目标模块不同存储数据，存在三种目标：Uart3的接收缓冲、SystemUnit的系统电压、SystemUnit过流标志
				case SPI_DEVICE_UART_RECEIVE:
				{
					buffer_queue_control.pAdd(&uart2_control.str_rx_buffer, temp_value);
				}
		//		case

		//		default
			}

			spi_control.receive_counter++;

			if(spi_control.receive_counter >= spi_control.receive_length)
			{
				spi_control.receive_counter = 0;
				spi_control.receive_length = 0;

				spi_control.receive_index = SPI_RECEIVE_CONTROL_WORD;
				spi_control.receive_state = SPI_RECEIVE_STATE_WAITRECEIVE;
			}
		}
	}

	SetRegBit(SPI_PORT, SPI_SS_BIT);												// 主机SS拉高从机SS
}


/*************************************************************************************************************************
** 函数名称:			IsrSpiProcessExtInt
**
** 函数描述:			外部中断中调用的中断处理函数
** 					!注意! 该函数依赖ExtInt模块的初始化
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
** 创建日期:			2009-08-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrSpiProcessExtInt(void)
{
	if (spi_control.receive_state == SPI_RECEIVE_STATE_WAITRECEIVE)
	{
		if (spi_control.send_state == SPI_SEND_STATE_SENDING)
		{
			spi_control.switch_asynchronous = TRUE;
			return;
		}
		else
		{
			spi_control.receive_state = SPI_RECEIVE_STATE_RECEIVING;
		}
	}

	ProcessSendMsg();
}


/*************************************************************************************************************************
** 函数名称:			IsrSpiProcessTimerInt
**
** 函数描述:			定时器中调用的中断处理函数
** 					!注意! 该函数依赖Timer模块的初始化
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
** 创建日期:			2009-08-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrSpiProcessTimerInt(void)
{
	uint8 temp_value = 0;

	if (SPI_SEND_STATE_WAITSEND == spi_control.send_state)
	{
		if (SPI_RECEIVE_STATE_WAITRECEIVE == spi_control.receive_state)
		{
			spi_control.send_state = SPI_SEND_STATE_SENDING;
			temp_value = ((spi_control.send_length << SPI_DATA_LENGTH_OFFSET) & SPI_DATA_LENGTH_MASK)
						| ((spi_control.send_device << SPI_DATA_DEVICE_OFFSET) & SPI_DATA_DEVICE_MASK);

			ClrRegBit(SPI_PORT, SPI_SS_BIT);										// 主机SS拉低从机SS
			SetReg(SPI_SPDR, temp_value);											// 开始传输，返回等待数据传输完成中断发生
		}
	}
}

/*************************************************************************************************************************
** 函数名称:			SIGNAL(SIG_SPI)
**
** 函数描述:			SPI传输完成中断中调用的中断处理函数
** 					!注意! 该函数依赖SPI中断使能
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
** 创建日期:			2009-08-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
ISR(SPI_STC_vect, ISR_BLOCK)
{
	ProcessReceiveMsg();

	if (spi_control.switch_asynchronous == TRUE)
	{
		spi_control.receive_state = SPI_RECEIVE_STATE_RECEIVING;	// 切换模式
		spi_control.switch_asynchronous = FALSE;					// 解除异步模式切换标志
	}

	if (SPI_RECEIVE_STATE_WAITRECEIVE == spi_control.receive_state)
	{
		ProcessSendMsg();
	}
}

/*************************************************************************************************************************
** 函数名称:			TransportByte
**
** 函数描述:			该函数用于向SPI移位寄存器中写入一个值，开始与从机交换数据，并等待数据传输完成。
** 					!注意! 本函数只能在SPI中断关闭的状况下使用。
**					数据发送速率为16/4M，1字节的传输时间在2-3us，后台等待时间要小于利用中断处理的时间，所以不使用中断。
**					                 
** 输入变量:			uint8 val, uint8* back_val
** 返回值:			uint8
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-04-14
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 TransportByte(uint8 val, uint8* back_val)
{
	#define SPI_TIMEOUT_COUNT			700						// 约1ms
	
	uint16 count = 0;
	
	count = 0;

	ClrRegBit(SPI_PORT, SPI_SS_BIT);							// 主机SS拉低从机SS

	SetReg(SPI_SPDR, val);										// 开始传输
	
	while(!GetRegBit(SPI_SPSR, SPIF))							// 等待传输结束
	{
		count++;
		if (count > SPI_TIMEOUT_COUNT)
		{
			SetRegBit(SPI_PORT, SPI_SS_BIT);
			
			return ERR_OVERFLOW;								// 等待超过SPI_TIMEOUT_COUNT个查询周期后，超时退出，报告错误
		}
	}
	
	*back_val = GetReg(SPI_SPDR);
	
	SetRegBit(SPI_PORT, SPI_SS_BIT);							// 传输完成后，拉高从机SS，使SPI从机立即复位接收和发送逻辑，不再接收数据

	return ERR_OK;
}


/*************************************************************************************************************************
** 函数名称:			InitSpi()
**
** 函数描述:			 初始化SPI模块
**					                 
** 输入变量:		    void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			SetRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-30
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitSpi(void)
{
	SetReg(spi_control.state, ERR_OK);								// 初始化状态寄存器

	buffer_queue_control.pInit(&spi_control.str_send_queue, 32);	// 初始化缓冲区

	if (!GetRegBit(spi_control.str_send_queue.state, INIT_COMPLETE))// 检查队列空间是否分配成功
	{
		SetRegMask(spi_control.state, ERR_MASK, ERR_NOTAVAIL);		// 置错误类型标志位
		return;
	}

	SetSpiMode(SPI_MODE_MASTER);
	SetSpiClockRate(SPI_PERSCALE_4);
	EnableSpiInterrupt();

	spi_control.send_device = 0x00;									// 设备类型编号
	spi_control.send_length = 0x00;									// 缓冲队列长度
	spi_control.send_state = SPI_SEND_STATE_WAITUPDATE;				// 状态

	spi_control.receive_state = SPI_RECEIVE_STATE_WAITRECEIVE;		// 接收状态
	spi_control.receive_index = SPI_RECEIVE_CONTROL_WORD;			// 接收索引
	spi_control.receive_length = 0x00;								// 接收数据长度
	spi_control.receive_device = 0x00;								// 接收数据来源
	spi_control.receive_counter = 0x00;								// 接收计数器

	spi_control.switch_asynchronous = FALSE;						// 异步状态切换状态解除

	SetRegBit(SPI_SPCR, SPE);										// 使能SPI模块

	spi_control.pEnableInterrupt = EnableSpiInterrupt;
	spi_control.pDisableInterrupt = DisableSpiInterrupt;
	spi_control.pSetClockRate = SetSpiClockRate;
	spi_control.pSetClockMode = SetSpiClockMode;
	spi_control.pSetMode = SetSpiMode;
	spi_control.pStartTransport = StartTransport;
	spi_control.pTransportByte = TransportByte;
	spi_control.pWriteSendBuffer = WriteSpiSendBuffer;

	SetRegBit(spi_control.state, INIT_COMPLETE);					// 置位初始化完成标志
}


/*************************************************************************************************************************
														结构体声明
*************************************************************************************************************************/
SPI_CONTROL_STRUCT spi_control = { .pInit = InitSpi };


/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
