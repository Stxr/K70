/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Spi.h
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
#ifndef SPI_H_
#define SPI_H_

#include "ConfigTypes.h"
#include "Lib/Queue.h"
#include "Drivers/Led.h"
#include "Drivers/Uart.h"
#include "Drivers/VirtualUart.h"


#define		SPI_GLOBALS

#ifndef   SPI_GLOBALS
     #define SPI_EXT     
#else 
     #define SPI_EXT  extern
#endif 


/*************************************************************************************************************************
														硬件定义
*************************************************************************************************************************/
#define SPI_DDR				DDRB
#define SPI_PORT			PORTB
#define SPI_PIN				PINB

#define SPI_MISO_BIT		3
#define SPI_MOSI_BIT		2
#define SPI_SCK_BIT			1
#define SPI_SS_BIT			0


#define SPI_SPCR			SPCR					// 控制寄存器
/*************************************************************************************************************************
 *  7    6    5   4     3    2    1    0
 * SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0
 *
 * Bit 7 – SPIE: SPI 中断使能
 * 		置位后,只要 SPSR 寄存器的 SPIF 和 SREG 寄存器的全局中断使能位置位,就会引发
 * 		SPI 中断。
 * Bit 6 – SPE: SPI 使能
 * 		SPE 置位将使能 SPI。
 * Bit 5 – DORD: 数据次序
 * 		DORD 置位时数据的 LSB 首先发送;否则数据的 MSB 首先发送。
 * Bit 4 – MSTR: 主 / 从选择
 * 		MSTR 置位时选择主机模式,  否则为从机。 如果 MSTR 为 ’1’,SS配置为输入,但被拉低,
 * 		则 MSTR 被清零,寄存器 SPSR 的 SPIF 置位。 用户必须重新设置 MSTR 进入主机模式。
 * Bit 3 – CPOL: 时钟极性
 * 		CPOL为高表示空闲时SCK为高电平;否则,表示空闲时SCK为低电平。
 *
 * CPOL		起始沿		结束沿
 *  0		上升沿		下降沿
 *  1		下降沿		上升沿
 *
 * Bit 2 – CPHA: 时钟相位
 * 		CPHA 决定数据是在 SCK 的起始沿采样还是在 SCK 的结束沿采样。
 *
 * CPHA		起始沿		结束沿
 *   0		采样			设置
 *   1		设置			采样
 *
 * Bits 1, 0 – SPR1, SPR0: SPI 时钟速率选择 1 和 0
 *
 * SPI2X	SPR1	SPR0	SCK 频率
 *   0		 0		 0		fosc /4
 *   0		 0		 1		fosc /16
 *   0		 1		 0		fosc /64
 *   0		 1		 1		fosc /128
 *   1		 0		 0		fosc /2
 *   1		 0		 1		fosc /8
 *   1		 1		 0		fosc /32
 *   1		 1		 1		fosc /64
 *
*************************************************************************************************************************/

#define SPI_SPSR			SPSR					// 状态寄存器
/*************************************************************************************************************************
 *  7    6   5 4 3 2 1   0
 * SPIF WCOL – – – – – SPI2X
 *
 * Bit 7 – SPIF: SPI 中断标志
 * 		串行发送结束后,SPIF 置位。若此时寄存器 SPCR 的 SPIE 和全局中断使能位置位,SPI
 * 		中断即产生。如果 SPI 为主机, SS 配置为输入,且被拉低, SPIF 也将置位。进入中断
 * 		例程后 SPIF 自动清零。或者可以通过先读 SPSR,紧接着访问 SPDR 来对 SPIF 清零。
 * Bit 6 – WCOL: 写冲突标志
 * 		在发送当中对 SPI 数据寄存器 SPDR 写数据将置位 WCOL。WCOL 可以通过先读 SPSR,
 * 		紧接着访问 SPDR 来清零。
 * Bit 0 – SPI2X:SPI 倍速
 * 		置位后 SPI 的速度加倍。若为主机,则 SCK 频率可达 CPU 频率的一半。若为从机,只
 * 		能保证 fosc /4。
 *
*************************************************************************************************************************/

#define SPI_SPDR			SPDR					// 数据寄存器


/*************************************************************************************************************************
 * 													控制参数定义
*************************************************************************************************************************/
#define	SPI_SPCR_SPR_OFFSET			0x00								// 时钟频率控制偏移
#define	SPI_SPCR_SPR_MASK			0x03 << SPI_SPCR_SPR_OFFSET			// 时钟频率控制遮罩

#define SPI_PERSCALE_4				0x00								// F_CPU / 4
#define SPI_PERSCALE_16				0x01								// F_CPU / 16
#define SPI_PERSCALE_64				0x02								// F_CPU / 64
#define SPI_PERSCALE_128			0x03								// F_CPU / 128
#define SPI_PERSCALE_2				0x04								// F_CPU / 2
#define SPI_PERSCALE_8				0x05								// F_CPU / 8
#define SPI_PERSCALE_32				0x06								// F_CPU / 32

#define SPI_SPCR_CLOCKMODE_OFFSET	0x02								// 时钟极性相位模式偏移
#define SPI_SPCR_CLOCKMODE_MASK		0x03 << SPI_SPCR_CLOCKMODE_OFFSET	// 时钟极性相位模式遮罩

#define SPI_CLOCKMODE_RISING		0x00								// 上升沿起始
#define SPI_CLOCKMODE_FALLING		0x01								// 下降沿起始
#define SPI_CLOCKMODE_SAMPLING		0x00								// 起始沿采样
#define SPI_CLOCKMODE_SETING		0x01								// 起始沿设置

#define SPI_MODE_MASTER				0x01								// 主机模式
#define SPI_MODE_SLAVE				0x00								// 从机模式

#define SPI_SEND_FILLER				0xF0								// 用于交换从机数据的填充物


/*************************************************************************************************************************
 * 													数据状态控制字定义
*************************************************************************************************************************/
#define SPI_SEND_STATE_WAITSEND			0									// 等待发送
#define SPI_SEND_STATE_SENDING			1									// 发送中
#define SPI_SEND_STATE_WAITUPDATE		2									// 等待更新
#define SPI_SEND_STATE_UPDATING			3									// 更新中
#define SPI_RECEIVE_STATE_WAITRECEIVE	0									// 等待接收
#define SPI_RECEIVE_STATE_RECEIVING	1										// 接收中

#define SPI_DATA_LENGTH_OFFSET			0									// 发送长度偏移量
#define SPI_DATA_LENGTH_MASK			0x0F << SPI_DATA_LENGTH_OFFSET		// 发送长度遮罩
#define SPI_DATA_DEVICE_OFFSET			4									// 发送设备偏移量
#define SPI_DATA_DEVICE_MASK			0x0F << SPI_DATA_DEVICE_OFFSET		// 发送设备遮罩

#define SPI_RECEIVE_CONTROL_WORD		0									// 接收数据包索引“控制字”
#define SPI_RECEIVE_PACKET				1									// 接收数据包索引“数据包”


/*************************************************************************************************************************
 * 													虚拟设备编号
*************************************************************************************************************************/
//Rs422:
//		接收数据			《－		0x01
#define SPI_DEVICE_UART_RECEIVE		0x01
//		波特率设置		－》		0x02
#define SPI_DEVICE_UART_BAUD		0x02
//		发送数据			－》		0x03
#define SPI_DEVICE_UART_SEND		0x03
//AudioPluse:
//		声音队列写入		－》		0x04
#define SPI_DEVICE_AUDIO_QUEUE		0x04
//		声音单音阶写入	－》		0x05
#define SPI_DEVICE_AUDIO_ALONE		0x05
//		发声				－》		0x06
#define SPI_DEVICE_AUDIO_START		0x06
//		发声停止			－》		0x07
#define SPI_DEVICE_AUDIO_MUTE		0x07
//SystemVoltage:
//		电压读取			《－		0x08
#define SPI_DEVICE_VOLTAGE_READ		0x08
//		过压报警电压设置	－》		0x09
#define SPI_DEVICE_VOLTAGE_MAX		0x09
//		欠压报警电压设置	－》		0x0A
#define SPI_DEVICE_VOLTAGE_MIN		0x0A
//PowerAlame:
//		电源报警			《－		0x0B
#define SPI_DEVICE_POWER_ALAME		0x0B


/*************************************************************************************************************************
 * 													控制结构体定义
*************************************************************************************************************************/
typedef struct SSpiControl
{
	uint8 				state;					// SPI状态

	BUFFER_QUEUE		str_send_queue;			// 发送队列

	uint8				send_device;			// 设备类型编号
	uint8				send_length;			// 缓冲队列长度
	uint8				send_state;				// 状态

	uint8				receive_state;			// 接收状态
	uint8				receive_index;			// 接收索引
	uint8				receive_length;			// 接收数据长度
	uint8				receive_device;			// 接收数据来源
	uint8				receive_counter;		// 接收计数器

	uint8				switch_asynchronous;	// 异步状态切换标志

	tVoidVoid* 			pInit;
	tVoidVoid*			pEnableInterrupt;
	tVoidVoid*			pDisableInterrupt;
	tVoidUint8*			pSetClockRate;
	tVoidUint8Uint8*	pSetClockMode;
	tVoidUint8*			pSetMode;
	tVoidUint8*			pStartTransport;
	tUint8Uint8Puint8*	pTransportByte;
	tVoidUint8*			pWriteSendBuffer;
}SPI_CONTROL_STRUCT;


/*************************************************************************************************************************
控制结构体声明
*************************************************************************************************************************/
SPI_EXT SPI_CONTROL_STRUCT spi_control;


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
