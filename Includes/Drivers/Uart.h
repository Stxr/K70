/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Uart.h
** 最后修订日期:		2009-03-07
** 最后版本:			1.0
** 描述:				Uart的标准编成接口(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-07
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
#ifndef UART_H_
#define UART_H_

#include "ConfigTypes.h"
#include "Lib/Queue.h"
#include "Drivers/Led.h"


#define		UART_GLOBALS

#ifndef   UART_GLOBALS
     #define UART_EXT     
#else 
     #define UART_EXT  extern
#endif 

#define UART0_BSU_EN			1
#define UART0_ISR_MODE			0

/*************************************************************************************************************************
														硬件宏定义
*************************************************************************************************************************/
// UCSRnA
/*************************************************************************************************************************
 *	7		6		5		4		3		2		1		0
 *	RXCn	TXCn	UDREn	FEn		DORn	UPEn	U2Xn	MPCMn
 *
 * Bit 7 – RXCn: USART 接收结束
 * Bit 6 – TXCn: USART 发送结束
 * Bit 5 – UDREn: USART 数据寄存器空
 * Bit 4 – FEn: 帧错误
 * Bit 3 – DORn: 数据过速
 * Bit 2 – UPEn: 奇偶校验错误
 * Bit 1 – U2Xn: 倍速发送
 * Bit 0 – MPCMn: 多处理器通信模式
*************************************************************************************************************************/

// UCSRnB
/*************************************************************************************************************************
 *	7		6		5		4		3		2		1		0
 *	RXCIEn	TXCIEn	UDRIEn	RXENn	TXENn	UCSZn2	RXB8n	TXB8n
 *
 * Bit 7 – RXCIEn: 接收结束中断使能
 * Bit 6 – TXCIE: 发送结束中断使能
 * Bit 5 – UDRIEn: USART 数据寄存器空中断使能
 * Bit 4 – RXENn: 接收使能
 * Bit 3 – TXENn: 发送使能
 * Bit 2 – UCSZn2: 字符长度
 * Bit 1 – RXB8n: 接收数据位8
 * Bit 0 – TXB8n: 发送数据位8
*************************************************************************************************************************/

// UCSRnC
/*************************************************************************************************************************
 *	7		6		5		4		3		2		1		0
 *	–		UMSELn	UPMn1	UPMn0	USBSn	UCSZn1	UCSZn0	UCPOLn
 *
 * Bit 6 – UMSELn: USART 模式选择,0 异步操作,1 同步操作
 * Bit 5:4 – UPMn1:0: 奇偶校验模式
 * 		UPMn1	UPMn0			奇偶模式
 * 		0		0 				禁止
 * 		0		1				保留
 * 		1		0				偶校验
 * 		1		1				奇校验
 * Bit 3 – USBSn: 停止位选择
 * 		USBSn 					停止位位数
 * 		0						1-bit
 * 		1						2-bits
 * Bit 2:1 – UCSZn1:0: 字符长度
 * 		UCSZn2	UCSZn1	UCSZn0 	字符长度
 * 		0		0		0		5 位
 * 		0		0		1		6 位
 * 		0		1		0		7 位
 * 		0		1		1		8 位
 *		1		0		0		保留
 *		1		0		1		保留
 *		1		1		0		保留
 *		1		1		1		9 位
 * Bit 0 – UCPOLn: 时钟极性
 *		UCPOLn	发送数据的改变(TxDn 引脚的输出)	接收数据的采样(RxDn 引脚的输入)
 *		0		XCKn上升沿						XCKn 下降沿
 *		1		XCKn下降沿						XCKn 上升沿
 *
*************************************************************************************************************************/


#define UART_BAUDRATE_1X		0								// 波特率模式选择
#define UART_BAUDRATE_2X		1

#define UART_U2X_BIT			1

#define UART0_UDR				UDR0							// UART0的硬件寄存器
#define UART0_UCSRA				UCSR0A
#define UART0_UCSRB				UCSR0B
#define UART0_UCSRC				UCSR0C

#define UART0_UBRR_H			UBRR0H
#define UART0_UBRR_L			UBRR0L

#define UART0_BAUDRATE			1000000							// UART0默认波特率

#define UART1_UDR				UDR1							// UART1的硬件寄存器
#define UART1_UCSRA				UCSR1A
#define UART1_UCSRB				UCSR1B
#define UART1_UCSRC				UCSR1C

#define UART1_UBRR_H			UBRR1H
#define UART1_UBRR_L			UBRR1L

#define UART1_BAUDRATE			115200							// UART1默认波特率


#define UART0_PORT				PORTE							// 定义输出所使用的端口数据寄存器
#define UART0_RX_PORT_BIT		PE0								// 
#define UART0_TX_PORT_BIT		PE1	
#define UART0_DDR				DDRE							// 定义所使用的端口方向寄存器
#define UART0_RX_DDR_BIT		DDE0							
#define UART0_TX_DDR_BIT		DDE1

#define UART1_PORT				PORTD		
#define UART1_RX_PORT_BIT		PD2
#define UART1_TX_PORT_BIT		PD3
#define UART1_DDR				DDRD
#define UART1_RX_DDR_BIT		DDD2
#define UART1_TX_DDR_BIT		DDD3

#define UART0_EN_PORT			PORTA							
#define UART0_RX_EN_PORT_BIT	PA1								
#define UART0_TX_EN_PORT_BIT	PA0	
#define UART0_EN_DDR			DDRA							
#define UART0_RX_EN_DDR_BIT		DDA1							
#define UART0_TX_EN_DDR_BIT		DDA0


/*************************************************************************************************************************
														结构体定义
*************************************************************************************************************************/
typedef struct SUartContorl
{
	volatile uint8		state;									// 状态标志

	BUFFER_QUEUE		str_rx_buffer;							// 建立接收缓冲区
	BUFFER_QUEUE		str_tx_buffer;							// 建立发送缓冲区

	tVoidVoid*			pInit;
	tVoidUint8Uint32*	pSetBaudRate;							// 设置波特率
	tVoidUint8*			pSendByte;								// 发送1字节变量
	tUint8Puint8*		pReceiveByte;							// 接收1字节变量
	tVoidUint8*			pWriteTxBuffer;							// 将元素写入发送缓冲队列
	tVoidVoid*			pSendTxBuffer;							// 将发送缓冲队列中的数据发送
	tUint8Void*			pTestReceiveBuffer;						// 测试接收缓冲是否存在数据
	tVoidVoid*			pClearReceiveBuffer;
}UART_CONTROL_STRUCT;


/*************************************************************************************************************************
														控制结构体声明
*************************************************************************************************************************/
UART_EXT UART_CONTROL_STRUCT uart0_control;
UART_EXT UART_CONTROL_STRUCT uart1_control;
//added by txr
UART_EXT UART_CONTROL_STRUCT uart0_test;
UART_EXT UART_CONTROL_STRUCT uart1_test;
#endif
/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
