/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Gpio.h
** 最后修订日期:		2009-03-17
** 最后版本:			1.0
** 描述:				Gpio控制函数接口，包含对PPM输出端口的定义(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-17
** 版本:				1.0
** 描述:				对接口的方向及电平的设置直接映射AVR本身的功能，不提供对51等MCU的接口控制方法的兼容(API)
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
#ifndef GPIO_H_
#define GPIO_H_

#include "ConfigTypes.h"


#define		GPIO_GLOBALS


#ifndef   GPIO_GLOBALS
     #define GPIO_EXT     
#else 
     #define GPIO_EXT  extern
#endif 


/*************************************************************************************************************************
                                                  GPIO端口定意
*************************************************************************************************************************/
#define GPIO1_PORT			PORTE							// 定义GPIO输出所使用的端口数据寄存器
#define GPIO1_PORT_BIT		PE4								// 定义端口中每一位与GPIO编号的对应关系
#define GPIO1_DDR			DDRE							// 定义GPIO输出所使用的端口方向寄存器
#define GPIO1_DDR_BIT		DDE4
#define GPIO1_PIN			PINE							// 定义GPIO输出所使用的端口的状态寄存器
#define GPIO1_PIN_BIT		PINE4
#define GPIO1_BIT			4

#define GPIO0_PORT			PORTE		
#define GPIO0_PORT_BIT		PE5	
#define GPIO0_DDR			DDRE	
#define GPIO0_DDR_BIT		DDE5
#define GPIO0_PIN			PINE
#define GPIO0_PIN_BIT		PINE5
#define GPIO0_BIT			5

#define GPIO3_PORT			PORTE		
#define GPIO3_PORT_BIT		PE6	
#define GPIO3_DDR			DDRE	
#define GPIO3_DDR_BIT		DDE6
#define GPIO3_PIN			PINE
#define GPIO3_PIN_BIT		PINE6
#define GPIO3_BIT			6

#define GPIO2_PORT			PORTE		
#define GPIO2_PORT_BIT		PE7	
#define GPIO2_DDR			DDRE	
#define GPIO2_DDR_BIT		DDE7
#define GPIO2_PIN			PINE
#define GPIO2_PIN_BIT		PINE7
#define GPIO2_BIT			7


#define GPIO5_PORT			PORTB	
#define GPIO5_PORT_BIT		PB6		
#define GPIO5_DDR			DDRB	
#define GPIO5_DDR_BIT		DDB6
#define GPIO5_PIN			PINB
#define GPIO5_PIN_BIT		PINB6
#define GPIO5_BIT			6

#define GPIO4_PORT			PORTB	
#define GPIO4_PORT_BIT		PB5		
#define GPIO4_DDR			DDRB	
#define GPIO4_DDR_BIT		DDB5
#define GPIO4_PIN			PINB
#define GPIO4_PIN_BIT		PINB5
#define GPIO4_BIT			5

#define GPIO7_PORT			PORTE	
#define GPIO7_PORT_BIT		PE2		
#define GPIO7_DDR			DDRE	
#define GPIO7_DDR_BIT		DDE2
#define GPIO7_PIN			PINE
#define GPIO7_PIN_BIT		PINE2
#define GPIO7_BIT			2

#define GPIO6_PORT			PORTE		
#define GPIO6_PORT_BIT		PE3			
#define GPIO6_DDR			DDRE	
#define GPIO6_DDR_BIT		DDE3
#define GPIO6_PIN			PINE
#define GPIO6_PIN_BIT		PINE3
#define GPIO6_BIT			3


#define GPIO11_PORT			PORTD		
#define GPIO11_PORT_BIT		PD6			
#define GPIO11_DDR			DDRD	
#define GPIO11_DDR_BIT		DDD6
#define GPIO11_PIN			PIND
#define GPIO11_PIN_BIT		PIND6
#define GPIO11_BIT			6

#define GPIO8_PORT			PORTD		
#define GPIO8_PORT_BIT		PD4			
#define GPIO8_DDR			DDRD	
#define GPIO8_DDR_BIT		DDD4
#define GPIO8_PIN			PIND
#define GPIO8_PIN_BIT		PIND4
#define GPIO8_BIT			4

#define GPIO10_PORT			PORTB		
#define GPIO10_PORT_BIT		PB7			
#define GPIO10_DDR			DDRB	
#define GPIO10_DDR_BIT		DDB7
#define GPIO10_PIN			PINB
#define GPIO10_PIN_BIT		PINB7
#define GPIO10_BIT			7

#define GPIO9_PORT			PORTD		
#define GPIO9_PORT_BIT		PD1			
#define GPIO9_DDR			DDRD	
#define GPIO9_DDR_BIT		DDD1
#define GPIO9_PIN			PIND
#define GPIO9_PIN_BIT		PIND1
#define GPIO9_BIT			1


#define GPIO_GROUP0_PIN		PINB
#define GPIO_GROUP1_PIN		PIND
#define GPIO_GROUP2_PIN		PINE

#define GPIO_GROUP0_MASK	(1 << GPIO4_BIT) | (1 << GPIO5_BIT) | (1 << GPIO10_BIT)
#define GPIO_GROUP1_MASK	(1 << GPIO8_BIT) | (1 << GPIO9_BIT) | (1 << GPIO11_BIT)
#define GPIO_GROUP2_MASK	(1 << GPIO0_BIT) | (1 << GPIO1_BIT) | (1 << GPIO2_BIT) | (1 << GPIO3_BIT) | (1 << GPIO6_BIT) | (1 << GPIO7_BIT)


/*************************************************************************************************************************
											结构体定义
*************************************************************************************************************************/
typedef void (tSetGpioDdr)(uint16 word);
typedef void (tSetGpioPort)(uint16 word);
typedef void (tSetOutputPort)(uint16 word);
typedef uint8 (tSetOutputBit)(uint8 bit, uint8 val);
typedef void (tGetGpioPort)(uint16* val);
typedef void (tGetInputPort)(uint16* val);
typedef uint8 (tGetInputBit)(uint8 bit, uint8* val);
typedef void (tInitGpio)(void);
typedef uint8 tGetInputFilteBit(uint8 channel, uint8* val);
typedef void tGetInputFilte(uint16* val);

typedef struct SGpioControl
{
	uint8 state;
	
	volatile uint8* port_mapping[12];				// 映射表
	volatile uint8* pin_mapping[12];				// 映射表
	volatile uint8* ddr_mapping[12];				// 映射表
	uint8 bit_mapping[12];		
	
	uint8 group0_buffer[8];							// 存储区域，固定端口的引脚区域
	uint8 group1_buffer[8];
	uint8 group2_buffer[8];
	
	uint8 buffer_count;								// 当前缓冲的位置
	
	uint8* p_buffer_mapping[12];					// 映射表
	
	
	uint16 direction_mask;							// 方向遮罩，用于输出引脚对值的处理
	
	tSetGpioDdr* pSetGpioDdr;
	tSetGpioPort* pSetGpioPort;
	tSetOutputPort* pSetOutputPort;
	tSetOutputBit* pSetOutputBit;
	tGetGpioPort* pGetGpioPort;
	tGetInputPort* pGetInputPort;
	tGetInputBit* pGetInputBit;
	tGetInputFilteBit* pGetInputFilteBit;
	tGetInputFilte* pGetInputFilte;
	
	tInitGpio* pInit;
}GPIO_CONTROL_STRUCT;


/*************************************************************************************************************************
										控制结构体声明
*************************************************************************************************************************/
GPIO_EXT GPIO_CONTROL_STRUCT gpio_control;


/*************************************************************************************************************************
										函数声明
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
