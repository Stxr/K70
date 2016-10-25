/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Timer.h
** 最后修订日期:		2009-03-18
** 最后版本:			1.0
** 描述:				使用16位TIMER的快速PWM模式实现2.5ms溢出中断与0.5-2.49ms可变的比较中断
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2008-03-18
** 版本:				1.0
** 描述:				2.5ms溢出中断为系统使用的操作，溢出中断在Kernel中的Isp.c中处理，比较匹配中断在Api中的Ppm.c中处理
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
#ifndef TIMER_H_
#define TIMER_H_

#include "ConfigTypes.h"

#define		TIMER_GLOBALS

#ifndef   TIMER_GLOBALS
     #define TIMER_EXT     
#else 
     #define TIMER_EXT  extern
#endif 


/*************************************************************************************************************************
														 硬件宏定义
*************************************************************************************************************************/
#define TIMER0_OCR					OCR0

#define TIMER1_OCRA_H				OCR1AH
#define TIMER1_OCRA_L				OCR1AL
#define TIMER1_OCRB_H				OCR1BH
#define TIMER1_OCRB_L				OCR1BL
#define TIMER1_OCRC_H				OCR1CH
#define TIMER1_OCRC_L				OCR1CL

#define TIMER2_OCR					OCR2

#define TIMER3_OCRA_H				OCR3AH
#define TIMER3_OCRA_L				OCR3AL
#define TIMER3_OCRB_H				OCR3BH
#define TIMER3_OCRB_L				OCR3BL
#define TIMER3_OCRC_H				OCR3CH
#define TIMER3_OCRC_L				OCR3CL

#define TIMER0_TCNT					TCNT0

#define TIMER1_TCNT_H				TCNT1H
#define TIMER1_TCNT_L				TCNT1L

#define TIMER1_ICR_H				ICR1H
#define TIMER1_ICR_L				ICR1L

#define TIMER3_TCNT_H				TCNT3H
#define TIMER3_TCNT_L				TCNT3L

#define TIMER3_ICR_H				ICR3H
#define TIMER3_ICR_L				ICR3L

#define TIMER2_TCNT					TCNT2

#define TIMER1_TCCRA				TCCR1A
#define TIMER3_TCCRA				TCCR3A
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	COM1A1	COM1A0	COM1B1	COM1B0	COM1C1	COM1C0	WGM11	WGM10
 * Bit 7:6 – COMnA1:0: 通道A 的比较输出模式
 * Bit 5:4 – COMnB1:0: 通道B 的比较输出模式
 * Bit 3:2 – COMnC1:0: 通道C 的比较输出模式
 * Bit 1:0 – WGMn1:0: 波形发生模式
*************************************************************************************************************************/

#define TIMER1_TCCRB				TCCR1B
#define TIMER3_TCCRB				TCCR3B
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 * 	ICNC1	ICES1	–		WGM13	WGM12	CS12	CS11	CS10
 * Bit 7 – ICNCn: 输入捕捉噪声抑制器
 * Bit 6 – ICESn: 输入捕捉触发沿选择
 * Bit 5 – 保留位
 * Bit 4:3 – WGMn3:2: 波形发生模式
 * Bit 2:0 – CSn2:0: 时钟选择
*************************************************************************************************************************/

#define TIMER1_TCCRC				TCCR1C
#define TIMER3_TCCRC				TCCR3C
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 * 	FOC1A	FOC1B	FOC1C	-		-		-		-		-
 * Bit 7 – FOCnA: 强制输出比较通道A
 * Bit 6 – FOCnB: 强制输出比较通道B
 * Bit 5 – FOCnC: 强制输出比较通道C
*************************************************************************************************************************/

#define TIMER0_TCCR					TCCR0
#define TIMER2_TCCR					TCCR2
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 * 	FOC0	WGM00	COM01	COM00	WGM01	CS02	CS01	CS00
 * Bit 7 – FOC0: 强制输出比较
 * Bit 6, 3 – WGM01:0: 波形产生模式
 * Bit 5:4 – COM01:0: 比较匹配输出模式
 * Bit 2:0 – CS02:0: 时钟选择
 *
 * 模式		WGM01		WGM00		T/C的工作模式	TOP		OCR0的更新时间		TOV0 的置位时刻
 * 0		0			0			普通				0xFF	立即更新				MAX
 * 1		0			1			PWM，相位修正	0xFF	TOP					BOTTOM
 * 2		1			0			CTC				OCR0	立即更新				MAX
 * 3		1			1			快速PWM			0xFF	TOP					MAX
*************************************************************************************************************************/

#define TIMER_ASSR					ASSR						/* 异步状态寄存器－ ASSR */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 * 	–		–		–		–		AS0		TCN0UB	OCR0UB	TCR0UB
 * Bit 3 – AS0: 异步T/C0
 * Bit 2 – TCN0UB: T/C0 更新中
 * Bit 1 – OCR0UB: 输出比较寄存器0 更新中
 * Bit 0 – TCR0UB:T/C 控制寄存器0 更新中
*************************************************************************************************************************/

#define TIMER_TIMSK					TIMSK						/* 中断控制寄存器－ TIMSK */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	OCIE2	TOIE2	TICIE1	OCIE1A	OCIE1B	TOIE1	OCIE0	TOIE0
 * Bit 7 – OCIE2:T/C2 输出比较匹配中断使能
 * Bit 6 – TOIE2:T/C2 溢出中断使能
 * Bit 5 – TICIE1: T/C1 输入捕捉中断使能
 * Bit 4 – OCIE1A:T/C1 输出比较 A 匹配中断使能
 * Bit 3 – OCIE1B:T/C1 输出比较 B 匹配中断使能
 * Bit 2 – TOIE1:T/C1 溢出中断使能
 * Bit 1 – OCIE0: T/C0 输出比较匹配中断使能
 * Bit 0 – TOIE0: T/C0 溢出中断使能
*************************************************************************************************************************/
#define TIMER_ETIMSK				ETIMSK						/* 中断控制寄存器－ETIMSK */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	–		–		TICIE3	OCIE3A	OCIE3B	TOIE3	OCIE3C	OCIE1C
 * Bit 7:6 – 保留位
 * Bit 5 – TICIE3:T/C3, 输入捕捉中断使能
 * Bit 4 – OCIE3A:T/C3 输出比较 A 匹配中断使能
 * Bit 3 – OCIE3B:T/C3 输出比较 B 匹配中断使能
 * Bit 2 – TOIE3:T/C3 溢出中断使能
 * Bit 1 – OCIE3C:T/C3 输出比较 C 匹配中断使能
 * Bit 0 – OCIE1C:T/C1 输出比较 C 匹配中断使能
*************************************************************************************************************************/
#define TIMER_TIFR					TIFR						/* 中断标志寄存器－ TIFR */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	OCF2	TOV2	ICF1	OCF1A	OCF1B	TOV1	OCF0	TOV0
 * Bit 7 – OCF2: 输出比较标志2
 * Bit 6 – TOV2:T/C2 溢出标志
 * Bit 5 – ICF1:T/C1 输入捕捉标志位
 * Bit 4 – OCF1A:T/C1 输出比较 A 匹配标志位
 * Bit 3 – OCF1B:T/C1 输出比较 B 匹配标志位
 * Bit 2 – TOV1:T/C1 溢出标志
 * Bit 1 – OCF0: 输出比较标志0
 * Bit 0 – TOV0:T/C0 溢出标志
*************************************************************************************************************************/
#define TIMER_ETIFR					ETIFR						/* 中断标志寄存器－ETIFR */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	–		–		ICF3	OCF3A	OCF3B	TOV3	OCF3C	OCF1C
 * Bit 7:6 – 保留位
 * Bit 5 – ICF3:T/C3 输入捕捉标志位
 * Bit 4 – OCF3A:T/C3 输出比较 A 匹配标志位
 * Bit 3 – OCF3B:T/C3 输出比较 B 匹配标志位
 * Bit 2 – TOV3:T/C3 溢出标志
 * Bit 1 – OCF3C:T/C3 输出比较 C 匹配标志位
 * Bit 0 – OCF1C:T/C1 输出比较 C 匹配标志位
*************************************************************************************************************************/
#define TIMER_SFIOR					SFIOR						/* 特殊功能IO寄存器 */
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 *	TSM		–		–		–		ACME	PUD		PSR0	PSR321
 * Bit 7 – TSM:T/C 同步模式
 * Bit 1 – PSR0: T/C0 预分频器复位
 * Bit 0 – PSR321: T/C3、T/C2 与 T/C1 预分频器复位
*************************************************************************************************************************/


/*************************************************************************************************************************
														  硬件参数宏定义
*************************************************************************************************************************/
#define TIMER3_TOP_VAL			(uint16)(F_CPU / 400)			// 2.5ms对应于当前计数频率的值，该值为计数器溢出值，
																// 计算依据为：2.5 * 10^(-3) * F_CPU

#define TIMER3_INIT_COM_VAL		(uint16)(F_CPU / 2000)			// 0.5ms对应于当前计数频率的值，该值为计数器最小比较匹配值，
																// 计算依据为：0.5 * 10^(-3) * F_CPU


/*************************************************16位定时器的模式设定**************************************************/
#define TIMER16_WGM10_MASK			0x03
#define TIMER16_WGM32_MASK			0x18

/*************************************************************************************************************************
定时器/ 计数器工作模式			TOPOCR1x	更新时刻TOV1		置位时刻
0		普通模式					0xFFFF		立即更新			MAX
1		8位相位修正PWM			0x00FF		TOP				BOTTOM
2		9位相位修正PWM			0x01FF		TOP				BOTTOM
3		10位相位修正PWM			0x03FF		TOP				BOTTOM
4		CTC						OCR1A		立即更新			MAX
5		8位快速PWM				0x00FF		TOP				TOP
6		9位快速PWM				0x01FF		TOP				TOP
7		10位快速PWM				0x03FF		TOP				TOP
8		相位与频率修正PWM		ICR1		BOTTOM			BOTTOM
9		相位与频率修正PWM		OCR1A		BOTTOM			BOTTOM
10		相位修正PWM				ICR1		TOP				BOTTOM
11		相位修正PWM				OCR1A		TOP				BOTTOM
12		CTC						ICR1		立即更新			MAX
13		保留						–			–				–
14		快速PWM					ICR1		TOP				TOP
15		快速PWM					OCR1A		TOP				TOP
*************************************************************************************************************************/
#define TIMER16_MODE_NORMAL			0x00

#define TIMER16_MODE_PWM_P8			0x01
#define TIMER16_MODE_PWM_P9			0x02
#define TIMER16_MODE_PWM_P10		0x03

#define TIMER16_MODE_CTC_OCR		0x04

#define TIMER16_MODE_PWM_FA8		0x05
#define TIMER16_MODE_PWM_FA9		0x06
#define TIMER16_MODE_PWM_FA10		0x07

#define TIMER16_MODE_PWM_PFICR		0x08
#define TIMER16_MODE_PWM_PFOCR		0x09

#define TIMER16_MODE_PWM_PICR		0x0A
#define TIMER16_MODE_PWM_POCR		0x0B

#define TIMER16_MODE_CTC_ICR		0x0C

#define TIMER16_MODE_PWM_FAICR		0x0E
#define TIMER16_MODE_PWM_FAOCR		0x0F

/********************************************* 16位定时器的分频设定*****************************************************/
#define TIMER_CS_MASK				0x07

/*************************************************************************************************************************
0 无时钟源 (T/C 停止)
1 clkI/O/1 ( 无预分频)
2 clkI/O/8 ( 来自预分频器)
3 clkI/O/64 ( 来自预分频器)
4 clkI/O/256 ( 来自预分频器)
5 clkI/O/1024 ( 来自预分频器)
6 外部Tx 引脚，下降沿驱动
7 外部Tx 引脚，上升沿驱动
*************************************************************************************************************************/
#define TIMER16_PRESCALE_STOP		0x00
#define TIMER16_PRESCALE_1			0x01
#define TIMER16_PRESCALE_8			0x02
#define TIMER16_PRESCALE_64			0x03
#define TIMER16_PRESCALE_256		0x04
#define TIMER16_PRESCALE_1024		0x05
#define TIMER16_PRESCALE_TxFALL		0x06
#define TIMER16_PRESCALE_TxRISE		0x07


/********************************************* 16位定时器的中断设定*****************************************************/
#define TIMER16_IMR_MASK			0x3C

/*************************************************************************************************************************
 * TIMSK
 * Bit 7 – OCIE2:T/C2 输出比较匹配中断使能
 * Bit 6 – TOIE2:T/C2 溢出中断使能
 * Bit 5 – TICIE1: T/C1 输入捕捉中断使能
 * Bit 4 – OCIE1A:T/C1 输出比较 A 匹配中断使能
 * Bit 3 – OCIE1B:T/C1 输出比较 B 匹配中断使能
 * Bit 2 – TOIE1:T/C1 溢出中断使能
 * Bit 1 – OCIE0: T/C0 输出比较匹配中断使能
 * Bit 0 – TOIE0: T/C0 溢出中断使能
**************************************************************************************************************************
 * ETIMSK
 * Bit 5 – TICIE3:T/C3, 输入捕捉中断使能
 * Bit 4 – OCIE3A:T/C3 输出比较 A 匹配中断使能
 * Bit 3 – OCIE3B:T/C3 输出比较 B 匹配中断使能
 * Bit 2 – TOIE3:T/C3 溢出中断使能
 * Bit 1 – OCIE3C:T/C3 输出比较 C 匹配中断使能
 * Bit 0 – OCIE1C:T/C1 输出比较 C 匹配中断使能
*************************************************************************************************************************/
#define TIMER16_IMR_OCIC				0x08				// 输出比较 C 匹配中断使能
#define TIMER16_IMR_ICI					0x05				// 输入捕捉中断使能
#define TIMER16_IMR_OCIA				0x04				// 输出比较 A 匹配中断使能
#define TIMER16_IMR_OCIB				0x03				// 输出比较 B 匹配中断使能
#define TIMER16_IMR_OI					0x02				// 溢出中断使能


/***************************************** 16位定时器的比较输出通道模式设置*********************************************/
#define TIMER16_COMA_MASK			0xC0
#define TIMER16_COMB_MASK			0x30
#define TIMER16_COMC_MASK			0x0C

/*************************************************************************************************************************
Bit 7:6 – COM1A1:0: 通道A 的比较输出模式
Bit 5:4 – COM1B1:0: 通道B 的比较输出模式

	COMx1				COMx0
	0					0				普通端口操作， OC1A/OC1B 未连接
	0					1				比较匹配时OC1A/OC1B 电平取反
	1					0				比较匹配时清零OC1A/OC1B( 输出低电平)
	1					1				比较匹配时置位 OC1A/OC1B ( 输出高电平)
*************************************************************************************************************************/
#define TIMER_PWM_DISCONNECT		0x00
#define TIMER_PWM_TOGGLE			0x01
#define TIMER_PWM_CLEAR				0x02
#define TIMER_PWM_SET				0x03

#define TIMER_PWM0_PORT				PORTB
#define TIMER_PWM0_DDR				DDRB
#define TIMER_PWM1_PORT				PORTB
#define TIMER_PWM1_DDR				DDRB
#define TIMER_PWM2_PORT				PORTB
#define TIMER_PWM2_DDR				DDRB
#define TIMER_PWM3_PORT				PORTE
#define TIMER_PWM3_DDR				DDRE

#define TIMER_PWM0_BIT				4
#define TIMER_PWM1A_BIT				5
#define TIMER_PWM1B_BIT				6
#define TIMER_PWM1C_BIT				7
#define TIMER_PWM2_BIT				7
#define TIMER_PWM3A_BIT				3
#define TIMER_PWM3B_BIT				4
#define TIMER_PWM3C_BIT				5


/********************************************* 8位定时器的中断设定*****************************************************/

/*************************************************************************************************************************
 * TIMSK
 * Bit 7 – OCIE2:T/C2 输出比较匹配中断使能
 * Bit 6 – TOIE2:T/C2 溢出中断使能
 * Bit 1 – OCIE0: T/C0 输出比较匹配中断使能
 * Bit 0 – TOIE0: T/C0 溢出中断使能
*************************************************************************************************************************/
#define TIMER8_IMR_OCI				0x01
#define TIMER8_IMR_OI				0x00

/********************************************* 8位定时器的分频设定*****************************************************/

/*************************************************************************************************************************
Timer0
0 无时钟源 (T/C 停止)
1 clkI/O/1 ( 无预分频)
2 clkI/O/8 ( 来自预分频器)
3 clkI/O/32 ( 来自预分频器)
4 clkI/O/64 ( 来自预分频器)
5 clkI/O/128 ( 来自预分频器)
6 clkI/O/256 ( 来自预分频器)
7 clkI/O/1024 ( 来自预分频器)
*************************************************************************************************************************/
#define TIMER0_PRESCALE_STOP		0x00
#define TIMER0_PRESCALE_1			0x01
#define TIMER0_PRESCALE_8			0x02
#define TIMER0_PRESCALE_32			0x03
#define TIMER0_PRESCALE_64			0x04
#define TIMER0_PRESCALE_128			0x05
#define TIMER0_PRESCALE_256			0x06
#define TIMER0_PRESCALE_1024		0x07


/*************************************************************************************************************************
Timer2
0 无时钟源 (T/C 停止)
1 clkI/O/1 ( 无预分频)
2 clkI/O/8 ( 来自预分频器)
3 clkI/O/64 ( 来自预分频器)
4 clkI/O/256 ( 来自预分频器)
5 clkI/O/1024 ( 来自预分频器)
6 外部T2 引脚，下降沿驱动
7 外部T2 引脚，上升沿驱动
*************************************************************************************************************************/
#define TIMER2_PRESCALE_STOP		0x00
#define TIMER2_PRESCALE_1			0x01
#define TIMER2_PRESCALE_8			0x02
#define TIMER2_PRESCALE_64			0x03
#define TIMER2_PRESCALE_256			0x04
#define TIMER2_PRESCALE_1024		0x05
#define TIMER2_PRESCALE_T1FALL		0x06
#define TIMER2_PRESCALE_T1RISE		0x07


/********************************************* 8位定时器的工作模式设定*****************************************************/
#define TIMER8_WGM_MASK				0x48
#define TIMER8_WGM0_OFFSET			0x06
#define TIMER8_WGM1_OFFSET			0x03
/*************************************************************************************************************************
 * 	7		6		5		4		3		2		1		0
 * 	FOC0	WGM00	COM01	COM00	WGM01	CS02	CS01	CS00
 * Bit 7 – FOC0: 强制输出比较
 * Bit 6, 3 – WGM01:0: 波形产生模式
 * Bit 5:4 – COM01:0: 比较匹配输出模式
 * Bit 2:0 – CS02:0: 时钟选择
 *
 * 模式		WGM01		WGM00		T/C的工作模式	TOP		OCR0的更新时间		TOV0 的置位时刻
 * 0		0			0			普通				0xFF	立即更新				MAX
 * 1		0			1			PWM，相位修正	0xFF	TOP					BOTTOM
 * 2		1			0			CTC				OCR0	立即更新				MAX
 * 3		1			1			快速PWM			0xFF	TOP					MAX
*************************************************************************************************************************/
#define TIMER8_MODE_NORMAL			0x00
#define TIMER8_MODE_PWM_P			0x01
#define TIMER8_MODE_CTC_OCR			0x02
#define TIMER8_MODE_PWM_FA			0x03


/*************************************************************************************************************************
												初始化定义
*************************************************************************************************************************/
// 初始化中使用的初始分频值
#define PRESCALE_1					1
#define PRESCALE_8					8
#define PRESCALE_64					54
#define PRESCALE_256				256
#define PRESCALE_1024				1024

// 初始化分频值（256分频）
#define TIMER0_PRESCALE_VAL			PRESCALE_256
// 初始化中断定时值（4ms）
#define TIMER0_TIMING_MS			2


/*************************************************************************************************************************
											根据初始化定义进行的运算
*************************************************************************************************************************/
#if (1 == TIMER0_PRESCALE_VAL)
	#define  TIMER0_PRESCALE_INIT		TIMER0_PRESCALE_1
#elif (8 == TIMER0_PRESCALE_VAL)
	#define  TIMER0_PRESCALE_INIT		TIMER0_PRESCALE_8
#elif (64 == TIMER0_PRESCALE_VAL)
	#define  TIMER0_PRESCALE_INIT		TIMER0_PRESCALE_64
#elif (256 == TIMER0_PRESCALE_VAL)
	#define  TIMER0_PRESCALE_INIT		TIMER0_PRESCALE_256
#elif (1024 == TIMER0_PRESCALE_VAL)
	#define  TIMER0_PRESCALE_INIT		TIMER0_PRESCALE_1024
#else
	#error	!error: The value is meaningless in this model
#endif

// 计算定时器重载值
#define TIMER0_RELOAD_VAL			(0x100 - (uint8)((TIMER0_TIMING_MS * (F_CPU / 1000)) / (uint32)TIMER0_PRESCALE_VAL ))

/*************************************************************************************************************************
												 结构体定义
*************************************************************************************************************************/
typedef struct STimer16Control
{
	volatile uint8 state;

	tVoidVoid* pInit;
	tVoidUint16* pSetCounterRegister;
	tVoidUint16* pSetCompareRegisterA;
	tVoidUint16* pSetCompareRegisterB;
	tVoidUint16* pSetCompareRegisterC;
	tVoidUint16* pSetCaptureRegister;
	tVoidUint8* pSetMode;
	tVoidUint8* pSetPrescaler;
	tVoidUint8*	pSetCompareOutputAMode;
	tVoidUint8*	pSetCompareOutputBMode;
	tVoidUint8*	pSetCompareOutputCMode;
	tVoidUint8* pEnableInterrupt;
	tVoidUint8* pDisableInterrupt;

}TIMER16_CONTROL_STRUCT;


typedef struct STimer8Control
{
	volatile uint8 state;

	tVoidVoid* pInit;
	tVoidUint8* pSetCounterRegister;
	tVoidUint8* pSetCompareRegister;
	tVoidUint8* pSetMode;
	tVoidUint8* pSetPrescaler;
	tVoidUint8*	pSetCompareOutputMode;
	tVoidUint8* pEnableInterrupt;
	tVoidUint8* pDisableInterrupt;

}TIMER8_CONTROL_STRUCT;


/*************************************************************************************************************************
														控制结构体声明
*************************************************************************************************************************/
TIMER_EXT TIMER8_CONTROL_STRUCT timer0_control;
TIMER_EXT TIMER16_CONTROL_STRUCT timer1_control;
TIMER_EXT TIMER16_CONTROL_STRUCT timer3_control;


/*************************************************************************************************************************
														函数声明
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
