/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			PPM.h
** 最后修订日期:  	2009-03-03
** 最后版本:			1.0
** 描述:				使用16位TIMER的快速PWM模式实现8路PPM控制(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2008-08-05
** 版本:				1.0
** 描述:				使用16位TIMER的快速PWM模式实现8路PPM控制(API)
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
#ifndef PPM_H_
#define PPM_H_

#include "ConfigTypes.h"
#include "Drivers/Timer.h"
#include "Drivers/Led.h"

#define		PPM_GLOBALS


#ifndef   PPM_GLOBALS
     #define PPM_EXT     
#else 
     #define PPM_EXT  extern
#endif 

#define PPM_PORT_ALONE			0
/*************************************************************************************************************************
                                                  PPM输出端口定意
*************************************************************************************************************************/
#if PPM_PORT_ALONE

	#define PPM0_PORT			PORTC							// 定义PPM输出所使用的端口数据寄存器
	#define PPM0_PORT_BIT		PC0								// 定义端口中每一位与PPM编号的对应关系
	#define PPM0_DDR			DDRC							// 定义PPM输出所使用的端口方向寄存器
	#define PPM0_DDR_BIT		DDC0

	#define PPM1_PORT			PORTC		
	#define PPM1_PORT_BIT		PC1	
	#define PPM1_DDR			DDRC	
	#define PPM1_DDR_BIT		DDC1

	#define PPM2_PORT			PORTC		
	#define PPM2_PORT_BIT		PC2	
	#define PPM2_DDR			DDRC	
	#define PPM2_DDR_BIT		DDC2

	#define PPM3_PORT			PORTC		
	#define PPM3_PORT_BIT		PC3	
	#define PPM3_DDR			DDRC	
	#define PPM3_DDR_BIT		DDC3

	#define PPM4_PORT			PORTC	
	#define PPM4_PORT_BIT		PC4		
	#define PPM4_DDR			DDRC	
	#define PPM4_DDR_BIT		DDC4

	#define PPM5_PORT			PORTC	
	#define PPM5_PORT_BIT		PC5		
	#define PPM5_DDR			DDRC	
	#define PPM5_DDR_BIT		DDC5

	#define PPM6_PORT			PORTC	
	#define PPM6_PORT_BIT		PC6		
	#define PPM6_DDR			DDRC	
	#define PPM6_DDR_BIT		DDC6

	#define PPM7_PORT			PORTC		
	#define PPM7_PORT_BIT		PC7			
	#define PPM7_DDR			DDRC	
	#define PPM7_DDR_BIT		DDC7

#else
	
	#define PPM_PORT			PORTC							// 定义PPM输出所使用的端口数据寄存器
	#define PPM_DDR				DDRC							// 定义PPM输出所使用的端口方向寄存器

#endif

/*************************************************************************************************************************
                                                 定时器硬件相关定意
*************************************************************************************************************************/

#define PPM_COMPARE_INT		SIG_OUTPUT_COMPARE3A					// 定义中断入口名称
#define PPM_TIMER_STATE		timer3_control.state					// 定时器状态变量定义
#define SetTimerPosition	(timer3_control.pSetCompareRegisterA)	// 定义PPM位置设置接口函数


/*************************************************************************************************************************
                                                PPM控制值域参数定意
*************************************************************************************************************************/
#define PPM_TOP_VAL			(uint16)(F_CPU / 400)				// 2.5ms对应于当前计数频率的值，该值为计数器溢出值，
																// 计算依据为：2.5 * 10^(-3) * F_CPU
#define PPM_MAX_VAL			(uint16)(F_CPU / 400 - 10)			// 计数器最大比较匹配值，到溢出中断发生有10个周期
#define PPM_MIN_VAL			(uint16)(F_CPU / 2000)				// 0.5ms对应于当前计数频率的值，该值为计数器最小比较匹配值，
																// 计算依据为：0.5 * 10^(-3) * F_CPU
#define PPM_MIDDLE_VAL		(uint16)((3 * F_CPU) / 2000)		// 1.5ms对应于当前计数频率的值，用于初始化

#define PPM_MAX_SPEED		(uint16)((PPM_MAX_VAL - PPM_MIN_VAL) / 15)
																// 最大舵机速度，以0.1秒60度为极限转速设计
																// 相当于12度每20ms，
																// 有计算式：(PPM_MAX_VAL - PPM_MIN_VAL) * 12 / 180

#define PPM_SERVO_MODE		0x0F
#define PPM_MOTO_MODE		0xF0

#define PPM_BROADCASTING	0x1E								// 广播通道号, 为0xFE - 0xE0, 由于PPM在舵机ID段中被定义在E0--EF段

/*************************************************************************************************************************
                                                	结构体定意
*************************************************************************************************************************/
typedef struct													// 定义ppm的状态结构，通过对该结构体的操作，可以改变PPM的状态
{
	volatile uint16 speed;										// PPM速度
	volatile uint16 target_position;							// PPM目标位置	
	volatile uint16 current_position;							// PPM当前位置
	volatile uint16 cw_limit;									// 顺时针位置限制(最小值)
	volatile uint16 ccw_limit;									// 逆时针位置限制(最大值)
	
	volatile uint8 mode;
	volatile uint8 enable_torque;								// 扭矩输出使能标志
}PPM_STATE_STRUCT;


typedef uint8 (tSetPpmLimit)(uint8 channel, uint16 ccw, uint16 cw);
typedef uint8 (tGetPpmLimit)(uint8 channel, uint16* ccw, uint16* cw);
typedef uint8 (tGetPpmPosition)(uint8 channel, uint16* position);
typedef uint8 (tStopPpm)(uint8 id);
typedef uint8 (tSetPpmPosition)(uint8 channel, uint16 position, uint16 speed);
typedef uint8 (tSetPpmAcceleration)(uint8 channel, uint16 acceleration);
typedef uint8 (tSetPpmVelocity)(uint8 channel, uint8 direction, uint16 velocity);
typedef uint8 (tRunPpm)(uint8 channel);
typedef void (tInitPpm)(void);

typedef struct SPpmControl										// 定义ppm的控制结构
{
	volatile uint8 state;
	volatile uint8 channel;										// PPM通道
	volatile uint8 channel_Mapping[8];
	
	tSetPpmLimit* pSetPpmLimit;
	tGetPpmLimit* pGetPpmLimit;
	tGetPpmPosition* pGetPpmPosition;
	tStopPpm* pStopPpm;
	tSetPpmAcceleration* pSetPpmAcceleration;
	tSetPpmVelocity* pSetPpmVelocity;
	
	tSetPpmPosition* pSetPpmPosition;							// 位置设置函数指针
	tRunPpm* pRunPpm;											// 位置变更执行函数指针
	tInitPpm* pInit;											// 初始化指针
}PPM_CONTROL_STRUCT;

/*************************************************************************************************************************
                                                         函数声明
*************************************************************************************************************************/


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
PPM_EXT PPM_CONTROL_STRUCT ppm_control;


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
