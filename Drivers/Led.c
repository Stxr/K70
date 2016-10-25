/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Led.c
** 最后修订日期:		2009-03-18
** 最后版本:			1.0
** 描述:				LED的操作的接口定义(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-18
** 版本:				1.0
** 描述:				初始化LED等操作
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
#include "Drivers/Led.h"


/*************************************************************************************************************************
** 函数名称:			SetLedBit
**
** 函数描述:			对LED进行单个控制。
**                      
**					    
** 输入变量:			uint8 val;
** 返回值:			void
**
** 使用宏或常量:		相关宏定义,参见LED.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:			律晔
** 创建日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetLedBit(uint8 val)
{
	#if LED_PORT_ALONE	
	#else
		SetRegBit(LED_PORT, LED_BIT_BOTTOM + val);
	#endif
}
	

/*************************************************************************************************************************
** 函数名称:			ClrLedBit
**
** 函数描述:			对LED进行单个控制。
**                      
**					    
** 输入变量:			uint8 val;
** 返回值:			void
**
** 使用宏或常量:		相关宏定义,参见LED.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:			律晔
** 创建日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ClrLedBit(uint8 val)
{
	#if LED_PORT_ALONE	
	#else
		ClrRegBit(LED_PORT, LED_BIT_BOTTOM + val);
	#endif
}
	



/*************************************************************************************************************************
** 函数名称:			ChangeLedBit
**
** 函数描述:			对LED进行单个控制。
**                      
**					    
** 输入变量:			uint8 val;
** 返回值:				void
**
** 使用宏或常量:		相关宏定义,参见LED.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:				律晔
** 创建日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ChangeLedBit(uint8 val)
{
	#if LED_PORT_ALONE
	#else

		ChangeRegBit(LED_PORT, LED_BIT_BOTTOM + val);

	#endif
}
	


/*************************************************************************************************************************
** 函数名称:			SetLedGroup
**
** 函数描述:			对四个调试用LED进行成组控制。
**                      
**					    
** 输入变量:			uint8 val;
** 返回值:				void
**
** 使用宏或常量:		相关宏定义,参见LED.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:				律晔
** 创建日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetLedGroup(uint8 val)
{
	#if LED_PORT_ALONE
	#else

		SetReg(LED_PORT, (val << LED_BIT_BOTTOM) & LED_MASK);
	
	#endif
	
}

/*************************************************************************************************************************
** 函数名称:			InitLed
**
** 函数描述:			初始化4个调试用LED，对LED的操作本来应该放在API层完成，
**					但是考虑的他们是用于对系统进行调试的，所以应该尽可能接近底层，所以放在了驱动程序接口层。
**                      				    
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		相关宏定义,参见LED.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:			律晔
** 创建日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitLed(void)
{
	#if LED_PORT_ALONE
	#else

		SetReg(led_control.state, 0);									// 初始化状态寄存器
		
		SetRegBits(LED_DDR, LED_MASK);
		ClrRegBits(LED_PORT, LED_MASK);
		led_control.pSetLedGroup = SetLedGroup;
		led_control.pClrLedBit = ClrLedBit;
		led_control.pSetLedBit = SetLedBit;
		led_control.pChangeLedBit = ChangeLedBit;
		
		SetRegBit(led_control.state, INIT_COMPLETE);					// 置位初始化完成标志

	#endif
}
	


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
LED_CONTROL_STRUCT led_control = { .pInit = InitLed };


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
