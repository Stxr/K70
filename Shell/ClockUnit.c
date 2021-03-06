/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			ClockUnit.c
** 最后修订日期:		2009-11-18
** 最后版本:			1.0
** 描述:				系统时钟单元，用于产生系统时钟，并通过对系统时钟的操作，实现延时等方法。
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-11-18
** 版本:				1.0
** 描述:				时钟触发来源于SystemTask.c中的SystemClockTick()函数。
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
#include "Shell/ClockUnit.h"


/*************************************************************************************************************************
**                                                      外部函数声明
*************************************************************************************************************************/

/*************************************************************************************************************************
** 函数名称:			ClearSystemClock
**
** 函数描述:			系统时钟归零
**                      
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
** 创建日期:			2009-11-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ClearSystemClock(void)
{
	clock_control.counter = 0;
}

/*************************************************************************************************************************
** 函数名称:			GetSystemClock
**
** 函数描述:			读取当期系统时钟值
**
**
** 输入变量:			uint32*;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-11-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void GetSystemClock(uint32* p_val)
{
	*p_val = clock_control.counter;
}

/*************************************************************************************************************************
** 函数名称:			DelaySystemMs
**
** 函数描述:			毫秒级系统延时
**
**
** 输入变量:			uint16;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-11-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void DelaySystemMs(uint16 val)
{
	clock_control.delay_counter = 0;

	while (clock_control.delay_counter <= (val >> 1))
	{
		_delay_us(10);
	}
}

/*************************************************************************************************************************
** 函数名称:			DelaySystemS
**
** 函数描述:			秒级系统延时
**
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
** 创建日期:			2009-11-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void DelaySystemS(uint8 val)
{
	uint32 temp_value = 0;

	temp_value = clock_control.counter;				// 记录当前系统时间
	temp_value += (val * 500);								// 计算延时后的系统时间

	while (clock_control.counter < temp_value);		// 时间没有到则等待;
}


/*************************************************************************************************************************
** 函数名称:			ServeSystemClock
**
** 函数描述:			系统时钟服务函数，在2ms定时系统服务函数中调用
**
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
** 创建日期:			2009-11-18
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void ServeSystemClock(void)
{
	clock_control.counter++;
	clock_control.delay_counter++;
}


/*************************************************************************************************************************
 * 												控制结构体定义
*************************************************************************************************************************/
CLOCK_STRUCT clock_control =
{
	.counter = 0,

	.pClearClock = ClearSystemClock,
	.pGetClock = GetSystemClock,
	.pDelayMs = DelaySystemMs,
	.pDelayS = DelaySystemS,
};

/*************************************************************************************************************************
**												文件结束
*************************************************************************************************************************/
