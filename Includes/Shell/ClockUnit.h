/*******************************************************Copyright*********************************************************
**												北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			ClockUnit.h
** 最后修订日期:		2009-03-29
** 最后版本:			1.0
** 描述:				系统信息状态单元，提供系统信息察看接口，系统初始化接口。
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-29
** 版本:				1.0
** 描述:				实现系统信息查询。
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
#ifndef CLOCKUNIT_H_
#define CLOCKUNIT_H_

#include "ConfigTypes.h"


#define		CLOCKUNIT_GLOBALS

#ifndef		CLOCKUNIT_GLOBALS
     #define	CLOCKUNIT_EXT
#else 
     #define	CLOCKUNIT_EXT	extern
#endif 


/************************************************************************************************************************/
/*												系统状态控制结构体的结构定义												*/
typedef struct SClock
{
	volatile uint32 counter;													// 毫秒计数器,单位为2ms
	volatile uint16 delay_counter;

	void (*pClearClock)(void);
	void (*pGetClock)(uint32* p_val);
	void (*pDelayMs)(uint16 val);
	void (*pDelayS)(uint8 val);
}CLOCK_STRUCT;


CLOCKUNIT_EXT CLOCK_STRUCT clock_control;				// 系统时钟控制结构体

#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
