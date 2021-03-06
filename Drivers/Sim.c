/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Sim.c
** 最后修订日期:		2009-11-22
** 最后版本:			1.0
** 描述:		    	内部外部RAM选择，电源模式设置，中断向量位置设置，看门狗设置等系统控制方法的编程接口
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-11-22
** 版本:				1.0
** 描述:				主要目的为实现系统初始化算法和Reset算法
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
#include "Drivers/Sim.h"


/*************************************************************************************************************************
** 函数名称:			EnableExtRam
**
** 函数描述:			使能外部RAM
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void EnableExtRam(void)
{
	SetReg(SIM_XMCRA, 0x00);
	SetReg(SIM_XMCRB, 0x00);
	SetReg(EXT_RAM_DDR, 0xFF);
	SetReg(EXT_RAM_PORT, 0x00);
	SetRegBit(SIM_MCUCR, SRE);
}


/*************************************************************************************************************************
** 函数名称:			DisableExtRam
**
** 函数描述:			禁用外部RAM
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void DisableExtRam(void)
{
	ClrRegBit(SIM_MCUCR, SRE);
}


/*************************************************************************************************************************
** 函数名称:			SetXtalDivide
**
** 函数描述:			时钟分频设置
**
**
** 输入变量:		    uint8 mode;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			SetRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetXtalDivide(uint8 val)
{
	SetReg(SIM_XDIV, val & 0x7F);								// 取低七位
}


/*************************************************************************************************************************
** 函数名称:			EnableXtalDivide
**
** 函数描述:			时钟分频使能
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void EnableXtalDivide(void)
{
	SetRegBit(SIM_XDIV, XDIVEN);
}


/*************************************************************************************************************************
** 函数名称:			DisableXtalDivide
**
** 函数描述:			时钟分频禁用
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void DisableXtalDivide(void)
{
	ClrRegBit(SIM_XDIV, XDIVEN);
}


/*************************************************************************************************************************
** 函数名称:			ResetSoft
**
** 函数描述:			重启
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ResetSoft(void)
{
	wdt_enable(WDTO_15MS);

	while (1)
	{
		_delay_loop_1(100);
	}
}


/*************************************************************************************************************************
** 函数名称:			Init
**
** 函数描述:			初始化
**
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
** 创建日期:			2009-11-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitSim(void)
{
	SetReg(sim_control.state , 0x00);								// 初始化状态寄存器

	SetReg(SIM_MCUSR, 0x00);
	wdt_disable();													// 初始化看门狗

	DisableExtRam();
	DisableXtalDivide();

	sim_control.pEnableExtRam = EnableExtRam;
	sim_control.pDisableExtRam = DisableExtRam;
	sim_control.pEnableXtalDivide = EnableXtalDivide;
	sim_control.pDisableXtalDivide = DisableXtalDivide;
	sim_control.pSetXtalDivide = SetXtalDivide;
	sim_control.pReset = ResetSoft;

	SetRegBit(sim_control.state, INIT_COMPLETE);					// 置位初始化完成标志
}


/*************************************************************************************************************************
**														控制结构体声明
*************************************************************************************************************************/
SIM_CONTROL_STRUCT sim_control =
{
	.pInit = InitSim,
};


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
