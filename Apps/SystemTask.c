/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			SystemTask.c
** 最后修订日期:  	2009-05-31
** 最后版本:			1.0
** 描述:				系统服务函数
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-05-31
** 版本:				1.0
** 描述:				系统服务函数
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
#include "Apps/SystemTask.h"

extern void ServeSystemClock(void);

/*************************************************************************************************************************
** 函数名称:			ProcessingCommand
**
** 函数描述:			指令解析处理函数
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
** 创建日期:			2009-05-31
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void ProcessingCommand(void)
{
	uint8 temp_state = 0;

	temp_state = uprobot_control.pUpRobotSlaveRxPacket(&main_communication.str_stack.str_slave_receive, main_communication.p_uart_control);
	SetRegMask(main_communication.state, ERR_MASK, temp_state);

	if(ERR_OK == temp_state || ERR_CRC == temp_state)
	{
		led_control.pChangeLedBit(3);

		temp_state = command_control.pCommandParse(&main_communication);

		if(temp_state == TRUE)
		{
			uprobot_control.pUpRobotSlaveTxPacket(&main_communication.str_stack.str_slave_send, main_communication.p_uart_control);
		}

	}
}


/*************************************************************************************************************************
** 函数名称:			SystemClockTick
**
** 函数描述:			在2ms定时中断中被调用的系统节拍，用于进行系统命令解析等服务
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
** 创建日期:			2009-05-31
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void SystemClockTick(void)
{
//	uint8 count = 0;
	ServeSystemClock();

	while (main_communication.p_uart_control->pTestReceiveBuffer())
	{
		ProcessingCommand();
	}

	sayn_communications_control.adc_single_sayn_count++;
	if ((sayn_communications_control.adc_single_sayn_time > 0) && (sayn_communications_control.adc_single_sayn_count == sayn_communications_control.adc_single_sayn_time))
	{

		sayn_communications_control.adc_single_sayn_count = 0;
		GetAdcSamplingAtTimes(&main_communication,sayn_communications_control.adc_single_sayn_parameter[0]);

	}

	sayn_communications_control.adc_multi_sayn_count++;
	if ((sayn_communications_control.adc_multi_sayn_time > 0) &&(sayn_communications_control.adc_multi_sayn_count == sayn_communications_control.adc_multi_sayn_time))
	{
		sayn_communications_control.adc_multi_sayn_count = 1;
		GetAdcMultiAtTimes(&main_communication,sayn_communications_control.adc_multi_sayn_parameter[0]);
	}

	sayn_communications_control.adc_all_sayn_count++;
	if ((sayn_communications_control.adc_all_sayn_time > 0) &&(sayn_communications_control.adc_all_sayn_count == sayn_communications_control.adc_all_sayn_time))
	{
		sayn_communications_control.adc_all_sayn_count = 1;
		GetAdcAllAtTimes(&main_communication);
	}

	sayn_communications_control.io_sayn_count++;
	if ((sayn_communications_control.io_sayn_time > 0) &&(sayn_communications_control.io_sayn_count == sayn_communications_control.io_sayn_time))
	{
		sayn_communications_control.io_sayn_count = 1;
		GetIoInputAtTimes(&main_communication);
	}

}


/*************************************************************************************************************************
														结构体声明
*************************************************************************************************************************/



/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
