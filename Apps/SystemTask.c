/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			SystemTask.c
** ����޶�����:  	2009-05-31
** ���汾:			1.0
** ����:				ϵͳ������
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-05-31
** �汾:				1.0
** ����:				ϵͳ������
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
*************************************************************************************************************************/
#include "Apps/SystemTask.h"

extern void ServeSystemClock(void);

/*************************************************************************************************************************
** ��������:			ProcessingCommand
**
** ��������:			ָ�����������
**						
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-05-31
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			SystemClockTick
**
** ��������:			��2ms��ʱ�ж��б����õ�ϵͳ���ģ����ڽ���ϵͳ��������ȷ���
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-05-31
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
														�ṹ������
*************************************************************************************************************************/



/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
