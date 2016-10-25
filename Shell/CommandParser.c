/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			CommandParser.c
** ����޶�����:		2009-03-20
** ���汾:			1.0
** ����:				��������������ڽ�����λ����ָ�������Ӧ��ִ�к���������Ӧ�����ݡ�
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-20
** �汾:				1.0
** ����:				������λ����ָ�������Ӧ��ִ�к���������Ӧ�����ݡ�
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
#include "ConfigTypes.h"

#include "Drivers/Adc.h"
#include "Drivers/Led.h"
#include "Drivers/Gpio.h"

#include "Shell/CommandParser.h"
#include "Shell/SystemUnit.h"
#include "Shell/ServoUnit.h"

/*************************************************************************************************************************
** ��������:			FillSlaveSendStruct
**
** ��������:			�Խṹ�巽ʽ���ӻ����ͽṹ�塣
**
**
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			�쿡��
** �޶�����:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/

static void FillSlaveSendStruct(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 parameter_length,uint8 * parameter )
{
	uint8 count = 0;
	p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
	p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
	p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

	p_communication_str->str_stack.str_slave_send.parameter_length = parameter_length;



	for(count = 0; count < parameter_length; count++)
	{
		p_communication_str->str_stack.str_slave_send.parameter[count] = parameter[count];
	}

	p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(p_communication_str->state, ERR_MASK, ERR_OK);	// ����״̬��־
}

/*************************************************************************************************************************
** ��������:			FillSlaveSendStruct
**
** ��������:			�����鷽ʽ���ӻ����ͽṹ�塣
**
**
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			�쿡��
** �޶�����:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/

static void FillSlaveSendStructByArray(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_functional,uint8 * p_parameter )
{
	uint8 count = 0;

	p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
	p_communication_str->str_stack.str_slave_send.parameter_length = p_functional[0];
	p_communication_str->str_stack.str_slave_send.functional_unit = p_functional[1];
	p_communication_str->str_stack.str_slave_send.method_code = p_functional[2];

	for(count = 0; count < p_functional[0]; count++)
	{
		p_communication_str->str_stack.str_slave_send.parameter[count] = p_parameter[count];
	}

	p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(p_communication_str->state, ERR_MASK, ERR_OK);	// ����״̬��־
}



/*************************************************************************************************************************
** ��������:			StateResponse
**
** ��������:			 ״̬Ӧ�𣬱������ὫͨѶ�еĴ�������ͨ��Stateλ������λ������ͨѶ���������޷���ֵ����ʱʹ��;
**                      
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void StateResponse(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
	p_communication_str->str_stack.str_slave_send.parameter_length = 0;							// ���ز�������Ϊ0
	
	p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
	p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
	
	p_communication_str->str_stack.str_slave_send.state_byte = p_communication_str->state;		// ����״̬��־
}


/*************************************************************************************************************************
** ��������:			AdcSamplingActuator
**
** ��������:			Adc����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**                      
**					                 
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 AdcSamplingActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_ADC_VAL					0x00			// 0�ŷ���
	#define GET_ADC_VAL_LEN				0x01			// 0�ŷ����Ĳ�������
	
	#define GET_ADC_VAL_SINGLE_AT_TIMES		0x60
	#define GET_ADC_VAL_MULTI_AT_TIMES		0x61
	#define GET_ADC_VAL_ALL_AT_TIMES		0x62


//	uint8 count = 0;
	
	uint8 temp_8bit_parameter[8];						// ����������
	uint16 temp_16bit_parameter[4];						// ����������
	uint8 *p_parameter;
//	uint8 temp;
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET��
		*****************************************************************************************************************/
		case GET_ADC_VAL:
		{
			if(GET_ADC_VAL_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);
				temp_8bit_parameter[7] = adc_control.pGetValue(temp_8bit_parameter[0], &temp_16bit_parameter[0]);
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					Out16(temp_8bit_parameter[2], temp_8bit_parameter[1], temp_16bit_parameter[0])
					p_parameter = temp_8bit_parameter;
					FillSlaveSendStruct(p_communication_str,3,p_parameter);
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}

		}
		case GET_ADC_VAL_SINGLE_AT_TIMES:
		{
			sayn_communications_control.adc_single_sayn_parameter[0] = p_communication_str->str_stack.str_slave_receive.parameter[0];
			sayn_communications_control.adc_single_sayn_parameter[1] = p_communication_str->str_stack.str_slave_receive.parameter[1];
			if (sayn_communications_control.adc_single_sayn_parameter[0]<8)
			{
				sayn_communications_control.adc_single_sayn_time = p_communication_str->str_stack.str_slave_receive.parameter[1]*5;
				if((sayn_communications_control.adc_single_sayn_time<10) && (sayn_communications_control.adc_single_sayn_time>0))
				{
					sayn_communications_control.adc_single_sayn_time = 10;
				}
				if(GetAdcSamplingAtTimes(p_communication_str,sayn_communications_control.adc_single_sayn_parameter[0]) == ERR_OK)
				{
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_RANGE;				// ��������Χ
			}


		}
		case GET_ADC_VAL_MULTI_AT_TIMES:
		{

			sayn_communications_control.adc_multi_sayn_parameter[0] = p_communication_str->str_stack.str_slave_receive.parameter[0];
			sayn_communications_control.adc_multi_sayn_parameter[1] = p_communication_str->str_stack.str_slave_receive.parameter[1];
			sayn_communications_control.adc_multi_sayn_time = p_communication_str->str_stack.str_slave_receive.parameter[1]*5;
			if((sayn_communications_control.adc_multi_sayn_time<10) && (sayn_communications_control.adc_multi_sayn_time>0))
			{
				sayn_communications_control.adc_multi_sayn_time = 10;
			}
			if(GetAdcMultiAtTimes(p_communication_str,sayn_communications_control.adc_multi_sayn_parameter[0]) == ERR_OK)
			{
				return ERR_OK;
			}
			else
			{
				return ERR_NOTAVAIL;
			}

		}
		case GET_ADC_VAL_ALL_AT_TIMES:
		{

			sayn_communications_control.adc_all_sayn_parameter[0] = p_communication_str->str_stack.str_slave_receive.parameter[0];
			sayn_communications_control.adc_all_sayn_time = p_communication_str->str_stack.str_slave_receive.parameter[0]*5;
			if((sayn_communications_control.adc_all_sayn_time<10) && (sayn_communications_control.adc_all_sayn_time>0))
			{
				sayn_communications_control.adc_all_sayn_time = 10;
			}
			if(GetAdcAllAtTimes(p_communication_str) == ERR_OK)
			{
				return ERR_OK;
			}
			else
			{
				return ERR_NOTAVAIL;
			}

		}
		/*****************************************************************************************************************/
		default:
		{
			return ERR_VALUE;							// ����ķ����޷�ʹ��
		}
	}
}



/*************************************************************************************************************************
** ��������:			GetAdcSamplingAtTimes
**
** ��������:			��ʱ��ȡ��·ADCֵ�����Ӵ��ڷ��ء�
**
**
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcSamplingAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id)
{


	uint8 temp_8bit_parameter[8];						// ����������
	uint16 temp_16bit_parameter[4];						// ����������
	uint8 functional[3];
	uint8 temp_parameter_length = 3;

	functional[0]= temp_parameter_length; 								//����λ����
	functional[1] = sayn_communications_control.adc_single_sayn_functional;
	functional[2] = sayn_communications_control.adc_single_sayn_method;
	temp_8bit_parameter[0] = (sayn_communications_control.adc_single_sayn_parameter[0]);
	temp_8bit_parameter[7] = adc_control.pGetValue(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

	if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
	{
		Out16(temp_8bit_parameter[2], temp_8bit_parameter[1], temp_16bit_parameter[0]);
		FillSlaveSendStructByArray(p_communication_str,&functional[0],&temp_8bit_parameter[0]);
		uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);					return ERR_OK;
		return ERR_OK;
	}
	else
	{
		return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
	}
}

/*************************************************************************************************************************
** ��������:			GetAdcMultiAtTimes
**
** ��������:			��ʱ��ȡ��·ADCֵ�����Ӵ��ڷ��ء�
**
**
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id)
{
	uint8 temp_8bit_parameter[18];
	uint16 temp_16bit_parameter[9];						// ����������
	uint8 functional[3];
	uint8 i = 0, temp_parameter_length=1, adc_no=0;
	uint8 temp = 0;

	temp_8bit_parameter[0] = adc_id;
//	uint8 test[3];



	if(adc_id != 0 )
	{
		for(i=0; i<8; i++)
		{
			if(GetRegBit(adc_id, i))
			{

				temp &= adc_control.pGetValue(i, &temp_16bit_parameter[adc_no]);
				adc_no++;
				temp_parameter_length +=2 ;

/*				functional[0] = 1;
				functional[1] = sayn_communications_control.adc_multi_sayn_functional;
				functional[2] = sayn_communications_control.adc_multi_sayn_method;
				test[0] = i;
				FillSlaveSendStructByArray(p_communication_str,&functional[0],&test[0]);
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
*/
			}
		}
		functional[0]= temp_parameter_length;
		for(i=0; i<adc_no; i++)
		{
			Out16(temp_8bit_parameter[i*2+2], temp_8bit_parameter[i*2+1], temp_16bit_parameter[i])
		}
		temp_8bit_parameter[0] = adc_id;
		functional[1] = sayn_communications_control.adc_multi_sayn_functional;
		functional[2] = sayn_communications_control.adc_multi_sayn_method;

		if(ERR_OK == temp)											// ��ȡ�ɹ�
		{

			FillSlaveSendStructByArray(p_communication_str,&functional[0],&temp_8bit_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);

			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;
		}

	}
	else
	{
		return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
	}
}


/*************************************************************************************************************************
** ��������:			GetAdcAllAtTimes
**
** ��������:			��ʱ��ȡ����ADCֵ�����Ӵ��ڷ��ء�
**
**
** �������			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcAllAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	uint8 temp_8bit_parameter[10];
	uint16 temp_16bit_parameter[8];						// ����������
	uint8 functional[3];
	uint8 adc_id = 0;
	uint8 temp_parameter_length=8 ;
	uint8 temp = 0;

	for(adc_id=0; adc_id<temp_parameter_length; adc_id++)
	{
		temp &= adc_control.pGetValue(adc_id, &temp_16bit_parameter[adc_id]);
	}
	functional[0]= temp_parameter_length*2;
	functional[1] = sayn_communications_control.adc_all_sayn_functional;
	functional[2] = sayn_communications_control.adc_all_sayn_method;


	for(adc_id=0; adc_id<temp_parameter_length; adc_id++)
	{
		Out16(temp_8bit_parameter[adc_id*2+1], temp_8bit_parameter[adc_id*2], temp_16bit_parameter[adc_id]);
	}


	if(ERR_OK == temp)											// ��ȡ�ɹ�
	{

		FillSlaveSendStructByArray(p_communication_str,&functional[0],&temp_8bit_parameter[0]);
		uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);					return ERR_OK;
		return ERR_OK;
	}
	else
	{
		return ERR_NOTAVAIL;
	}
}
/*************************************************************************************************************************
** ��������:			GpioActuator
**
** ��������:			Gpio����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**                      
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GpioActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_INPUT_VAL					0x00						// 0�ŷ���
	#define GET_INPUT_VAL_LEN						0x00				// 0�ŷ����Ĳ�������
	#define GET_INPUT_VAL_BACK_LEN							0x02		// 0�ŷ����Ĳ�������

	#define SET_DIRECTION_VAL				0x20
	#define SET_DIRECTION_VAL_LEN					0x02
		
	#define SET_OUTPUT_VAL					0x21					
	#define SET_OUTPUT_VAL_LEN						0x02
	
	#define GET_INPUT_VAL_AT_TIMES			0X60
	#define GET_INPUT_VAL_AT_TIMES_LEN			0X01
	#define GET_INPUT_VAL_AT_TIMES_BACK_LEN			0X02

	
	
	uint8 count = 0;
	
	uint8 temp_8bit_parameter[8];						// ����������
	uint16 temp_16bit_parameter[4];						// ����������
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET��
		*****************************************************************************************************************/
		case GET_INPUT_VAL:
		{
			if(GET_INPUT_VAL_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				(gpio_control.pGetInputFilte)(&temp_16bit_parameter[0]);
				
				p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
				
				p_communication_str->str_stack.str_slave_send.parameter_length = GET_INPUT_VAL_BACK_LEN;
				
				Out16(temp_8bit_parameter[1], temp_8bit_parameter[0], temp_16bit_parameter[0])
				
				for(count = 0; count < GET_INPUT_VAL_BACK_LEN; count++)
				{
					p_communication_str->str_stack.str_slave_send.parameter[count] = temp_8bit_parameter[count];
				}
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;

			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
		}
		
		/*****************************************************************************************************************
															SET��
		*****************************************************************************************************************/
		case SET_DIRECTION_VAL:
		{
			if(SET_DIRECTION_VAL_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_DIRECTION_VAL_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				In16(temp_16bit_parameter[0], temp_8bit_parameter[1], temp_8bit_parameter[0]);
				
				(gpio_control.pSetGpioDdr)(temp_16bit_parameter[0]);
			
				p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
				
				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		
		case SET_OUTPUT_VAL:
		{
			if(SET_OUTPUT_VAL_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_OUTPUT_VAL_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				In16(temp_16bit_parameter[0], temp_8bit_parameter[1], temp_8bit_parameter[0]);
				
				(gpio_control.pSetOutputPort)(temp_16bit_parameter[0]);
			
				p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
				
				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case GET_INPUT_VAL_AT_TIMES:
		{
			if(GET_INPUT_VAL_AT_TIMES_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				sayn_communications_control.io_sayn_time = p_communication_str->str_stack.str_slave_receive.parameter[0];
				if((sayn_communications_control.io_sayn_time<5) && (sayn_communications_control.io_sayn_time>0))
				{
					sayn_communications_control.io_sayn_time = 5;
				}
				if (GetIoInputAtTimes(p_communication_str) == ERR_OK)
				{
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;
				}

			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
		}

	
		/*****************************************************************************************************************/
		default:
		{
			return ERR_VALUE;							// ����ķ����޷�ʹ��
		}
	}
}
/*************************************************************************************************************************
** ��������:			GpioActuator
**
** ��������:			Gpio����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetIoInputAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str)
{

	uint8 temp_8bit_parameter[2];
	uint16 temp_16bit_parameter[2];						// ����������
	uint8 * p_parameter;									//���ں������ݵ�һ����ʱָ�����
	uint8 p_functional[3];
	uint8 parameter_length = 2;

	(gpio_control.pGetInputFilte)(&temp_16bit_parameter[0]);

	p_functional[0] = parameter_length;
	p_functional[1] = sayn_communications_control.io_sayn_functional;
	p_functional[2] = sayn_communications_control.io_sayn_method;

	Out16(temp_8bit_parameter[1], temp_8bit_parameter[0], temp_16bit_parameter[0])
	p_parameter = &temp_8bit_parameter[0];
	FillSlaveSendStructByArray(p_communication_str,p_functional,p_parameter);
	uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);

	return ERR_OK;


}

/*************************************************************************************************************************
** ��������:			SignalControlActuator
**
** ��������:			SignalControl����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SignalControlActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_SIGNAL_BYTE					0x00						// 0�ŷ���
	#define GET_SIGNAL_BYTE_LEN						0x01				// 0�ŷ����Ĳ�������
	#define GET_SIGNAL_BYTE_BACK_LEN						0x02		// 0�ŷ����Ĳ�������

	#define SET_SIGNAL_BYTE					0x20
	#define SET_SIGNAL_BYTE_LEN						0x02


	uint8 count = 0;

	uint8 temp_8bit_parameter[8];						// ����������
//	uint16 temp_16bit_parameter[4];						// ����������

	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
														GET��
		*****************************************************************************************************************/
		case GET_SIGNAL_BYTE:
		{
			if(GET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);

				temp_8bit_parameter[7] = (signal_control.pGetByte)(temp_8bit_parameter[0], &temp_8bit_parameter[1]);

				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SIGNAL_BYTE_BACK_LEN;


					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					p_communication_str->str_stack.str_slave_send.parameter[1] = temp_8bit_parameter[1];


					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־

					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
		}

		/*****************************************************************************************************************
															SET��
		*****************************************************************************************************************/
		case SET_SIGNAL_BYTE:
		{
			if(SET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SIGNAL_BYTE_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}

				(signal_control.pSetByte)(temp_8bit_parameter[0], temp_8bit_parameter[1]);

				p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0

				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־

				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		default:
		{
			return ERR_VALUE;							// ����ķ����޷�ʹ��
			break;
		}
	}
}


/*************************************************************************************************************************
** ��������:			SystemStateActuator
**
** ��������:			SystemState����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**
**
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-11-04
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
//static uint8 SystemStateActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
//{
////	#define GET_SIGNAL_BYTE					0x00						// 0�ŷ���
////	#define GET_SIGNAL_BYTE_LEN						0x01				// 0�ŷ����Ĳ�������
////	#define GET_SIGNAL_BYTE_BACK_LEN						0x02		// 0�ŷ����Ĳ�������
//
//	#define RESET_SYSTEM					0x40
//	#define RESET_SYSTEM_LEN						0x00
//
//
//	uint8 count = 0;
//
//	uint8 temp_8bit_parameter[8];						// ����������
////	uint16 temp_16bit_parameter[4];						// ����������
//
//	switch(p_communication_str->str_stack.str_slave_receive.method_code)
//	{
//		/*****************************************************************************************************************
//														GET��
//		*****************************************************************************************************************/
//		case GET_SIGNAL_BYTE:
//		{
//			if(GET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
//			{
//				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);
//
//				temp_8bit_parameter[7] = (signal_control.pGetByte)(temp_8bit_parameter[0], &temp_8bit_parameter[1]);
//
//				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
//				{
//					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
//					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
//					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
//
//					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SIGNAL_BYTE_BACK_LEN;
//
//
//					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
//					p_communication_str->str_stack.str_slave_send.parameter[1] = temp_8bit_parameter[1];
//
//
//					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
//
//					return ERR_OK;
//				}
//				else
//				{
//					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
//				}
//			}
//			else
//			{
//				return ERR_VALUE;						// �������Ȳ���ȷ
//			}
//		}
//
//		/*****************************************************************************************************************
//															SET��
//		*****************************************************************************************************************/
//		case SET_SIGNAL_BYTE:
//		{
//			if(SET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
//			{
//				for(count = 0; count < SET_SIGNAL_BYTE_LEN; count++)
//				{
//					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
//				}
//
//				(signal_control.pSetByte)(temp_8bit_parameter[0], temp_8bit_parameter[1]);
//
//				p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
//				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
//
//				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
//				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
//
//				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
//
//				return ERR_OK;
//			}
//			else
//			{
//				return ERR_VALUE;						// �������Ȳ���ȷ
//			}
//			break;
//		}
//		default:
//		{
//			return ERR_VALUE;							// ����ķ����޷�ʹ��
//			break;
//		}
//	}
//}

/*************************************************************************************************************************
** ��������:			ServoActuator
**
** ��������:			Servo����ģ��ִ�к��������ڰ��շ�����ŵ�����Ӧ��Ӳ�������ӿڡ�
**                      
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-24
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ServoActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_SERVO_POSITION					0x00					// GET�� 0�ŷ���
	#define GET_SERVO_POSITION_LEN						0x01			// GET�� 0�ŷ����Ĳ�������
	#define GET_SERVO_POSITION_BACK_LEN							0x03	// GET�� 0�ŷ����ķ��ز�������
	
	#define GET_SERVO_LIMIT						0x01
	#define GET_SERVO_LIMIT_LEN							0x01	
	#define GET_SERVO_LIMIT_BACK_LEN							0x05
	
	#define GET_SERVO_LOAD						0x02					// GET�� 2�ŷ���
	#define GET_SERVO_LOAD_LEN						0x01			// GET�� 2�ŷ����Ĳ�������
	#define GET_SERVO_LOAD_BACK_LEN							0x03	// GET�� 2�ŷ����ķ��ز�������

	
	#define SET_SERVO_MODE						0x20					// SET�� 0�ŷ���
	#define SET_SERVO_MODE_LEN							0x02			// 
	
	#define SET_SERVO_POSITION					0x21
	#define SET_SERVO_POSITION_LEN						0x05
	
	#define SET_SERVO_VELOCITY					0x22
	#define SET_SERVO_VELOCITY_LEN						0x04
	
	#define SET_SERVO_LIMIT						0x23
	#define SET_SERVO_LIMIT_LEN							0x05
	
	#define SET_SERVO_ID						0x24
	#define SET_SERVO_ID_LEN							0x02

	#define SET_SERVO_TORQUE					0x25
	#define SET_SERVO_TORQUE_LEN						0x02


	#define ACTION_SERVO						0x40
	#define ACTION_SERVO_LEN							0x01
	
	#define TEST_SERVO							0x41
	#define TEST_SERVO_LEN								0x01
	#define TEST_SERVO_BACK_LEN									0x01
	
	#define GET_SERVO_POSITION_MULTI_AT_TIMES	0X60

	#define SET_SERVO_MODE_MULTI_AT_TIMES		0X70
	#define SET_SERVO_POSITION_MULTI_AT_TIMES	0X71
	#define SET_SERVO_VELOCITY_MULTI_AT_TIMES	0X72
	#define SET_SERVO_TORQUE_MULTI_AT_TIMES		0X73

	

	uint8 count = 0;
	uint8 temp_8bit_parameter[8];								// ����������
	uint16 temp_16bit_parameter[4];								// ����������
//	uint8 test[3];
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET��
		*****************************************************************************************************************/
		case GET_SERVO_POSITION:
		{
			if(GET_SERVO_POSITION_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);


				temp_8bit_parameter[7] = (servo_control.pGetPosition)(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

				if(TRUE == temp_8bit_parameter[7])																		// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_POSITION_BACK_LEN;		// ���ز�������Ϊ3
					
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					
					Out16(temp_8bit_parameter[3], temp_8bit_parameter[2], temp_16bit_parameter[0])
					
					for(count = 0; count < GET_SERVO_POSITION_BACK_LEN - 1; count++)
					{
						p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
					}
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);


					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case GET_SERVO_LIMIT:
		{
			if(GET_SERVO_LIMIT_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);

				temp_8bit_parameter[7] = (servo_control.pGetAngleLimit)(temp_8bit_parameter[0], &temp_16bit_parameter[0], &temp_16bit_parameter[1]);

				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_LIMIT_BACK_LEN;		// ���ز�������Ϊ5
					
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					
					Out16(temp_8bit_parameter[2], temp_8bit_parameter[3], temp_16bit_parameter[0]);
					Out16(temp_8bit_parameter[4], temp_8bit_parameter[5], temp_16bit_parameter[1]);
					
					for(count = 0; count < GET_SERVO_LIMIT_BACK_LEN - 1; count++)
					{
						p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
					}
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		

		case GET_SERVO_LOAD:
				{
					if(GET_SERVO_LOAD_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
					{
						temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);


						temp_8bit_parameter[7] = (servo_control.pGetLoad)(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

						if(TRUE == temp_8bit_parameter[7])																		// ��ȡ�ɹ�
						{
							p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
							p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_LOAD_BACK_LEN;		// ���ز�������Ϊ3

							p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];

							Out16(temp_8bit_parameter[3], temp_8bit_parameter[2], temp_16bit_parameter[0])

							for(count = 0; count < GET_SERVO_LOAD_BACK_LEN - 1; count++)
							{
								p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
							}

							p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
							p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

							p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
							uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
							return ERR_OK;
						}
						else
						{
							return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
						}
					}
					else
					{
						return ERR_VALUE;						// �������Ȳ���ȷ
					}
					break;
				}


		/*****************************************************************************************************************
															SET��
		*****************************************************************************************************************/
		case SET_SERVO_MODE:
		{
			if(SET_SERVO_MODE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SERVO_MODE_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				temp_8bit_parameter[7] = (servo_control.pSetMode)(temp_8bit_parameter[0], temp_8bit_parameter[1]);
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case SET_SERVO_POSITION:
		{
			if(SET_SERVO_POSITION_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SERVO_POSITION_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				In16(temp_16bit_parameter[0], temp_8bit_parameter[2], temp_8bit_parameter[1]);
				In16(temp_16bit_parameter[1], temp_8bit_parameter[4], temp_8bit_parameter[3]);
				
				temp_8bit_parameter[7] = (servo_control.pSetPosition)(temp_8bit_parameter[0], temp_16bit_parameter[0], temp_16bit_parameter[1]);
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case SET_SERVO_VELOCITY:
		{
			if(SET_SERVO_VELOCITY_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SERVO_VELOCITY_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				In16(temp_16bit_parameter[0], temp_8bit_parameter[3], temp_8bit_parameter[2]);
				
				temp_8bit_parameter[7] = (servo_control.pSetVelocity)(temp_8bit_parameter[0], temp_8bit_parameter[1], temp_16bit_parameter[0]);
				
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case SET_SERVO_LIMIT:
		{
			if(SET_SERVO_LIMIT_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SERVO_LIMIT_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				In16(temp_16bit_parameter[0], temp_8bit_parameter[2], temp_8bit_parameter[1]);
				In16(temp_16bit_parameter[1], temp_8bit_parameter[4], temp_8bit_parameter[3]);
				
				temp_8bit_parameter[7] = (servo_control.pSetAngleLimit)(temp_8bit_parameter[0], temp_16bit_parameter[0], temp_16bit_parameter[1]);
				
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case SET_SERVO_ID:
		{
			if(SET_SERVO_ID_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < SET_SERVO_ID_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				temp_8bit_parameter[7] = (servo_control.pSetId)(temp_8bit_parameter[0], temp_8bit_parameter[1]);
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}

		case SET_SERVO_TORQUE:
			{
				if(SET_SERVO_TORQUE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
				{
					for(count = 0; count < SET_SERVO_TORQUE_LEN; count++)
					{
						temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
					}

					temp_8bit_parameter[7] = (servo_control.pSetTorqueEnable)(temp_8bit_parameter[0], temp_8bit_parameter[1]);
					if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
					{
						p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
						p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ0

						p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
						p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

						p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
						uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
						return ERR_OK;
					}
					else
					{
						return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
					}
				}
				else
				{
					return ERR_VALUE;						// �������Ȳ���ȷ
				}
				break;
			}
		
		/*****************************************************************************************************************
															FUNCTION��
		*****************************************************************************************************************/
		case ACTION_SERVO:
		{
			if(ACTION_SERVO_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < ACTION_SERVO_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				temp_8bit_parameter[7] = (servo_control.pActionSync)();
				
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// ���ز�������Ϊ2
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case TEST_SERVO:
		{
			if(TEST_SERVO_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				for(count = 0; count < TEST_SERVO_LEN; count++)
				{
					temp_8bit_parameter[count] = (p_communication_str->str_stack.str_slave_receive.parameter[count]);
				}
				
				temp_8bit_parameter[7] = (servo_control.pTest)(temp_8bit_parameter[0], &temp_8bit_parameter[6]);
				
				if(TRUE == temp_8bit_parameter[7])											// ��ȡ�ɹ�
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = TEST_SERVO_BACK_LEN;		// ���ز�������Ϊ1
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[6];
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// ����״̬��־
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
				}
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case GET_SERVO_POSITION_MULTI_AT_TIMES:
		{

			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //��Ҫ��ѯ�Ķ����
			{
				sayn_communications_control.servo_get_position_time = p_communication_str->str_stack.str_slave_receive.parameter[p_communication_str->str_stack.str_slave_receive.parameter_length-1];
				sayn_communications_control.servo_get_position_num = p_communication_str->str_stack.str_slave_receive.parameter_length-1;


				for(count=0; count<sayn_communications_control.servo_get_position_num; count++)
				{
					sayn_communications_control.servo_get_position_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				GetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_get_position_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		
		case SET_SERVO_POSITION_MULTI_AT_TIMES:
		{


			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //��Ҫ��ѯ�Ķ����
			{
				sayn_communications_control.servo_set_position_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//���ݳ���

				for(count=0; count<sayn_communications_control.servo_set_position_num; count++)//��������洢����
				{
					sayn_communications_control.servo_set_position_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_position_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		case SET_SERVO_MODE_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //��Ҫ��ѯ�Ķ����
			{
				sayn_communications_control.servo_set_mode_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//���ݳ���

				for(count=0; count<sayn_communications_control.servo_set_mode_num; count++)//��������洢����
				{
					sayn_communications_control.servo_set_mode_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_mode_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}

			break;
		}
		case SET_SERVO_TORQUE_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //��Ҫ���õĶ����
			{
				sayn_communications_control.servo_set_torque_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//���ݳ���

				for(count=0; count<sayn_communications_control.servo_set_torque_num; count++)//��������洢����
				{
					sayn_communications_control.servo_set_torque_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_torque_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}

			break;
		}
		/*****************************************************************************************************************/
		case SET_SERVO_VELOCITY_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length > 0)
			{
				sayn_communications_control.servo_set_velocity_num = p_communication_str->str_stack.str_slave_receive.parameter_length;
				for(count=0; count<sayn_communications_control.servo_set_velocity_num; count++)//��������洢����
				{
					sayn_communications_control.servo_set_velocity_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoVelocityMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_velocity_parameter[0]);
				return ERR_OK;

			}
			else
			{
				return ERR_VALUE;						// �������Ȳ���ȷ
			}
			break;
		}
		default:
		{
			return ERR_VALUE;							// ����ķ����޷�ʹ��
			break;
		}	
	}
}

uint8 GetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
	uint8 temp_8bit_parameter[61];		//�洢λ��
	uint16 temp_16bit_parameter[30];  //�洢λ��
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = sayn_communications_control.servo_get_position_num *3;
	functional[1] = sayn_communications_control.servo_get_position_functional;
	functional[2] = sayn_communications_control.servo_get_position_method;

	for(count=0; count<sayn_communications_control.servo_get_position_num; count++)
	{
		temp_8bit_parameter[60] &= (servo_control.pGetPosition)(sayn_communications_control.servo_get_position_parameter[count],&temp_16bit_parameter[count] );
	}
	if(TRUE == temp_8bit_parameter[60])																		// ��ȡ�ɹ�
	{
		for(count=0 ; count<(sayn_communications_control.servo_get_position_num); count++)
		{
			temp_8bit_parameter[count] = sayn_communications_control.servo_get_position_parameter[count];
			Out16(temp_8bit_parameter[count*3+2], temp_8bit_parameter[count*3+1], temp_16bit_parameter[0]);
		}
		FillSlaveSendStructByArray(p_communication_str,&functional[0],&temp_8bit_parameter[0]);
		uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
		return ERR_OK;
		}
	else
	{
		return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
	}
}
uint8 SetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
//	uint8 temp_8bit_parameter[151];		//�洢λ��
	uint16 temp_16bit_parameter[60];  //�洢λ��
	uint8 functional[3];
//	uint8 test[6];
	uint8 count = 0;

	functional[0] = 0;
	functional[1] = sayn_communications_control.servo_set_position_functional;
	functional[2] = sayn_communications_control.servo_set_position_method;

/*	test[0] = 0x32;
	test[1] = 0x01;
	test[2] = 0xFF;
	FillSlaveSendStructByArray(p_communication_str,&test[0],&p_parameter[0]);
	uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
*/


	if(sayn_communications_control.servo_set_position_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_position_num/5); count++)
		{
			In16(temp_16bit_parameter[count*2], p_parameter[count*5+2], p_parameter[count*5+1]);
			In16(temp_16bit_parameter[count*2+1], p_parameter[count*5+4], p_parameter[count*5+3]);
			p_parameter[151] = (servo_control.pSetPosition)(p_parameter[count*5], temp_16bit_parameter[count*2], temp_16bit_parameter[count*2+1]);
		}

//		if(TRUE == p_parameter[151])											// ��ȡ�ɹ�
//		{
/*			FillSlaveSendStructByArray(p_communication_str,&functional[0],&p_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
		}*/return ERR_OK;
	}
	else
	{
		return ERR_VALUE;						// �������Ȳ���ȷ
	}
}
uint8 SetServoVelocityMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{

	uint16 temp_16bit_parameter[60];  //�洢λ��
	uint8 functional[3];

	uint8 count = 0;

	functional[0] = 0;
	functional[1] = sayn_communications_control.servo_set_velocity_functional;
	functional[2] = sayn_communications_control.servo_set_velocity_method;

	if(sayn_communications_control.servo_set_velocity_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_velocity_num/4); count++)
		{
			In16(temp_16bit_parameter[count], p_parameter[count*4+3], p_parameter[count*4+2]);
			p_parameter[151] = (servo_control.pSetVelocity)(p_parameter[count*4], p_parameter[count*4+1], temp_16bit_parameter[count]);

		}

		return ERR_OK;
	}
	else
	{
		return ERR_VALUE;						// �������Ȳ���ȷ
	}
}

uint8 SetServoModeMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{

	uint8 temp = 0;
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = 0x00;  //�������ݳ���
	functional[1] = sayn_communications_control.servo_set_mode_functional;
	functional[2] = sayn_communications_control.servo_set_mode_method;

	if(sayn_communications_control.servo_set_mode_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_mode_num/2); count++)
		{
			temp = (servo_control.pSetMode)(sayn_communications_control.servo_set_mode_parameter[count*2], sayn_communications_control.servo_set_mode_parameter[count*2+1]);
		}

		if(TRUE == temp)											// ��ȡ�ɹ�
		{
			FillSlaveSendStructByArray(p_communication_str,&functional[0],&sayn_communications_control.servo_set_mode_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
		}
	}
	else
	{
		return ERR_VALUE;						// �������Ȳ���ȷ
	}

}
uint8 SetServoTorqueMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
	uint8 temp = 0;
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = 0x00;  //�������ݳ���
	functional[1] = sayn_communications_control.servo_set_torque_functional;
	functional[2] = sayn_communications_control.servo_set_torque_method;

	if(sayn_communications_control.servo_set_torque_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_torque_num/2); count++)
		{
			temp = (servo_control.pSetTorqueEnable)(sayn_communications_control.servo_set_torque_parameter[count*2], sayn_communications_control.servo_set_torque_parameter[count*2+1]);
		}

		if(TRUE == temp)											// ��ȡ�ɹ�
		{
			FillSlaveSendStructByArray(p_communication_str,&functional[0],&sayn_communications_control.servo_set_torque_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// �����ֵ�޷�ʹ��
		}
	}
	else
	{
		return ERR_VALUE;						// �������Ȳ���ȷ
	}
}

/*************************************************************************************************************************
** ��������:			CommandParse
**
** ��������:			�����������, ���ڽ�����ָ���Ľӿں���, ������������д��Э��ṹ��, �ɹ�����TRUE, ʧ�ܷ���FALSE��
**                      
**					                 
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit;
**
** ������:			����
** ��������:			2009-03-22
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CommandParse(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	uint8 temp_state = FALSE;
	
	if(ERR_CRC == (p_communication_str->state & ERR_MASK))							// У������󷵻�
	{
		StateResponse(p_communication_str);
		return TRUE;
	}
	
	if(p_communication_str->local_id == p_communication_str->str_stack.str_slave_receive.device_id)
	{
		switch(p_communication_str->str_stack.str_slave_receive.functional_unit)
		{
			case SYSTEM_INFO:
			{
				break;
			}
			case SYSTEM_STATE:
			{
				break;
			}
			case MAIN_COMMUNICATION:
			{
				break;
			}
			case MINOR_COMMUNICATION:
			{
				break;
			}
			case GPIO_PORT:
			{
				temp_state = GpioActuator(p_communication_str);					// GPIO����ģ�鴦�����
				if(ERR_OK == temp_state)
				{
					return FALSE;														// ��GPIO����ģ�鴦�����������Ӧ��
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// д״̬�Ĵ������ô���״̬���غ���
					StateResponse(p_communication_str);
					return TRUE;														// ����UpRobot��ջ���ͺ���
				}
				break;
			}
			case ADC_SAMPLING:
			{
				temp_state = AdcSamplingActuator(p_communication_str);					// ADC����ģ�鴦�����
				if(ERR_OK == temp_state)
				{
					return FALSE;														// ��ADC����ģ�鴦�����������Ӧ��
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// д״̬�Ĵ������ô���״̬���غ���
					StateResponse(p_communication_str);
					return TRUE;														// ����UpRobot��ջ���ͺ���
				}
				break;
			}
			case SERVO_MOTOR:
			{
				temp_state = ServoActuator(p_communication_str);						// Servo����ģ�鴦�����
				if(ERR_OK == temp_state)
				{
					return FALSE;														// ����UpRobot��ջ���ͺ���
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// д״̬�Ĵ������ô���״̬���غ���
					StateResponse(p_communication_str);
					return TRUE;														// ����UpRobot��ջ���ͺ���
				}
				break;
			}
			case SIGNAL_CONTROLER:
			{
				temp_state = SignalControlActuator(p_communication_str);					// Servo����ģ�鴦�����
				if(ERR_OK == temp_state)
				{
					return TRUE;														// ����UpRobot��ջ���ͺ���
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// д״̬�Ĵ������ô���״̬���غ���
					StateResponse(p_communication_str);
					return TRUE;														// ����UpRobot��ջ���ͺ���
				}
				break;
			}
			default:
			{
				SetRegMask(p_communication_str->state, ERR_MASK, ERR_VALUE);			// ����ģ���Ŵ���
				StateResponse(p_communication_str);
				return TRUE;															// ����UpRobot��ջ���ͺ���
				break;
			}

		}
	}
	else if(UPROBOT_BROADCAST_ID == p_communication_str->str_stack.str_slave_receive.device_id)
	{
		;
	}
	else			// ��ַƥ��ʧ�ܣ����߼����������ڵ�ǰ����
	{
		;// ����MINOR_COMMUNICATION����ģʽ����͸�����䣬��͸������MINOR_COMMUNICATION���ص����ݰ���������ݰ�����ʧ�ܣ�
		// ���ɱ�������λ�����ش�����Ϣ��
	}
	
	return FALSE;
}


/*************************************************************************************************************************
														���ƽṹ������
*************************************************************************************************************************/
COMMAND_CONTROL_STRUCT command_control = \
{\
	.pCommandParse = CommandParse \
};


SAYN_CONTROL_STRUCT sayn_communications_control = \
{\
	.adc_single_sayn_time = 0 , \
	.adc_single_sayn_count = 0 , \
	.adc_single_sayn_functional = 0x07,\
	.adc_single_sayn_method = 0x60,\
	.adc_single_sayn_length = 0x03,\
	\
	.adc_multi_sayn_time = 0, \
	.adc_multi_sayn_count = 0, \
	.adc_multi_sayn_functional = 0x07, \
	.adc_multi_sayn_method = 0x61, \
	.adc_multi_sayn_length = 0, \
	\
	.adc_all_sayn_time = 0,\
	.adc_all_sayn_count = 0,\
	.adc_all_sayn_functional = 0x07, \
	.adc_all_sayn_method = 0x62, \
	.adc_all_sayn_length = 17, \
	\
	.io_sayn_time = 0,\
	.io_sayn_count = 0, \
	.io_sayn_functional = 0x06, \
	.io_sayn_method = 0x60, \
	.io_sayn_length = 2,\
	\
	.servo_get_position_time = 0, \
	.servo_get_position_count = 0, \
	.servo_get_position_num = 0, \
	.servo_get_position_functional = 0X08, \
	.servo_get_position_method = 0X60, \
	.servo_get_position_length = 0, \
	\
	.servo_set_position_num = 0,\
	.servo_set_position_functional = 0X08, \
	.servo_set_position_method = 0X71, \
	.servo_set_position_length = 0, \
	\
	.servo_set_mode_num = 0,\
	.servo_set_mode_functional = 0X08, \
	.servo_set_mode_method = 0X70, \
	.servo_set_mode_length = 0, \
	\
	.servo_set_torque_num = 0,\
	.servo_set_torque_functional = 0X08, \
	.servo_set_torque_method = 0X72, \
	.servo_set_torque_length = 0 \
};
/*************************************************************************************************************************
**														�ļ�����
*************************************************************************************************************************/
