/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			CommandParser.c
** 最后修订日期:		2009-03-20
** 最后版本:			1.0
** 描述:				命令解析器，用于解析上位机的指令，调用相应的执行函数，返回应答数据。
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-20
** 版本:				1.0
** 描述:				解析上位机的指令，调用相应的执行函数，返回应答数据。
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
#include "ConfigTypes.h"

#include "Drivers/Adc.h"
#include "Drivers/Led.h"
#include "Drivers/Gpio.h"

#include "Shell/CommandParser.h"
#include "Shell/SystemUnit.h"
#include "Shell/ServoUnit.h"

/*************************************************************************************************************************
** 函数名称:			FillSlaveSendStruct
**
** 函数描述:			以结构体方式填充从机发送结构体。
**
**
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			徐俊辉
** 修订日期:			2010-03-09
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

	p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(p_communication_str->state, ERR_MASK, ERR_OK);	// 返回状态标志
}

/*************************************************************************************************************************
** 函数名称:			FillSlaveSendStruct
**
** 函数描述:			以数组方式填充从机发送结构体。
**
**
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2010-03-09
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			徐俊辉
** 修订日期:			2010-03-09
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

	p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(p_communication_str->state, ERR_MASK, ERR_OK);	// 返回状态标志
}



/*************************************************************************************************************************
** 函数名称:			StateResponse
**
** 函数描述:			 状态应答，本方法会将通讯中的错误类型通过State位返回上位机。在通讯错误和完成无返回值命令时使用;
**                      
**					                 
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void StateResponse(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
	p_communication_str->str_stack.str_slave_send.parameter_length = 0;							// 返回参数长度为0
	
	p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
	p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
	
	p_communication_str->str_stack.str_slave_send.state_byte = p_communication_str->state;		// 返回状态标志
}


/*************************************************************************************************************************
** 函数名称:			AdcSamplingActuator
**
** 函数描述:			Adc功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**                      
**					                 
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 AdcSamplingActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_ADC_VAL					0x00			// 0号方法
	#define GET_ADC_VAL_LEN				0x01			// 0号方法的参数长度
	
	#define GET_ADC_VAL_SINGLE_AT_TIMES		0x60
	#define GET_ADC_VAL_MULTI_AT_TIMES		0x61
	#define GET_ADC_VAL_ALL_AT_TIMES		0x62


//	uint8 count = 0;
	
	uint8 temp_8bit_parameter[8];						// 参数缓冲区
	uint16 temp_16bit_parameter[4];						// 参数缓冲区
	uint8 *p_parameter;
//	uint8 temp;
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET段
		*****************************************************************************************************************/
		case GET_ADC_VAL:
		{
			if(GET_ADC_VAL_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);
				temp_8bit_parameter[7] = adc_control.pGetValue(temp_8bit_parameter[0], &temp_16bit_parameter[0]);
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					Out16(temp_8bit_parameter[2], temp_8bit_parameter[1], temp_16bit_parameter[0])
					p_parameter = temp_8bit_parameter;
					FillSlaveSendStruct(p_communication_str,3,p_parameter);
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_RANGE;				// 参数超范围
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
			return ERR_VALUE;							// 请求的方法无法使用
		}
	}
}



/*************************************************************************************************************************
** 函数名称:			GetAdcSamplingAtTimes
**
** 函数描述:			定时获取单路ADC值，并从串口返回。
**
**
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcSamplingAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id)
{


	uint8 temp_8bit_parameter[8];						// 参数缓冲区
	uint16 temp_16bit_parameter[4];						// 参数缓冲区
	uint8 functional[3];
	uint8 temp_parameter_length = 3;

	functional[0]= temp_parameter_length; 								//数据位长度
	functional[1] = sayn_communications_control.adc_single_sayn_functional;
	functional[2] = sayn_communications_control.adc_single_sayn_method;
	temp_8bit_parameter[0] = (sayn_communications_control.adc_single_sayn_parameter[0]);
	temp_8bit_parameter[7] = adc_control.pGetValue(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

	if(TRUE == temp_8bit_parameter[7])											// 读取成功
	{
		Out16(temp_8bit_parameter[2], temp_8bit_parameter[1], temp_16bit_parameter[0]);
		FillSlaveSendStructByArray(p_communication_str,&functional[0],&temp_8bit_parameter[0]);
		uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);					return ERR_OK;
		return ERR_OK;
	}
	else
	{
		return ERR_NOTAVAIL;				// 请求的值无法使用
	}
}

/*************************************************************************************************************************
** 函数名称:			GetAdcMultiAtTimes
**
** 函数描述:			定时获取多路ADC值，并从串口返回。
**
**
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id)
{
	uint8 temp_8bit_parameter[18];
	uint16 temp_16bit_parameter[9];						// 参数缓冲区
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

		if(ERR_OK == temp)											// 读取成功
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
		return ERR_NOTAVAIL;				// 请求的值无法使用
	}
}


/*************************************************************************************************************************
** 函数名称:			GetAdcAllAtTimes
**
** 函数描述:			定时获取所有ADC值，并从串口返回。
**
**
** 输入变量			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetAdcAllAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	uint8 temp_8bit_parameter[10];
	uint16 temp_16bit_parameter[8];						// 参数缓冲区
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


	if(ERR_OK == temp)											// 读取成功
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
** 函数名称:			GpioActuator
**
** 函数描述:			Gpio功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**                      
**					                 
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GpioActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_INPUT_VAL					0x00						// 0号方法
	#define GET_INPUT_VAL_LEN						0x00				// 0号方法的参数长度
	#define GET_INPUT_VAL_BACK_LEN							0x02		// 0号方法的参数长度

	#define SET_DIRECTION_VAL				0x20
	#define SET_DIRECTION_VAL_LEN					0x02
		
	#define SET_OUTPUT_VAL					0x21					
	#define SET_OUTPUT_VAL_LEN						0x02
	
	#define GET_INPUT_VAL_AT_TIMES			0X60
	#define GET_INPUT_VAL_AT_TIMES_LEN			0X01
	#define GET_INPUT_VAL_AT_TIMES_BACK_LEN			0X02

	
	
	uint8 count = 0;
	
	uint8 temp_8bit_parameter[8];						// 参数缓冲区
	uint16 temp_16bit_parameter[4];						// 参数缓冲区
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET段
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
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;

			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
		}
		
		/*****************************************************************************************************************
															SET段
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
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
				
				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
				
				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
				
				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
				uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				return ERR_VALUE;						// 参数长度不正确
			}
		}

	
		/*****************************************************************************************************************/
		default:
		{
			return ERR_VALUE;							// 请求的方法无法使用
		}
	}
}
/*************************************************************************************************************************
** 函数名称:			GpioActuator
**
** 函数描述:			Gpio功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
uint8 GetIoInputAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str)
{

	uint8 temp_8bit_parameter[2];
	uint16 temp_16bit_parameter[2];						// 参数缓冲区
	uint8 * p_parameter;									//用于函数传递的一个临时指针变量
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
** 函数名称:			SignalControlActuator
**
** 函数描述:			SignalControl功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SignalControlActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_SIGNAL_BYTE					0x00						// 0号方法
	#define GET_SIGNAL_BYTE_LEN						0x01				// 0号方法的参数长度
	#define GET_SIGNAL_BYTE_BACK_LEN						0x02		// 0号方法的参数长度

	#define SET_SIGNAL_BYTE					0x20
	#define SET_SIGNAL_BYTE_LEN						0x02


	uint8 count = 0;

	uint8 temp_8bit_parameter[8];						// 参数缓冲区
//	uint16 temp_16bit_parameter[4];						// 参数缓冲区

	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
														GET段
		*****************************************************************************************************************/
		case GET_SIGNAL_BYTE:
		{
			if(GET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);

				temp_8bit_parameter[7] = (signal_control.pGetByte)(temp_8bit_parameter[0], &temp_8bit_parameter[1]);

				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SIGNAL_BYTE_BACK_LEN;


					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					p_communication_str->str_stack.str_slave_send.parameter[1] = temp_8bit_parameter[1];


					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志

					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
		}

		/*****************************************************************************************************************
															SET段
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
				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0

				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志

				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		default:
		{
			return ERR_VALUE;							// 请求的方法无法使用
			break;
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			SystemStateActuator
**
** 函数描述:			SystemState功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-11-04
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
//static uint8 SystemStateActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
//{
////	#define GET_SIGNAL_BYTE					0x00						// 0号方法
////	#define GET_SIGNAL_BYTE_LEN						0x01				// 0号方法的参数长度
////	#define GET_SIGNAL_BYTE_BACK_LEN						0x02		// 0号方法的参数长度
//
//	#define RESET_SYSTEM					0x40
//	#define RESET_SYSTEM_LEN						0x00
//
//
//	uint8 count = 0;
//
//	uint8 temp_8bit_parameter[8];						// 参数缓冲区
////	uint16 temp_16bit_parameter[4];						// 参数缓冲区
//
//	switch(p_communication_str->str_stack.str_slave_receive.method_code)
//	{
//		/*****************************************************************************************************************
//														GET段
//		*****************************************************************************************************************/
//		case GET_SIGNAL_BYTE:
//		{
//			if(GET_SIGNAL_BYTE_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
//			{
//				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);
//
//				temp_8bit_parameter[7] = (signal_control.pGetByte)(temp_8bit_parameter[0], &temp_8bit_parameter[1]);
//
//				if(TRUE == temp_8bit_parameter[7])											// 读取成功
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
//					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
//
//					return ERR_OK;
//				}
//				else
//				{
//					return ERR_NOTAVAIL;				// 请求的值无法使用
//				}
//			}
//			else
//			{
//				return ERR_VALUE;						// 参数长度不正确
//			}
//		}
//
//		/*****************************************************************************************************************
//															SET段
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
//				p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
//
//				p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
//				p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
//
//				p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
//
//				return ERR_OK;
//			}
//			else
//			{
//				return ERR_VALUE;						// 参数长度不正确
//			}
//			break;
//		}
//		default:
//		{
//			return ERR_VALUE;							// 请求的方法无法使用
//			break;
//		}
//	}
//}

/*************************************************************************************************************************
** 函数名称:			ServoActuator
**
** 函数描述:			Servo功能模块执行函数，用于按照方法编号调用相应的硬件操作接口。
**                      
**					                 
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-24
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ServoActuator(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	#define GET_SERVO_POSITION					0x00					// GET组 0号方法
	#define GET_SERVO_POSITION_LEN						0x01			// GET组 0号方法的参数长度
	#define GET_SERVO_POSITION_BACK_LEN							0x03	// GET组 0号方法的返回参数长度
	
	#define GET_SERVO_LIMIT						0x01
	#define GET_SERVO_LIMIT_LEN							0x01	
	#define GET_SERVO_LIMIT_BACK_LEN							0x05
	
	#define GET_SERVO_LOAD						0x02					// GET组 2号方法
	#define GET_SERVO_LOAD_LEN						0x01			// GET组 2号方法的参数长度
	#define GET_SERVO_LOAD_BACK_LEN							0x03	// GET组 2号方法的返回参数长度

	
	#define SET_SERVO_MODE						0x20					// SET组 0号方法
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
	uint8 temp_8bit_parameter[8];								// 参数缓冲区
	uint16 temp_16bit_parameter[4];								// 参数缓冲区
//	uint8 test[3];
	 
	switch(p_communication_str->str_stack.str_slave_receive.method_code)
	{
		/*****************************************************************************************************************
															GET段
		*****************************************************************************************************************/
		case GET_SERVO_POSITION:
		{
			if(GET_SERVO_POSITION_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);


				temp_8bit_parameter[7] = (servo_control.pGetPosition)(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

				if(TRUE == temp_8bit_parameter[7])																		// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_POSITION_BACK_LEN;		// 返回参数长度为3
					
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					
					Out16(temp_8bit_parameter[3], temp_8bit_parameter[2], temp_16bit_parameter[0])
					
					for(count = 0; count < GET_SERVO_POSITION_BACK_LEN - 1; count++)
					{
						p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
					}
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);


					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		
		case GET_SERVO_LIMIT:
		{
			if(GET_SERVO_LIMIT_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
			{
				temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);

				temp_8bit_parameter[7] = (servo_control.pGetAngleLimit)(temp_8bit_parameter[0], &temp_16bit_parameter[0], &temp_16bit_parameter[1]);

				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_LIMIT_BACK_LEN;		// 返回参数长度为5
					
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];
					
					Out16(temp_8bit_parameter[2], temp_8bit_parameter[3], temp_16bit_parameter[0]);
					Out16(temp_8bit_parameter[4], temp_8bit_parameter[5], temp_16bit_parameter[1]);
					
					for(count = 0; count < GET_SERVO_LIMIT_BACK_LEN - 1; count++)
					{
						p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
					}
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		

		case GET_SERVO_LOAD:
				{
					if(GET_SERVO_LOAD_LEN == p_communication_str->str_stack.str_slave_receive.parameter_length)
					{
						temp_8bit_parameter[0] = (p_communication_str->str_stack.str_slave_receive.parameter[0]);


						temp_8bit_parameter[7] = (servo_control.pGetLoad)(temp_8bit_parameter[0], &temp_16bit_parameter[0]);

						if(TRUE == temp_8bit_parameter[7])																		// 读取成功
						{
							p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
							p_communication_str->str_stack.str_slave_send.parameter_length = GET_SERVO_LOAD_BACK_LEN;		// 返回参数长度为3

							p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[0];

							Out16(temp_8bit_parameter[3], temp_8bit_parameter[2], temp_16bit_parameter[0])

							for(count = 0; count < GET_SERVO_LOAD_BACK_LEN - 1; count++)
							{
								p_communication_str->str_stack.str_slave_send.parameter[count + 1] = temp_8bit_parameter[count + 2];
							}

							p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
							p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

							p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
							uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
							return ERR_OK;
						}
						else
						{
							return ERR_NOTAVAIL;				// 请求的值无法使用
						}
					}
					else
					{
						return ERR_VALUE;						// 参数长度不正确
					}
					break;
				}


		/*****************************************************************************************************************
															SET段
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
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
					if(TRUE == temp_8bit_parameter[7])											// 读取成功
					{
						p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
						p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为0

						p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
						p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;

						p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
						uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
						return ERR_OK;
					}
					else
					{
						return ERR_NOTAVAIL;				// 请求的值无法使用
					}
				}
				else
				{
					return ERR_VALUE;						// 参数长度不正确
				}
				break;
			}
		
		/*****************************************************************************************************************
															FUNCTION段
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
				
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = 0;		// 返回参数长度为2
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
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
				
				if(TRUE == temp_8bit_parameter[7])											// 读取成功
				{
					p_communication_str->str_stack.str_slave_send.device_id = p_communication_str->local_id;
					p_communication_str->str_stack.str_slave_send.parameter_length = TEST_SERVO_BACK_LEN;		// 返回参数长度为1
					p_communication_str->str_stack.str_slave_send.parameter[0] = temp_8bit_parameter[6];
					
					p_communication_str->str_stack.str_slave_send.functional_unit = p_communication_str->str_stack.str_slave_receive.functional_unit;
					p_communication_str->str_stack.str_slave_send.method_code = p_communication_str->str_stack.str_slave_receive.method_code;
					
					p_communication_str->str_stack.str_slave_send.state_byte = SetRegMask(main_communication.state, ERR_MASK, ERR_OK);	// 返回状态标志
					uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
					return ERR_OK;
				}
				else
				{
					return ERR_NOTAVAIL;				// 请求的值无法使用
				}
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		
		case GET_SERVO_POSITION_MULTI_AT_TIMES:
		{

			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //有要查询的舵机号
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
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		
		case SET_SERVO_POSITION_MULTI_AT_TIMES:
		{


			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //有要查询的舵机号
			{
				sayn_communications_control.servo_set_position_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//数据长度

				for(count=0; count<sayn_communications_control.servo_set_position_num; count++)//将数据域存储下来
				{
					sayn_communications_control.servo_set_position_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_position_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		case SET_SERVO_MODE_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //有要查询的舵机号
			{
				sayn_communications_control.servo_set_mode_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//数据长度

				for(count=0; count<sayn_communications_control.servo_set_mode_num; count++)//将数据域存储下来
				{
					sayn_communications_control.servo_set_mode_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_mode_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}

			break;
		}
		case SET_SERVO_TORQUE_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length >0)  //有要设置的舵机号
			{
				sayn_communications_control.servo_set_torque_num = p_communication_str->str_stack.str_slave_receive.parameter_length;//数据长度

				for(count=0; count<sayn_communications_control.servo_set_torque_num; count++)//将数据域存储下来
				{
					sayn_communications_control.servo_set_torque_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoPositionMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_torque_parameter[0]);
				return ERR_OK;
			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}

			break;
		}
		/*****************************************************************************************************************/
		case SET_SERVO_VELOCITY_MULTI_AT_TIMES:
		{
			if(p_communication_str->str_stack.str_slave_receive.parameter_length > 0)
			{
				sayn_communications_control.servo_set_velocity_num = p_communication_str->str_stack.str_slave_receive.parameter_length;
				for(count=0; count<sayn_communications_control.servo_set_velocity_num; count++)//将数据域存储下来
				{
					sayn_communications_control.servo_set_velocity_parameter[count] = p_communication_str->str_stack.str_slave_receive.parameter[count];
				}
				SetServoVelocityMultiAtTimes(p_communication_str,&sayn_communications_control.servo_set_velocity_parameter[0]);
				return ERR_OK;

			}
			else
			{
				return ERR_VALUE;						// 参数长度不正确
			}
			break;
		}
		default:
		{
			return ERR_VALUE;							// 请求的方法无法使用
			break;
		}	
	}
}

uint8 GetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
	uint8 temp_8bit_parameter[61];		//存储位置
	uint16 temp_16bit_parameter[30];  //存储位置
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = sayn_communications_control.servo_get_position_num *3;
	functional[1] = sayn_communications_control.servo_get_position_functional;
	functional[2] = sayn_communications_control.servo_get_position_method;

	for(count=0; count<sayn_communications_control.servo_get_position_num; count++)
	{
		temp_8bit_parameter[60] &= (servo_control.pGetPosition)(sayn_communications_control.servo_get_position_parameter[count],&temp_16bit_parameter[count] );
	}
	if(TRUE == temp_8bit_parameter[60])																		// 读取成功
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
		return ERR_NOTAVAIL;				// 请求的值无法使用
	}
}
uint8 SetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
//	uint8 temp_8bit_parameter[151];		//存储位置
	uint16 temp_16bit_parameter[60];  //存储位置
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

//		if(TRUE == p_parameter[151])											// 读取成功
//		{
/*			FillSlaveSendStructByArray(p_communication_str,&functional[0],&p_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// 请求的值无法使用
		}*/return ERR_OK;
	}
	else
	{
		return ERR_VALUE;						// 参数长度不正确
	}
}
uint8 SetServoVelocityMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{

	uint16 temp_16bit_parameter[60];  //存储位置
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
		return ERR_VALUE;						// 参数长度不正确
	}
}

uint8 SetServoModeMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{

	uint8 temp = 0;
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = 0x00;  //返回数据长度
	functional[1] = sayn_communications_control.servo_set_mode_functional;
	functional[2] = sayn_communications_control.servo_set_mode_method;

	if(sayn_communications_control.servo_set_mode_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_mode_num/2); count++)
		{
			temp = (servo_control.pSetMode)(sayn_communications_control.servo_set_mode_parameter[count*2], sayn_communications_control.servo_set_mode_parameter[count*2+1]);
		}

		if(TRUE == temp)											// 读取成功
		{
			FillSlaveSendStructByArray(p_communication_str,&functional[0],&sayn_communications_control.servo_set_mode_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// 请求的值无法使用
		}
	}
	else
	{
		return ERR_VALUE;						// 参数长度不正确
	}

}
uint8 SetServoTorqueMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter)
{
	uint8 temp = 0;
	uint8 functional[3];
	uint8 count = 0;

	functional[0] = 0x00;  //返回数据长度
	functional[1] = sayn_communications_control.servo_set_torque_functional;
	functional[2] = sayn_communications_control.servo_set_torque_method;

	if(sayn_communications_control.servo_set_torque_num > 0)
	{
		for(count = 0; count < (sayn_communications_control.servo_set_torque_num/2); count++)
		{
			temp = (servo_control.pSetTorqueEnable)(sayn_communications_control.servo_set_torque_parameter[count*2], sayn_communications_control.servo_set_torque_parameter[count*2+1]);
		}

		if(TRUE == temp)											// 读取成功
		{
			FillSlaveSendStructByArray(p_communication_str,&functional[0],&sayn_communications_control.servo_set_torque_parameter[0]);
			uprobot_control.pUpRobotSlaveTxPacket(&p_communication_str->str_stack.str_slave_send, p_communication_str->p_uart_control);
			return ERR_OK;
		}
		else
		{
			return ERR_NOTAVAIL;				// 请求的值无法使用
		}
	}
	else
	{
		return ERR_VALUE;						// 参数长度不正确
	}
}

/*************************************************************************************************************************
** 函数名称:			CommandParse
**
** 函数描述:			命令解析函数, 用于解析出指定的接口函数, 并将返回数据写入协议结构体, 成功返回TRUE, 失败返回FALSE。
**                      
**					                 
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-03-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CommandParse(COMMUNICATION_PORT_STRUCT* p_communication_str)
{
	uint8 temp_state = FALSE;
	
	if(ERR_CRC == (p_communication_str->state & ERR_MASK))							// 校验码错误返回
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
				temp_state = GpioActuator(p_communication_str);					// GPIO功能模块处理程序
				if(ERR_OK == temp_state)
				{
					return FALSE;														// 由GPIO功能模块处理程序决定如何应答
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// 写状态寄存器调用错误状态返回函数
					StateResponse(p_communication_str);
					return TRUE;														// 调用UpRobot从栈发送函数
				}
				break;
			}
			case ADC_SAMPLING:
			{
				temp_state = AdcSamplingActuator(p_communication_str);					// ADC功能模块处理程序
				if(ERR_OK == temp_state)
				{
					return FALSE;														// 由ADC功能模块处理程序决定如何应答
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// 写状态寄存器调用错误状态返回函数
					StateResponse(p_communication_str);
					return TRUE;														// 调用UpRobot从栈发送函数
				}
				break;
			}
			case SERVO_MOTOR:
			{
				temp_state = ServoActuator(p_communication_str);						// Servo功能模块处理程序
				if(ERR_OK == temp_state)
				{
					return FALSE;														// 调用UpRobot从栈发送函数
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// 写状态寄存器调用错误状态返回函数
					StateResponse(p_communication_str);
					return TRUE;														// 调用UpRobot从栈发送函数
				}
				break;
			}
			case SIGNAL_CONTROLER:
			{
				temp_state = SignalControlActuator(p_communication_str);					// Servo功能模块处理程序
				if(ERR_OK == temp_state)
				{
					return TRUE;														// 调用UpRobot从栈发送函数
				}
				else
				{
					SetRegMask(p_communication_str->state, ERR_MASK, temp_state);		// 写状态寄存器调用错误状态返回函数
					StateResponse(p_communication_str);
					return TRUE;														// 调用UpRobot从栈发送函数
				}
				break;
			}
			default:
			{
				SetRegMask(p_communication_str->state, ERR_MASK, ERR_VALUE);			// 功能模块编号错误
				StateResponse(p_communication_str);
				return TRUE;															// 调用UpRobot从栈发送函数
				break;
			}

		}
	}
	else if(UPROBOT_BROADCAST_ID == p_communication_str->str_stack.str_slave_receive.device_id)
	{
		;
	}
	else			// 地址匹配失败，在逻辑控制器存在的前题下
	{
		;// 调用MINOR_COMMUNICATION主机模式进行透明传输，并透明传输MINOR_COMMUNICATION返回的数据包，如果数据包返回失败，
		// 则由本机向上位机返回错误信息。
	}
	
	return FALSE;
}


/*************************************************************************************************************************
														控制结构体声明
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
**														文件结束
*************************************************************************************************************************/
