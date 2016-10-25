/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Gpio.c
** 最后修订日期:		2009-03-17
** 最后版本:			1.0
** 描述:				Gpio控制函数接口，用于确定GPIO到IoPort的映射关系(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-17
** 版本:				1.0
** 描述:				对接口的方向及电平的设置直接映射AVR本身的功能，不提供对51等MCU的接口控制方法的兼容(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-26
** 版本:				1.1
** 描述:				对操作方法进行了更新，提高运行效率。引入映射表
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			
** 修订日期:	    
** 版本:		    
** 描述:            
**
*************************************************************************************************************************/
#include "Drivers/Gpio.h"


/*************************************************************************************************************************
** 函数名称:			SetGpioDdr
**
** 函数描述:			设置定义好的12BitGpio的方向。1为输出，0为输入。右对齐。
**                      
**					                 
** 输入变量:			uint16 word;
** 返回值:			void
**
** 使用宏或常量:		相关宏定义,参展Gpio.h;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:			律晔
** 创建日期:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetGpioDdr(uint16 val)
{
	uint8 count = 0;
	
	for(count = 0; count < 12; count++)
	{
		if(GetRegBit(val, count))
		{
			SetRegBit(*gpio_control.ddr_mapping[count], gpio_control.bit_mapping[count]);
			ClrRegBit(*gpio_control.port_mapping[count], gpio_control.bit_mapping[count]);
		}
		else
		{
			ClrRegBit(*gpio_control.ddr_mapping[count], gpio_control.bit_mapping[count]);
			SetRegBit(*gpio_control.port_mapping[count], gpio_control.bit_mapping[count]);
		}
	}
	
	gpio_control.direction_mask = val & 0x0FFF;
}


/*************************************************************************************************************************
** 函数名称:			SetGpioPort
**
** 函数描述:			设置定义好的12BitGpio的输出值。右对齐。
**					在DDR设置为输入时，1代表GPIO上拉使能，0代表输入为高阻。
**					在DDR设置为输出时，1代表输出高电平(输出阻抗大于2KR)，0代表输出低电平(灌电流小于20mA)。
**					                 
** 输入变量:			uint16 word;
** 返回值:			void;
**
** 使用宏或常量:		相关宏定义,参展Gpio.h;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:			律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetGpioPort(uint16 word)
{
	uint8 count = 0;
	
	for(count = 0; count < 12; count++)
	{
		if(GetRegBit(word, count))
		{
			SetRegBit(*gpio_control.port_mapping[count], gpio_control.bit_mapping[count]);
		}
		else
		{
			ClrRegBit(*gpio_control.port_mapping[count], gpio_control.bit_mapping[count]);
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			SetOutputPort
**
** 函数描述:			设置定义好的12BitGpio的输出值。右对齐。
**					在DDR设置为输入时，该位设置无效。
**					在DDR设置为输出时，1代表输出高电平(输出阻抗大于2KR)，0代表输出低电平(灌电流小于20mA)。
**					                 
** 输入变量:			uint16 word;
** 返回值:			void;
**
** 使用宏或常量:		相关宏定义,参展Gpio.h;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:			律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetOutputPort(uint16 word)
{
	uint16 temp_word = 0;
	
	temp_word = (word & gpio_control.direction_mask);
	
	SetGpioPort(temp_word);
}


/*************************************************************************************************************************
** 函数名称:			SetOutputBit
**
** 函数描述:			设置1BitGpio的输出值。
**					在DDR设置为输入时，该位设置无效。
**					在DDR设置为输出时，1代表输出高电平(输出阻抗大于2KR)，0代表输出低电平(灌电流小于20mA)。
**					                 
** 输入变量:			uint16 word;
** 返回值:			void;
**
** 使用宏或常量:		相关宏定义,参展Gpio.h;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:			律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetOutputBit(uint8 bit, uint8 val)
{

	if(bit >= 12)
	{
		return FALSE;
	}
	
	if(GetRegBit(gpio_control.direction_mask, bit))
	{
		if(val)
		{
			SetRegBit(*gpio_control.port_mapping[bit], gpio_control.bit_mapping[bit]);
		}
		else
		{
			ClrRegBit(*gpio_control.port_mapping[bit], gpio_control.bit_mapping[bit]);
		}
	}
	else
	{
		return FALSE;
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			GetGpioPort
**
** 函数描述:		    获取定义好的12BitGpio输入值，右对齐。
**                      在DDR设置为输入时，1输入高电平，0输入低电平。
**						在DDR设置为输出时，输出状态将被作为输入状态被采集。
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        相关宏定义,参展Gpio.h; 
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:		    	律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void GetGpioPort(uint16* val)
{
	uint16 temp_value = 0;
	uint8 count = 0;
	
	for(count = 0; count < 12; count++)
	{
		if(GetRegBit(*gpio_control.pin_mapping[count], gpio_control.bit_mapping[count]))
		{
			SetRegBit(temp_value, count);
		}
		else
		{
			ClrRegBit(temp_value, count);
		}
	}
	
	*val = temp_value;
}


/*************************************************************************************************************************
** 函数名称:			GetInputPort
**
** 函数描述:		    获取定义好的12BitGpio输入值，右对齐。
**                      在DDR设置为输入时，1输入高电平，0输入低电平。
**						在DDR设置为输出时，该端口采集值恒为0。
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        相关宏定义,参展Gpio.h; 
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:		    	律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void GetInputPort(uint16* val)
{
	uint16 temp_value = 0;

	GetGpioPort(&temp_value);
	
	*val = temp_value & ~gpio_control.direction_mask;
}


/*************************************************************************************************************************
** 函数名称:			GetInputBit
**
** 函数描述:		    获取1BitGpio输入值
**                      在DDR设置为输入时，1输入高电平，0输入低电平。
**						在DDR设置为输出时，报告错误。
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        相关宏定义,参展Gpio.h; 
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:		    	律晔
** 创建日期:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetInputBit(uint8 bit, uint8* val)
{
	uint8 temp_value = 0;

	if(bit >= 12)
	{
		return FALSE;
	}
	
	if(GetRegBit(gpio_control.direction_mask, bit))
	{
		return FALSE;
	}
	else
	{
		temp_value = GetRegBit(*gpio_control.pin_mapping[bit], gpio_control.bit_mapping[bit]);
		*val = temp_value >> gpio_control.bit_mapping[bit];
	}
	
	return TRUE;	
}


/*************************************************************************************************************************
** 函数名称:			IsrGpioSampling
**
** 函数描述:		    在定时中断中进行输入采样，每2.5ms一次。缓冲长度为8，用于输入滤波和异常的产生。
**                      
**						
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        相关宏定义,参展Gpio.h; 
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:		    	律晔
** 创建日期:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrGpioSampling(void)
{
	gpio_control.buffer_count++;
	
	if(gpio_control.buffer_count > 7)
	{
		gpio_control.buffer_count = 0;
	}
	
	gpio_control.group0_buffer[gpio_control.buffer_count] = GPIO_GROUP0_PIN;
	gpio_control.group1_buffer[gpio_control.buffer_count] = GPIO_GROUP1_PIN;
	gpio_control.group2_buffer[gpio_control.buffer_count] = GPIO_GROUP2_PIN;
}


/*************************************************************************************************************************
** 函数名称:			GetInputFilte
**
** 函数描述:		    获取滤波后的输入值
**                      
**						
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        相关宏定义,参展Gpio.h; 
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:		    	律晔
** 创建日期:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void GetInputFilte(uint16* val)
{
	uint8 count = 0;
	uint8 temp_channel = 0;
	uint16 result = 0;
	uint8 temp_buffer_count = 0;
	
	uint8 temp_channel_value[12] =				\
	{											\
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0		\
	};																								// 通道值，数组的下标代表通道号，值代表在本周期内获取值为TRUE的次数。
	
	for(count = gpio_control.buffer_count; count < gpio_control.buffer_count + 8; count++)			// 缓冲位置
	{
		if(count > 7)																				// 缓冲区环形化处理
		{
			temp_buffer_count = count - 8;
		}
		else
		{
			temp_buffer_count = count;
		}
		
		for(temp_channel = 0; temp_channel < 12; temp_channel++)		// 通道编号
		{
			if(GetRegBit(*(gpio_control.p_buffer_mapping[temp_channel] + temp_buffer_count), gpio_control.bit_mapping[temp_channel]))
			{
				temp_channel_value[temp_channel]++;
			}
		}
	}
	
	// 滤波算法, 
	
	for(count = 0; count < 12; count++)
	{
		if(temp_channel_value[count] > 4)
		{
			SetRegBit(result, count);
		}
	}
	
	*val = result;
}


/*************************************************************************************************************************
** 函数名称:			GetInputFilteBit
**
** 函数描述:			获取滤波后的输入值
**                      
**						
**					                 
** 输入变量:			void
** 返回值:			void
**
** 使用宏或常量:		相关宏定义,参展Gpio.h;
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit; 
**
** 创建人:			律晔
** 创建日期:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetInputFilteBit(uint8 channel, uint8* val)
{
	uint8 count = 0;
	uint8 result = 0;
	uint8 temp_buffer_count = 0;
	uint8 temp_channel_value =	0;
																									// 通道值，数组的下标代表通道号，值代表在本周期内获取值为TRUE的次数。
	if(channel > 11)
	{
		return FALSE;
	}
	
	
	for(count = gpio_control.buffer_count; count < gpio_control.buffer_count + 8; count++)			// 缓冲位置
	{
		if(count > 7)																				// 缓冲区环形化处理
		{
			temp_buffer_count = count - 8;
		}
		else
		{
			temp_buffer_count = count;
		}
		

		if(GetRegBit(*(gpio_control.p_buffer_mapping[channel] + temp_buffer_count), gpio_control.bit_mapping[channel]))
		{
			temp_channel_value++;
		}
	}
	
	// 滤波算法, 
	

	if(temp_channel_value > 4)
	{
		result = TRUE;
	}
	else
	{
		result = FALSE;
	}

	*val = result;
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			InitGpio
**
** 函数描述:			Gpio相关数据初始化。
**					包含Gpio部分端口
**                      
**					                 
** 输入变量:			void
** 返回值:			void
**
** 使用宏或常量:		相关宏定义,参展Gpio.h; 
** 使用全局变量:		None;
**
** 调用函数:			None 
**
** 创建人:			律晔
** 创建日期:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:              
** 修订日期:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitGpio(void)
{
	SetReg(gpio_control.state , 0);									// 初始化状态寄存器
	
	//初始化gpio的映射表
	
	gpio_control.port_mapping[0] = &GPIO0_PORT;
	gpio_control.port_mapping[1] = &GPIO1_PORT;
	gpio_control.port_mapping[2] = &GPIO2_PORT;
	gpio_control.port_mapping[3] = &GPIO3_PORT;
	
	gpio_control.port_mapping[4] = &GPIO4_PORT;
	gpio_control.port_mapping[5] = &GPIO5_PORT;
	gpio_control.port_mapping[6] = &GPIO6_PORT;
	gpio_control.port_mapping[7] = &GPIO7_PORT;
	
	gpio_control.port_mapping[8] = &GPIO8_PORT;
	gpio_control.port_mapping[9] = &GPIO9_PORT;
	gpio_control.port_mapping[10] = &GPIO10_PORT;
	gpio_control.port_mapping[11] = &GPIO11_PORT;
	
	gpio_control.pin_mapping[0] = &GPIO0_PIN;
	gpio_control.pin_mapping[1] = &GPIO1_PIN;
	gpio_control.pin_mapping[2] = &GPIO2_PIN;
	gpio_control.pin_mapping[3] = &GPIO3_PIN;
	
	gpio_control.pin_mapping[4] = &GPIO4_PIN;
	gpio_control.pin_mapping[5] = &GPIO5_PIN;
	gpio_control.pin_mapping[6] = &GPIO6_PIN;
	gpio_control.pin_mapping[7] = &GPIO7_PIN;
	
	gpio_control.pin_mapping[8] = &GPIO8_PIN;
	gpio_control.pin_mapping[9] = &GPIO9_PIN;
	gpio_control.pin_mapping[10] = &GPIO10_PIN;
	gpio_control.pin_mapping[11] = &GPIO11_PIN;
	
	
	gpio_control.ddr_mapping[0] = &GPIO0_DDR;
	gpio_control.ddr_mapping[1] = &GPIO1_DDR;
	gpio_control.ddr_mapping[2] = &GPIO2_DDR;
	gpio_control.ddr_mapping[3] = &GPIO3_DDR;
	
	gpio_control.ddr_mapping[4] = &GPIO4_DDR;
	gpio_control.ddr_mapping[5] = &GPIO5_DDR;
	gpio_control.ddr_mapping[6] = &GPIO6_DDR;
	gpio_control.ddr_mapping[7] = &GPIO7_DDR;
	
	gpio_control.ddr_mapping[8] = &GPIO8_DDR;
	gpio_control.ddr_mapping[9] = &GPIO9_DDR;
	gpio_control.ddr_mapping[10] = &GPIO10_DDR;
	gpio_control.ddr_mapping[11] = &GPIO11_DDR;
	
	
	gpio_control.bit_mapping[0] = GPIO0_BIT;
	gpio_control.bit_mapping[1] = GPIO1_BIT;
	gpio_control.bit_mapping[2] = GPIO2_BIT;
	gpio_control.bit_mapping[3] = GPIO3_BIT;
	
	gpio_control.bit_mapping[4] = GPIO4_BIT;
	gpio_control.bit_mapping[5] = GPIO5_BIT;
	gpio_control.bit_mapping[6] = GPIO6_BIT;
	gpio_control.bit_mapping[7] = GPIO7_BIT;
	
	gpio_control.bit_mapping[8] = GPIO8_BIT;
	gpio_control.bit_mapping[9] = GPIO9_BIT;
	gpio_control.bit_mapping[10] = GPIO10_BIT;
	gpio_control.bit_mapping[11] = GPIO11_BIT;
	
	gpio_control.p_buffer_mapping[0] = &gpio_control.group2_buffer[0];
	gpio_control.p_buffer_mapping[1] = &gpio_control.group2_buffer[0];
	gpio_control.p_buffer_mapping[2] = &gpio_control.group2_buffer[0];
	gpio_control.p_buffer_mapping[3] = &gpio_control.group2_buffer[0];
	
	gpio_control.p_buffer_mapping[4] = &gpio_control.group0_buffer[0];
	gpio_control.p_buffer_mapping[5] = &gpio_control.group0_buffer[0];
	gpio_control.p_buffer_mapping[6] = &gpio_control.group2_buffer[0];
	gpio_control.p_buffer_mapping[7] = &gpio_control.group2_buffer[0];
	
	gpio_control.p_buffer_mapping[8] = &gpio_control.group1_buffer[0];
	gpio_control.p_buffer_mapping[9] = &gpio_control.group1_buffer[0];
	gpio_control.p_buffer_mapping[10] = &gpio_control.group0_buffer[0];
	gpio_control.p_buffer_mapping[11] = &gpio_control.group1_buffer[0];
	
	SetGpioDdr(0x0000);
	SetGpioPort(0x0000);											// 初始化方向、输出电平
	
	gpio_control.buffer_count = 0;									// 初始化缓冲区位置计数器

	gpio_control.pSetGpioDdr = SetGpioDdr;							// 初始化函数指针
	gpio_control.pSetGpioPort = SetGpioPort;
	gpio_control.pSetOutputPort = SetOutputPort;
	gpio_control.pSetOutputBit = SetOutputBit;
	gpio_control.pGetGpioPort = GetGpioPort;
	gpio_control.pGetInputPort = GetInputPort;
	gpio_control.pGetInputBit = GetInputBit;
	gpio_control.pGetInputFilteBit = GetInputFilteBit;
	gpio_control.pGetInputFilte = GetInputFilte;
	
	SetRegBit(gpio_control.state , INIT_COMPLETE);					// 置位初始化完成标志
}


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
GPIO_CONTROL_STRUCT gpio_control = { .pInit = InitGpio };


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
