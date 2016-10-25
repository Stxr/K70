/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Adc.c
** 最后修订日期:		2009-03-17
** 最后版本:			1.0
** 描述:		    	使用外部基准源的模拟数字装换单元(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-17
** 版本:				1.0
** 描述:		    	使用外部2.5V基准，共8个通道，不使用中断，10位精度(API)
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
#include "Drivers/Adc.h"
									

/*************************************************************************************************************************
** 函数名称:			SetAdcChannel(val)
**
** 函数描述:		    该函数用于AD转换通道设置。
**
**					                 
** 输入变量:		    void
** 返回值:			uint8
**
** 使用宏或常量:		None
** 使用全局变量:	    g_recv_spi0_state;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2008-08-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define SetAdcChannel(val)		SetReg(ADC_ADMUX, (ADC_REFERENCE | ((uint8)val & ADC_MUX_MASK)))							// 设置通道


/*************************************************************************************************************************
** 函数名称:			GetAdcRegValue
**
** 函数描述:		    获取Adc转换结果,成功则返回TURE,失败则返回FALSE
**                      
**					                 
** 输入变量:		    *item 变量指针，该变量用于保存转换结果
** 返回值:			uint8，是否获取成功
**
** 使用宏或常量:		Adc端口定义相关宏，在Adc.h中定义。
** 使用全局变量:	    None;
**
** 调用函数:			SetRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-08-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetAdcRegValue(uint16* item)
{
	if(GetRegBit(ADC_ADCSRA, ADIF))										// 查询转换完成中断标志
	{
		In16(*item, GetReg(ADC_ADC_L), GetReg(ADC_ADC_H));				// 读取转换结果
		SetRegBit(ADC_ADCSRA, ADIF);									// 清中断标志
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** 函数名称:			GetAdcValue
**
** 函数描述:		    获取Adc缓冲器中的值,成功则返回TURE,失败则返回FALSE
**                      
**					                 
** 输入变量:		    *item 变量指针，该变量用于保存转换结果
** 返回值:		    	uint8，是否获取成功
**
** 使用宏或常量:        Adc端口定义相关宏，在Adc.h中定义。
** 使用全局变量:	    None;
**
** 调用函数:			SetRegBit;
**
** 创建人:		    	律晔
** 创建日期:			2009-08-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetAdcValue(uint8 channel, uint16* item)
{
	if(channel > 7)
	{
		return FALSE;
	}
	
	if(ERR_OK == (adc_control.state & ERR_MASK))
	{
		*item = adc_control.adc_value_buffer[channel];
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** 函数名称:			StartAdc
**
** 函数描述:		    启动指定通道的Adc转换。
**                      
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        Adc端口定义相关宏，在Adc.h中定义。
** 使用全局变量:	    None;
**
** 调用函数:			SetRegBit;
**
** 创建人:		    	律晔
** 创建日期:			2009-08-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void StartAdc(uint8 channel)
{
	SetAdcChannel(adc_control.mapping[channel]);								// 选择通道
	SetRegBit(ADC_ADCSRA, ADSC);												// 启动转换
}


/*************************************************************************************************************************
** 函数名称:			IsrAdcSampling
**
** 函数描述:			在中断中处理Adc8通道采样。通道全部采集周期为20ms。
**                      
**					                 
** 输入变量:		    void
** 返回值:			void
**
** 使用宏或常量:		Adc端口定义相关宏，在Adc.h中定义。
** 使用全局变量:	    None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrAdcSampling(void)
{
	uint8 temp_sign = FALSE;

	uint8 temp_value = 0;
	
	temp_value = adc_control.channel;

	temp_sign = (adc_control.pGetRegValue)(&adc_control.adc_value_buffer[temp_value++]);

	if(FALSE == temp_sign)
	{
		ClrRegBits(adc_control.state , ERR_MASK);
		SetRegBits(adc_control.state , ERR_NOTAVAIL);
	}

	if(temp_value > 7)
	{
		temp_value = 0;
	}

	(adc_control.pStartConvert)(temp_value);							// 启动下一通道转换
	adc_control.channel = temp_value;
}



/*************************************************************************************************************************
** 函数名称:			InitAdc
**
** 函数描述:		    初始化Adc。
**                      
**					                 
** 输入变量:		    void
** 返回值:			void
**
** 使用宏或常量:		Adc端口定义相关宏，在Adc.h中定义。
** 使用全局变量:		None;
**
** 调用函数:			ClrRegBit;
**
** 创建人:			律晔
** 创建日期:			2009-08-17
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitAdc(void)
{
	SetReg(adc_control.state , 0x00);								// 初始化状态寄存器
		
	SetReg(ADC_DDR, 0x00);											// 初始化端口
	SetReg(ADC_PORT, 0x00);
	
	SetReg(ADC_ADMUX, ADC_REFERENCE);								// 设置电压基准源、通道默认为ADC0
	SetReg(ADC_ADCSRA, (ADC_EN | ADC_PRESCALER));					// 设置分频数、启动ADC单元
	SetRegBit(ADC_ADCSRA, ADSC);									// 启动一次转换
	
	adc_control.mapping[0] = 7;										// 初始化映射表
	adc_control.mapping[1] = 0;										// 初始化映射表
	adc_control.mapping[2] = 1;										// 初始化映射表
	adc_control.mapping[3] = 2;										// 初始化映射表
	adc_control.mapping[4] = 3;										// 初始化映射表
	adc_control.mapping[5] = 4;										// 初始化映射表
	adc_control.mapping[6] = 5;										// 初始化映射表
	adc_control.mapping[7] = 6;										// 初始化映射表
	
	adc_control.pStartConvert = StartAdc;
	adc_control.pGetRegValue = GetAdcRegValue;						// 初始化函数指针
	adc_control.pGetValue = GetAdcValue;
	
	adc_control.channel = 0;										// 初始化通道编号
	
	SetRegBit(adc_control.state, INIT_COMPLETE);					// 置位初始化完成标志
}


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
ADC_CONTROL_STRUCT adc_control = { .pInit = InitAdc };


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
