/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Ppm.c
** 最后修订日期:		2009-03-06
** 最后版本:			1.0
** 描述:				使用16位TIMER的快速PWM模式实现8路PPM控制(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-07
** 版本:				1.0
** 描述:				使用16位TIMER的快速PWM模式实现8路PPM控制(API)
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-08-25
** 版本:				1.1
** 描述:				修正电调模式中存在的错误，错误原因在于角度限制在电调模式时仍然被锁定为0；
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
*************************************************************************************************************************/
#include "Drivers/Ppm.h"


/*************************************************************************************************************************
                                                	内部结构体定意
*************************************************************************************************************************/
static PPM_STATE_STRUCT ppm_state[8];									// 定义ppm的状态结构体数组，其编号为PPM通道号，用于控制PPM状态
static PPM_STATE_STRUCT ppm_buffer_state[8];							// 定义ppm的缓冲结构体数组。


/*************************************************************************************************************************
** 函数名称:			PPM_POSITION_CONVERSION
**
** 函数描述:			量纲转换函数，该函数用于将用户输入的范围在0-0x3FF的位置数据转换为PPM_MIN_VAL-PPM_MAX_VAL的系统数据
**
**					                 
** 输入变量:		    ta(output),val(input)
** 返回值:		    uint8
**
** 使用宏或常量:		None
** 使用全局变量:	    g_recv_spi0_state;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmPositionConversion(ta, val)	ta = PPM_MIN_VAL + (uint16)(((((uint32)(PPM_MAX_VAL - PPM_MIN_VAL) << 4) / 0x03FF) * (uint32)val) >> 4)


/*************************************************************************************************************************
** 函数名称:			PpmOutvalueConversion
**
** 函数描述:			量纲转换函数，该函数用于将0-PPM_MAX_VAL的系统数据转换为用户输入的范围在0-0x3FF的位置数据
**
**					                 
** 输入变量:		    ta(output),val(input)
** 返回值:			uint8
**
** 使用宏或常量:		None
** 使用全局变量:	    g_recv_spi0_state;
**
** 调用函数:			None
**
** 创建人:			律晔
** 创建日期:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmOutvalueConversion(ta, val)	ta = (uint16)( ( ((uint32)0x03FF << 12) / (uint32)(PPM_MAX_VAL - PPM_MIN_VAL)) * (uint32)(val - PPM_MIN_VAL) >> 12)


/*************************************************************************************************************************
** 函数名称:			PPM_SPEED_CONVERSION
**
** 函数描述:		    量纲转换函数，该函数用于将用户输入的范围在0-0x3FF的舵机速度数据转换为0-PPM_MAX_SPEED的系统数据
**
**					                 
** 输入变量:		    ta(output),val(input)
** 返回值:		    	uint8
**
** 使用宏或常量:        None
** 使用全局变量:	    g_recv_spi0_state;
**
** 调用函数:			None
**
** 创建人:		    	律晔
** 创建日期:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmSpeedConversion(ta, val)		ta = (PPM_MAX_SPEED / 0x03FF) * (uint16)val


/*************************************************************************************************************************
** 函数名称:			SetPpmPosition
**
** 函数描述:		    设置ppm通道速度、位置参数（不会立即执行）。这些设置会被保存到缓冲结构体。
**                      
**                      
**                      
**					                 
** 输入变量:		    uint8 channel(通道编号), uint16 position, uint16 speed
** 返回值:		    	void
**
** 使用宏或常量:        None;
** 使用全局变量:	    None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:		    	律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetPpmPosition(uint8 channel, uint16 position, uint16 speed)
{
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			if(PPM_SERVO_MODE == ppm_buffer_state[count].mode)
			{
				if(position > 0x03FF)
				{
					position = 0x03FF;
				}
				if(speed > 0x03FF)
				{
					speed = 0x03FF;
				}
				
				PpmPositionConversion(ppm_buffer_state[count].target_position, position);
				PpmSpeedConversion(ppm_buffer_state[count].speed, speed);
			}
			else
			{
				return FALSE;
			}
		}
	}
	else if(channel <= 7)
	{			
		if(PPM_SERVO_MODE == ppm_buffer_state[channel].mode)
		{
			if(position > 0x03FF)
			{
				position = 0x03FF;
			}
			if(speed > 0x03FF)
			{
				speed = 0x03FF;
			}
			
			PpmPositionConversion(ppm_buffer_state[channel].target_position, position);
			PpmSpeedConversion(ppm_buffer_state[channel].speed, speed);
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
	return TRUE;	
}


/*************************************************************************************************************************
** 函数名称:			SetPpmAcceleration
**
** 函数描述:		    设置ppm通道加速度参数（不会立即执行）。这些设置会被保存到缓冲结构体。
**                      
**                      
**                      
**					                 
** 输入变量:		    uint8 channel(通道编号), uint16 position, uint16 speed
** 返回值:		    	void
**
** 使用宏或常量:        None;
** 使用全局变量:	    None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:		    	律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetPpmAcceleration(uint8 channel, uint16 acceleration)
{
	uint16 temp_acceleration = 0;
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			if(PPM_MOTO_MODE == ppm_buffer_state[count].mode)
			{
				if(acceleration > 0x03FF)
				{
					temp_acceleration = 0x03FF;
				}
				else
				{
					temp_acceleration = acceleration;
				}
				PpmSpeedConversion(ppm_buffer_state[count].speed, temp_acceleration);
			}
		}
	}
	else if(channel <= 7)
	{
		if(PPM_MOTO_MODE == ppm_buffer_state[channel].mode)
		{
			if(acceleration > 0x03FF)
			{
				temp_acceleration = 0x03FF;
			}
			else
			{
				temp_acceleration = acceleration;
			}
			
			PpmSpeedConversion(ppm_buffer_state[channel].speed, temp_acceleration);
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetPpmVelocity
**
** 函数描述:			设置ppm通道速度、位置参数（不会立即执行）。这些设置会被保存到缓冲结构体。
**
**					                 
** 输入变量:			uint8 channel(通道编号), uint16 position, uint16 speed
** 返回值:			void
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:			律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetPpmVelocity(uint8 channel, uint8 direction, uint16 velocity)
{
	uint16 temp_velocity = 0;
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			if(PPM_MOTO_MODE == ppm_buffer_state[count].mode)
			{
				if(velocity > 0x03FF)
				{
					temp_velocity = 0x03FF;
				}
				else
				{
					temp_velocity = velocity;
				}
				
				if(0 == direction)						// 顺时针
				{
					PpmPositionConversion(ppm_buffer_state[count].target_position, (0x200 - (temp_velocity >> 1)));
				}
				else									// 逆时针
				{
					PpmPositionConversion(ppm_buffer_state[count].target_position, (0x200 + (temp_velocity >> 1)));
				}
			}
			else
			{
				return FALSE;
			}
		}
	}
	else if(channel <= 7)
	{
		if(PPM_MOTO_MODE == ppm_buffer_state[channel].mode)
		{
			if(velocity > 0x03FF)
			{
				temp_velocity = 0x03FF;
			}
			else
			{
				temp_velocity = velocity;
			}
			
			if(0 == direction)						// 顺时针
			{
				PpmPositionConversion(ppm_buffer_state[channel].target_position, (0x200 - (temp_velocity >> 1)));
			}
			else									// 逆时针
			{
				PpmPositionConversion(ppm_buffer_state[channel].target_position, (0x200 + (temp_velocity >> 1)));
			}
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetPpmLimit
**
** 函数描述:			设置ppm限位
**                      
**                                           
**					                 
** 输入变量:		    uint8 channel(通道编号), uint16 cw, uint16 ccw,
** 返回值:			uint8,
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:			律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			律晔
** 修订日期:			2009-08-25
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetPpmLimit(uint8 channel, uint16 cw, uint16 ccw)
{
	uint16 temp_cw = 0;
	uint16 temp_ccw = 0;
	
	uint8 count = 0;
	
	if(cw > 0x03FF)
	{
		temp_cw = 0x03FF;
	}
	else
	{
		temp_cw = cw;
	}
	
	if(ccw > 0x03FF)
	{
		temp_ccw = 0x03FF;
	}
	else
	{
		temp_ccw = ccw;
	}
	
	if(temp_cw > temp_ccw)
	{
		return FALSE;
	}
	
	
	if (PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			if((0 == temp_cw) && (0 == temp_ccw))
			{
				ppm_buffer_state[count].mode = PPM_MOTO_MODE;

				PpmPositionConversion(ppm_buffer_state[count].cw_limit, 0x0000);
				PpmPositionConversion(ppm_buffer_state[count].ccw_limit, 0x03FF);
			}
			else
			{
				ppm_buffer_state[count].mode = PPM_SERVO_MODE;

				PpmPositionConversion(ppm_buffer_state[count].cw_limit, temp_cw);
				PpmPositionConversion(ppm_buffer_state[count].ccw_limit, temp_ccw);
			}
		}
		return TRUE;
	}
	else if(channel <= 7)
	{
		if((0 == temp_cw) && (0 == temp_ccw))
		{
			ppm_buffer_state[channel].mode = PPM_MOTO_MODE;
			PpmPositionConversion(ppm_buffer_state[channel].cw_limit, 0x0000);			// 去掉角度限制
			PpmPositionConversion(ppm_buffer_state[channel].ccw_limit, 0x03FF);
		}
		else
		{
			ppm_buffer_state[channel].mode = PPM_SERVO_MODE;

			PpmPositionConversion(ppm_buffer_state[channel].cw_limit, temp_cw);
			PpmPositionConversion(ppm_buffer_state[channel].ccw_limit, temp_ccw);
		}
		
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}


/*************************************************************************************************************************
** 函数名称:			GetPpmLimit
**
** 函数描述:			获取ppm限位
**                      
**                                           
**					                 
** 输入变量:			uint8 channel(通道编号),  uint16* cw, uint16* ccw,
** 返回值:			void
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:			律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetPpmLimit(uint8 channel, uint16* cw, uint16* ccw)
{
	if(channel < 8)
	{			
		if (PPM_MOTO_MODE == ppm_buffer_state[channel].mode)
		{
			*ccw = 0;
			*cw = 0;
			return TRUE;
		}
		PpmOutvalueConversion(*ccw, ppm_buffer_state[channel].ccw_limit);
		PpmOutvalueConversion(*cw, ppm_buffer_state[channel].cw_limit);
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}


/*************************************************************************************************************************
** 函数名称:			GetPpmPosition
**
** 函数描述:		    获取ppm位置当前值
**                      
**                                           
**					                 
** 输入变量:		    uint8 channel(通道编号), uint16* position,
** 返回值:		    	void
**
** 使用宏或常量:        None;
** 使用全局变量:	    None;
**
** 调用函数:			PpmPositionConversion; PpmSpeedConversion;
**
** 创建人:		    	律晔
** 创建日期:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetPpmPosition(uint8 channel, uint16* position)
{
	uint16 temp_value;

	if(channel <= 7)
	{				
		PpmOutvalueConversion(temp_value, ppm_buffer_state[channel].current_position);
		
		*position = temp_value;

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** 函数名称:			RunPpm
**
** 函数描述:		    ppm设置执行，将缓冲中的数据导入状态变量。
**                      
**                      
**                      
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        None;
** 使用全局变量:	    None;
**
** 调用函数:			None;
**
** 创建人:		    	律晔
** 创建日期:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 RunPpm(uint8 channel)
{
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			if(ppm_buffer_state[count].target_position > ppm_buffer_state[count].ccw_limit)
			{
				ppm_state[count].target_position = ppm_buffer_state[count].ccw_limit;
			}
			
			if(ppm_buffer_state[count].target_position < ppm_buffer_state[count].cw_limit)
			{
				ppm_state[count].target_position = ppm_buffer_state[count].cw_limit;
			}
			
			ppm_state[count].speed = ppm_buffer_state[count].speed;
			ppm_state[count].target_position = ppm_buffer_state[count].target_position;
				
			ppm_buffer_state[count].enable_torque = TRUE;						// 扭矩输出使能
		}
	}
	else if(channel <= 7)
	{
		if(ppm_buffer_state[channel].target_position > ppm_buffer_state[channel].ccw_limit)
		{
			ppm_state[channel].target_position = ppm_buffer_state[channel].ccw_limit;
		}
		
		if(ppm_buffer_state[channel].target_position < ppm_buffer_state[channel].cw_limit)
		{
			ppm_state[channel].target_position = ppm_buffer_state[channel].cw_limit;
		}
		
		ppm_state[channel].speed = ppm_buffer_state[channel].speed;
		ppm_state[channel].target_position = ppm_buffer_state[channel].target_position;
		
		ppm_buffer_state[channel].enable_torque = TRUE;							// 使能扭矩
	}
	else
	{
		return FALSE;
	}
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			StopPpm
**
** 函数描述:		    ppm设置执行，将缓冲中的数据导入状态变量。
**                      
**                      
**                      
**					                 
** 输入变量:		    void
** 返回值:			void
**
** 使用宏或常量:		None;
** 使用全局变量:	    None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 StopPpm(uint8 channel)
{
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			ppm_buffer_state[count].enable_torque = FALSE;						// 扭矩输出不使能
		}
		return TRUE;
	}
	else if(channel <= 7)
	{
		ppm_buffer_state[channel].enable_torque = FALSE;						// 不使能扭矩
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** 函数名称:			SIGNAL(PPM_COMPARE_INT)
**
** 函数描述:		    Timer的比较中断服务函数，用于将通道拉低
**
**					                 
** 输入变量:		    void
** 返回值:			void
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			律晔
** 修订日期:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
SIGNAL(PPM_COMPARE_INT)
{
	#if PPM_PORT_ALONE
	
		switch(ppm_control.channel)								
		{
			case 0:																	// 将对应通道的输出置0
				ClrRegBit(PPM0_PORT, PPM0_PORT_BIT);
				break;
			case 1:
				ClrRegBit(PPM1_PORT, PPM1_PORT_BIT);
				break;
			case 2:
				ClrRegBit(PPM2_PORT, PPM2_PORT_BIT);
				break;
			case 3:
				ClrRegBit(PPM3_PORT, PPM3_PORT_BIT);
				break;
			case 4:
				ClrRegBit(PPM4_PORT, PPM4_PORT_BIT);
				break;
			case 5:
				ClrRegBit(PPM5_PORT, PPM5_PORT_BIT);
				break;
			case 6:
				ClrRegBit(PPM6_PORT, PPM6_PORT_BIT);
				break;
			case 7:
				ClrRegBit(PPM7_PORT, PPM7_PORT_BIT);
				break;
			default:
				break;
		}
		
	#else
	
		SetReg(PPM_PORT, 0x00);													// 将全部通道的输出置0
		
	#endif
}
	

/*************************************************************************************************************************
** 函数名称:			IsrPpmOverFlow
**
** 函数描述:		    在定时器2.5ms溢出中断中进行通道选择，数据计算、更替。
**					                 
** 输入变量:		    void
** 返回值:		    	void
**
** 使用宏或常量:        None
** 使用全局变量:	    None
**
** 调用函数:
**
** 创建人:		    	律晔
** 创建日期:			2008-03-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrPpmOverFlow(void)
{
	int16 calc_difference;

	uint8 temp_channel = 0;
	
	temp_channel = ppm_control.channel;
	
	temp_channel++;																			// 变更PPM通道
	
	if(temp_channel > 7)
	{
		temp_channel = 0;																	// 通道编号大于7则归0
	}
	
	#if PPM_PORT_ALONE
		switch(ppm_control.channel)															// 将对应通道的输出置1
		{
			case 0:
				SetRegBit(PPM0_PORT, PPM0_PORT_BIT);
				break;
			case 1:
				SetRegBit(PPM1_PORT, PPM1_PORT_BIT);
				break;
			case 2:
				SetRegBit(PPM2_PORT, PPM2_PORT_BIT);
				break;
			case 3:
				SetRegBit(PPM3_PORT, PPM3_PORT_BIT);
				break;
			case 4:
				SetRegBit(PPM4_PORT, PPM4_PORT_BIT);
				break;
			case 5:
				SetRegBit(PPM5_PORT, PPM5_PORT_BIT);
				break;
			case 6:
				SetRegBit(PPM6_PORT, PPM6_PORT_BIT);
				break;
			case 7:
				SetRegBit(PPM7_PORT, PPM7_PORT_BIT);
				break;
			default:
				break;
		}
	#else

		SetReg(PPM_PORT, 0x00);																// 拉低全部通道，增加可靠性
		
		if(TRUE == ppm_buffer_state[temp_channel].enable_torque)
		{
			SetRegBit(PPM_PORT, ppm_control.channel_Mapping[temp_channel]);
		}
		
	#endif

	calc_difference = ppm_state[temp_channel].target_position
	- ppm_state[temp_channel].current_position;												// 目标位置值与当前位置值相减
	
	
	if(0 < calc_difference)																	// 如果小于目标位置
	{
		if(calc_difference <= ppm_state[temp_channel].speed)
		{
			ppm_state[temp_channel].current_position = ppm_state[temp_channel].target_position;
		}
		else
		{
			ppm_state[temp_channel].current_position += ppm_state[temp_channel].speed;
		}
	}
	else if(0 > calc_difference)															// 如果大于目标位置
	{
		calc_difference = 0 - calc_difference;												// 取差值的绝对值
		if(calc_difference <= ppm_state[temp_channel].speed)
		{
			ppm_state[temp_channel].current_position = ppm_state[temp_channel].target_position;
		}
		else
		{
			ppm_state[temp_channel].current_position -= ppm_state[temp_channel].speed;
		}
	}
	
	ppm_buffer_state[temp_channel].current_position = ppm_state[temp_channel].current_position;
	SetTimerPosition(ppm_state[temp_channel].current_position);

	ppm_control.channel = temp_channel;
}


/*************************************************************************************************************************
** 函数名称:			InitPpm
**
** 函数描述:			ppm相关数据初始化。次函数依赖Timer的初始化。
**                      
**                      
**					                 
** 输入变量:			void
** 返回值:				void
**
** 使用宏或常量:		PPM_MIDDLE_VAL;
** 使用全局变量:		g_recv_spi0_state;
**
** 调用函数:			None
**
** 创建人:				律晔
** 创建日期:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitPpm(void)
{
	SetReg(ppm_control.state, 0);											// 初始化状态寄存器
	
	if(!GetRegBit(PPM_TIMER_STATE, INIT_COMPLETE))
	{
		ClrRegBits(ppm_control.state, ERR_MASK);
		SetRegBits(ppm_control.state, ERR_DEPENDENCE);						// 置错误类型标志位
		return;
	}
	
	uint8 count = 0;
	
	for(count = 0; count < 8; count++)
	{
		ppm_state[count].speed = PPM_MAX_SPEED;
		ppm_state[count].target_position = PPM_MIDDLE_VAL;
		ppm_state[count].current_position = PPM_MIDDLE_VAL;
		
		ppm_buffer_state[count].cw_limit = PPM_MIN_VAL;
		ppm_buffer_state[count].ccw_limit = PPM_MAX_VAL;
		ppm_buffer_state[count].speed = PPM_MAX_SPEED;
		ppm_buffer_state[count].target_position = PPM_MIDDLE_VAL;
		ppm_buffer_state[count].current_position = PPM_MIDDLE_VAL;
		ppm_buffer_state[count].enable_torque = FALSE;						// 扭矩输出不使能
		ppm_buffer_state[count].mode = PPM_SERVO_MODE;
	}
	
	
	#if PPM_PORT_ALONE
		
	#else

		ppm_control.channel_Mapping[0] = 1;									// 初始化通道映射表
		ppm_control.channel_Mapping[1] = 0;
		ppm_control.channel_Mapping[2] = 3;
		ppm_control.channel_Mapping[3] = 2;
		ppm_control.channel_Mapping[4] = 5;
		ppm_control.channel_Mapping[5] = 4;
		ppm_control.channel_Mapping[6] = 7;
		ppm_control.channel_Mapping[7] = 6;

	#endif
	
	ppm_control.channel = 0;												// 通道编号初始化
	
	#if PPM_PORT_ALONE														// 端口初始化
	
		ClrRegBit(PPM0_PORT, PPM0_PORT_BIT);
		SetRegBit(PPM0_DDR,PPM0_DDR_BIT);
		ClrRegBit(PPM1_PORT, PPM1_PORT_BIT);
		SetRegBit(PPM1_DDR,PPM1_DDR_BIT);
		ClrRegBit(PPM2_PORT, PPM2_PORT_BIT);
		SetRegBit(PPM2_DDR,PPM2_DDR_BIT);
		ClrRegBit(PPM3_PORT, PPM3_PORT_BIT);
		SetRegBit(PPM3_DDR,PPM3_DDR_BIT);
		ClrRegBit(PPM4_PORT, PPM4_PORT_BIT);
		SetRegBit(PPM4_DDR,PPM4_DDR_BIT);
		ClrRegBit(PPM5_PORT, PPM5_PORT_BIT);
		SetRegBit(PPM5_DDR,PPM5_DDR_BIT);
		ClrRegBit(PPM6_PORT, PPM6_PORT_BIT);
		SetRegBit(PPM6_DDR,PPM6_DDR_BIT);
		ClrRegBit(PPM7_PORT, PPM7_PORT_BIT);
		SetRegBit(PPM7_DDR,PPM7_DDR_BIT);
		
	#else
	
		SetReg(PPM_DDR, 0xFF);
		SetReg(PPM_PORT, 0x00);
	
	#endif
	

	ppm_control.pSetPpmPosition = SetPpmPosition;							// 位置设置函数指针
	ppm_control.pRunPpm = RunPpm;											// 位置变更执行函数指针
	
	ppm_control.pSetPpmLimit = SetPpmLimit;
	ppm_control.pGetPpmLimit = GetPpmLimit;
	ppm_control.pGetPpmPosition = GetPpmPosition;
	ppm_control.pStopPpm = StopPpm;
	ppm_control.pSetPpmAcceleration = SetPpmAcceleration;
	ppm_control.pSetPpmVelocity = SetPpmVelocity;

	SetRegBit(ppm_control.state, INIT_COMPLETE);							// 置位初始化完成标志
}


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
PPM_CONTROL_STRUCT ppm_control = { .pInit = InitPpm };


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
