/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			ServoUnit.c
** 最后修订日期:		2009-03-23
** 最后版本:			1.0
** 描述:				舵机命令执行器，用于解析执行舵机功能模块的指令，统领PPM与利用DynamixelProtocol协议栈的UART接口
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-19
** 版本:				1.0
** 描述:				执行的具体功能函数的参数传递与返回值的封装。
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
#include "Shell/ServoUnit.h"


/*************************************************************************************************************************
** 函数名称:			TestServo
**
** 函数描述:			测试节点，检查节点是否存在，系统认定模拟舵机节点恒存在。
**					节点存在则返回TRUE；不存在返沪FALSE；
**                      	                 
** 输入变量:			uint8 id;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 TestServo(uint8 id, uint8* state)
{
	uint8 temp_value = 0;
	uint8 back_id = 0;
	uint8 temp_state = 0;
	
	temp_value = id >> 4;

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = FALSE;
	}
	
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{
		temp_value = ttlbus_control.pPing(id, &back_id);
		
		if(ERR_OK == temp_value)
		{
			if((back_id == id) || (BSU_BROADCASTING_ID == id))
			{
				temp_state = TRUE;
			}
			else
			{
				temp_state = FALSE;
			}
		}
		else
		{
			temp_state = FALSE;
		}
	}
	
	*state = ttlbus_control.state;
	
	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			SetServoTorque
**
** 函数描述:			设置节点扭矩输出，有使能、不使能两种状态。
**					设置成功返回TRUE；不成功返回FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 enable;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-06-02
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoTorqueEnable(uint8 id, uint8 enable)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))					// PPMID段，调用PPM方法
	{
		if (SERVO_DIS == enable)
		{
			(ppm_control.pStopPpm)(id - 0xE0);
		}
		else
		{
			(ppm_control.pRunPpm)(id - 0xE0);
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))				// BSU的ID段，调用BSU总线方法
	{

		if (SERVO_DIS == enable)
		{
			temp_state = ttlbus_control.pWriteByte(id, 0x18, 0x00);				// 设置节点为卸载状态
		}
		else
		{
			temp_state = ttlbus_control.pWriteByte(id, 0x18, 0x01);				// 设置节点为力矩输出状态
		}

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			CompareServoPosition
**
** 函数描述:			比较本次位置设置与上一次位置设置是否相同。
**					不同返回TRUE；相同返回FALSE；
**                      	                 
** 输入变量:			uint8 id, uint16 position, uint16 speed;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CompareServoPosition(uint8 id, uint16 position, uint16 velocity)
{
	uint8 temp_state = FALSE;
	uint8 temp_value = 0;
	uint8 count = 0;

	temp_value = (id >> 4);

	if (SERVO_PPM_ID == temp_value)						// PPMID段，调用PPM方法
	{
		return TRUE;
	}

	if (BSU_BROADCASTING_ID == id)
	{
		for (count = 0; count < ttlbus_control.device_amount; count++)
		{
			servo_control.last_position[count] = position;
			servo_control.last_velocity[count] = velocity;
		}
		return TRUE;
	}

	temp_state = mapping_control.pInverse1DimArray(ttlbus_control.p_device_mapping, ttlbus_control.device_amount, id, &temp_value);

	if (TRUE == temp_state)
	{
		timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

		if ((servo_control.last_position[temp_value] == position) && (servo_control.last_velocity[temp_value] == velocity))
		{
			temp_state = FALSE;
		}
		else
		{
			servo_control.last_position[temp_value] = position;
			servo_control.last_velocity[temp_value] = velocity;
		}

		timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 系统中断
	}
	else
	{
		temp_state = FALSE;
	}

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			CompareServoVelocity
**
** 函数描述:			比较本次位置设置与上一次速度设置是否相同。
**					不同返回TRUE；相同返回FALSE；
**
** 输入变量:			uint8 id, uint8 direction, uint16 velocity;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CompareServoVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_state = FALSE;
	uint8 temp_value = 0;
	uint8 count = 0;

	uint16 temp_velocity = 0;

	temp_value = (id >> 4);

	if (SERVO_PPM_ID == temp_value)						// PPMID段，调用PPM方法
	{
		return TRUE;
	}

	temp_velocity = (direction << 10) + velocity;

	if (BSU_BROADCASTING_ID == id)
	{
		for (count = 0; count < ttlbus_control.device_amount; count++)
		{
			servo_control.last_velocity[count] = temp_velocity;
		}
		return TRUE;
	}

	temp_state = mapping_control.pInverse1DimArray(ttlbus_control.p_device_mapping, ttlbus_control.device_amount, id, &temp_value);

	if (TRUE == temp_state)
	{
		timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// 禁止系统中断

		if (servo_control.last_velocity[temp_value] == temp_velocity)
		{
			temp_state = FALSE;
		}
		else
		{
			servo_control.last_velocity[temp_value] = temp_velocity;
		}

		timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// 系统中断
	}
	else
	{
		temp_state = FALSE;
	}

	return temp_state;
}


/*************************************************************************************************************************
** 函数名称:			SetServoRegPosition
**
** 函数描述:			设置结点位置。
**					设置成功返回TRUE；不成功返回FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoRegPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// 设置位置限制打开到最大，转换为舵机模式

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))					// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteRegWord2(id, 0x1E, position, speed);				//

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoPosition
**
** 函数描述:			设置结点位置，立即执行。
**					设置成功返回TRUE；不成功返回FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-11
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// 设置位置限制打开到最大，转换为舵机模式

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pRunPpm)(id - 0xE0);

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))					// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteWord2(id, 0x1E, position, speed);				//

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoSyncPosition
**
** 函数描述:			设置结点位置，发送SyncAction后同步执行。
**					设置成功返回TRUE；不成功返回FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoSyncPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = CompareServoPosition(id, position, speed);

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)					// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// 设置位置限制打开到最大，转换为舵机模式

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pRunPpm)(id - 0xE0);

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if(SERVO_PPM_ID != temp_value)													// BSU的ID段，调用BSU总线方法
	{
		if (TRUE == ttlbus_control.sync_update_state)
		{
			ttlbus_control.pWriteSyncControl(0x1E, 0x04);
			ttlbus_control.sync_update_state = FALSE;
		}

		temp_state = ttlbus_control.pWriteSyncWord2(id, position, speed);						//

		if(ERR_OK != temp_state)
		{
			return FALSE;															//
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoRegVelocity
**
** 函数描述:			设置结点速度。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 direction, uint16 velocity
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoRegVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}
		
		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// 设置加速度为最大

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteRegWord(id, 0x20, (direction << 10) + velocity);			//
			
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoRegVelocity
**
** 函数描述:			设置结点速度。
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 direction, uint16 velocity
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// 设置加速度为最大

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		ppm_control.pRunPpm(id - 0xE0);
	}

	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteWord(id, 0x20, (direction << 10) + velocity);			//

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoSyncVelocity
**
** 函数描述:			设置结点速度。
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 direction, uint16 velocity
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoSyncVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = CompareServoVelocity(id, direction, velocity);

	if (FALSE == temp_state)
	{
		return TRUE;																// 判别为重复发送，忽略本条指令，返回操作成功
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// 设置加速度为最大

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pRunPpm)(id - 0xE0);

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}

	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{
		if (TRUE == ttlbus_control.sync_update_state)
		{
			ttlbus_control.pWriteSyncControl(0x1E, 0x04);
			ttlbus_control.sync_update_state = FALSE;
		}

		temp_state = ttlbus_control.pWriteSyncWord2(id, 0x0200, (direction << 10) + velocity);			//

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoLimit
**
** 函数描述:			设置结点位置。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoAngleLimit(uint8 id, uint16 cw, uint16 ccw)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, cw, ccw);					// 设置位置限制打开到最大，转换为舵机模式
		
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteWord2(id, 0x06, cw, ccw);									// 设置位置限制打开到最大，转换为舵机模式
	
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoId
**
** 函数描述:			设置结点位置。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoId(uint8 id, uint8 new_id)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value))										// PPMID段，调用PPM方法
	{																		
		return FALSE;
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))			// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pWriteByte(id, 0x03, new_id);						// 设置位置限制打开到最大，转换为舵机模式
		
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			GetServoPosition
**
** 函数描述:			获取结点位置。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoPosition(uint8 id, uint16* position)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)														// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pGetPpmPosition)(id - 0xE0, position);				// 设置位置限制打开到最大，转换为舵机模式
		
		
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU的ID段，调用BSU总线方法
	{

		temp_state = ttlbus_control.pReadWord(id, 0x24, position);									// 设置位置限制打开到最大，转换为舵机模式
		

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			GetServoLastPosition
**
** 函数描述:			获取结点上一次给定位置。
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoLastPosition(uint8 id, uint16* last_position)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if ((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))			// PPMID段，调用PPM方法
	{
		return FALSE;
	}
	else
	{
		temp_state = mapping_control.pInverse1DimArray(ttlbus_control.p_device_mapping, ttlbus_control.device_amount, id, &temp_value);

		if (TRUE == temp_state)
		{
			*last_position = servo_control.last_position[temp_value];
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			CalcServoVelocity
**
** 函数描述:
**					设置成功返回TRUE；不成功返回FALSE；
**
** 输入变量:			uint8 id, uint16 position, uint16 ret_time, uint16* calc_velocity, uint16* calc_time;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CalcServoVelocity(uint8 id, uint16 position, uint16 ref_time, uint16* calc_velocity, uint16* calc_time)
{
	uint8 temp_state = 0;

	uint16 last_position = 0;
	uint16 temp_calc_velocity = 0;
	uint16 temp_calc_time = 0;

	uint32 temp_value = 0;

	temp_state = GetServoLastPosition(id, &last_position);

	if (TRUE == temp_state)
	{
		if (position > last_position)
		{
			temp_value = (uint32)(position - last_position);
		}
		else if (last_position > position)
		{
			temp_value = (uint32)(last_position - position);
		}
		else
		{
			*calc_time = 0;
			return TRUE;
		}

		temp_calc_velocity = (uint16)(((temp_value << 10) / (uint32)(ref_time)) - 1);

		if (temp_calc_velocity > 1023)
		{
			temp_calc_velocity = 1023;
			temp_calc_time = (uint16)(((temp_value << 10) / (((uint32)temp_calc_velocity) + 1)));
		}
		else
		{
			temp_calc_time = ref_time;
		}

		*calc_time = temp_calc_time;
		*calc_velocity = temp_calc_velocity;
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			GetServoLoad
**
** 函数描述:			获取结点负载。
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoLoad(uint8 id, uint16* load)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		return FALSE;
	}
	if((SERVO_PPM_ID != temp_value) )													// BSU的ID段，调用BSU总线方法
	{

		temp_state = ttlbus_control.pReadWord(id, 0x28, load);										// 读取Load值。

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			GetServoLimit
**
** 函数描述:			获取结点位置限制。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoAngleLimit(uint8 id, uint16* cw, uint16* ccw)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)														// PPMID段，调用PPM方法
	{
		temp_state = (ppm_control.pGetPpmLimit)(id - 0xE0, cw, ccw);					// 读取cw，ccw
	
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	
	if(SERVO_PPM_ID != temp_value)														// BSU的ID段，调用BSU总线方法
	{
		temp_state = ttlbus_control.pReadWord2(id, 0x06, cw, ccw);									// 读取节点
		
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			ActionServo
**
** 函数描述:			设置结点位置。
**					设置成功返回TRUE；不成功返沪FALSE；
**                      	                 
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionServo(void)
{
	uint8 temp_state = 0;
	
	temp_state = (ppm_control.pRunPpm)(0x1E);					// 设置位置限制打开到最大，转换为舵机模式
		
	if(FALSE == temp_state)
	{
		return FALSE;
	}

	temp_state = ttlbus_control.pAction();				// 设置位置限制打开到最大，转换为舵机模式

	if(ERR_OK != temp_state)
	{
		return FALSE;
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			ActionSyncServo
**
** 函数描述:			节点动作
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionSyncServo(void)
{
	ttlbus_control.pActionSync();
	ttlbus_control.sync_update_state = TRUE;				// TRUE时允许访问Control命令

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			SetServoMode
**
** 函数描述:			设置节点工作模式，有舵机模式、电机模式、卸载模式。
**					设置成功返回TRUE；不成功返沪FALSE；
**
** 输入变量:			uint8 id, uint8 mode;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoMode(uint8 id, uint8 mode)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	uint8 temp_mode = SERVO_MODE_SERVO;

	uint16 temp_cw_limit = 0;
	uint16 temp_ccw_limit = 0;

	if (id == BSU_BROADCASTING_ID)
	{
		temp_mode = mode;
	}
	else
	{
		temp_state = SetServoTorqueEnable(id, SERVO_DIS);

		if (FALSE != temp_state)
		{
			GetServoAngleLimit(id, &temp_cw_limit, &temp_ccw_limit);

			if ((0 == temp_cw_limit) && (0 == temp_ccw_limit))
			{
				if (SERVO_MODE_MOTO == mode)
				{
					return TRUE;
				}
				else
				{
					temp_mode = mode;
				}
			}
			else
			{
				if (SERVO_MODE_SERVO == mode)
				{
					return TRUE;
				}
				else
				{
					temp_mode = mode;
				}
			}
		}
		else
		{
			return FALSE;
		}
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID段，调用PPM方法
	{
		switch(temp_mode)
		{
			case SERVO_MODE_SERVO:
			{
				temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, 0x0000, 0x03FF);		// 设置位置限制打开到最大，转换为舵机模式

				break;
			}
			case SERVO_MODE_MOTO:
			{
				temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, 0x0000, 0x0000);		// 设置位置限制同时为0，转换为电机模式(可用于控制电调)

				if (FALSE != temp_state)
				{
					ppm_control.pSetPpmVelocity(id - 0xE0, 1, 0);
					_delay_ms(60);														// 等待3个舵机周期
					ppm_control.pStopPpm(id - 0xE0);									// PPM输出停止
				}

				break;
			}
			default:
			{
				break;
			}
		}

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))							// BSU的ID段，调用BSU总线方法
	{
		switch(temp_mode)
		{
			case SERVO_MODE_SERVO:
			{
				temp_state = ttlbus_control.pWriteWord2(id, 0x06, 0x0000, 0x03FF);			// 设置位置限制打开到最大，转换为舵机模式
				_delay_ms(20);

				break;
			}
			case SERVO_MODE_MOTO:
			{
				temp_state = ttlbus_control.pWriteWord2(id, 0x06, 0x0000, 0x0000);			// 设置位置限制同时为0，转换为电机模式(可用于控制电调)
				_delay_ms(20);

				break;
			}
			default:
			{
				break;
			}
		}

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** 函数名称:			InitTtlbusSyncMode
**
** 函数描述:			同步写控制模式初始化
**
**
** 输入变量:			uint8* p_mapping, uint8 amount
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitServoNodeBuffer(uint8* p_mapping, uint8 amount)
{
	uint8 count = 0;

	uint8 temp_state = 0;
	uint8 temp_value = 0;
	uint8 temp_amount = 0;

	if (amount > 32)
	{
		return;
	}

	for (count = 0; count < amount; count++)
	{
		temp_value = *(p_mapping + count);

		if ((SERVO_PPM_ID != (temp_value >> 4)) && (BSU_BROADCASTING_ID != temp_value) )
		{
			servo_control.node_mapping[temp_amount] = *(p_mapping + count);
			temp_amount++;
		}
	}

	ttlbus_control.pInitSync(&servo_control.node_mapping[0], temp_amount);

	for (count = 0; count < amount; count++)
	{
		temp_state = GetServoPosition(*(p_mapping + count), &servo_control.last_position[count]);

		if (FALSE == temp_state)
		{
			servo_control.last_position[count] = 0;
		}
		servo_control.last_velocity[count] = 0;
	}
}


/*************************************************************************************************************************
													控制结构体声明
*************************************************************************************************************************/
SERVO_CONTROL_STRUCT servo_control =
{
	.p_ppm_control = &ppm_control,

	.pTest = TestServo,
	.pSetMode = SetServoMode,

	.pSetPosition = SetServoPosition,
	.pSetRegPosition = SetServoRegPosition,
	.pSetSyncPosition = SetServoSyncPosition,

	.pSetVelocity = SetServoVelocity,
	.pSetRegVelocity = SetServoRegVelocity,
	.pSetSyncVelocity = SetServoSyncVelocity,

	.pSetAngleLimit = SetServoAngleLimit,

	.pSetId = SetServoId,

	.pGetPosition = GetServoPosition,

	.pGetLoad = GetServoLoad,

	.pGetAngleLimit = GetServoAngleLimit,

	.pAction = ActionServo,
	.pActionSync = ActionSyncServo,

	.pSetTorqueEnable = SetServoTorqueEnable,

	.pInitNodeBuffer = InitServoNodeBuffer,

	.pCalcVelocity = CalcServoVelocity,
};


/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
