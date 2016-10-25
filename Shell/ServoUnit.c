/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			ServoUnit.c
** ����޶�����:		2009-03-23
** ���汾:			1.0
** ����:				�������ִ���������ڽ���ִ�ж������ģ���ָ�ͳ��PPM������DynamixelProtocolЭ��ջ��UART�ӿ�
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-19
** �汾:				1.0
** ����:				ִ�еľ��幦�ܺ����Ĳ��������뷵��ֵ�ķ�װ��
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
#include "Shell/ServoUnit.h"


/*************************************************************************************************************************
** ��������:			TestServo
**
** ��������:			���Խڵ㣬���ڵ��Ƿ���ڣ�ϵͳ�϶�ģ�����ڵ����ڡ�
**					�ڵ�����򷵻�TRUE�������ڷ���FALSE��
**                      	                 
** �������:			uint8 id;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 TestServo(uint8 id, uint8* state)
{
	uint8 temp_value = 0;
	uint8 back_id = 0;
	uint8 temp_state = 0;
	
	temp_value = id >> 4;

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = FALSE;
	}
	
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoTorque
**
** ��������:			���ýڵ�Ť���������ʹ�ܡ���ʹ������״̬��
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 enable;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-06-02
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoTorqueEnable(uint8 id, uint8 enable)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))					// PPMID�Σ�����PPM����
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
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))				// BSU��ID�Σ�����BSU���߷���
	{

		if (SERVO_DIS == enable)
		{
			temp_state = ttlbus_control.pWriteByte(id, 0x18, 0x00);				// ���ýڵ�Ϊж��״̬
		}
		else
		{
			temp_state = ttlbus_control.pWriteByte(id, 0x18, 0x01);				// ���ýڵ�Ϊ�������״̬
		}

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** ��������:			CompareServoPosition
**
** ��������:			�Ƚϱ���λ����������һ��λ�������Ƿ���ͬ��
**					��ͬ����TRUE����ͬ����FALSE��
**                      	                 
** �������:			uint8 id, uint16 position, uint16 speed;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CompareServoPosition(uint8 id, uint16 position, uint16 velocity)
{
	uint8 temp_state = FALSE;
	uint8 temp_value = 0;
	uint8 count = 0;

	temp_value = (id >> 4);

	if (SERVO_PPM_ID == temp_value)						// PPMID�Σ�����PPM����
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
		timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

		if ((servo_control.last_position[temp_value] == position) && (servo_control.last_velocity[temp_value] == velocity))
		{
			temp_state = FALSE;
		}
		else
		{
			servo_control.last_position[temp_value] = position;
			servo_control.last_velocity[temp_value] = velocity;
		}

		timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ϵͳ�ж�
	}
	else
	{
		temp_state = FALSE;
	}

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			CompareServoVelocity
**
** ��������:			�Ƚϱ���λ����������һ���ٶ������Ƿ���ͬ��
**					��ͬ����TRUE����ͬ����FALSE��
**
** �������:			uint8 id, uint8 direction, uint16 velocity;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CompareServoVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_state = FALSE;
	uint8 temp_value = 0;
	uint8 count = 0;

	uint16 temp_velocity = 0;

	temp_value = (id >> 4);

	if (SERVO_PPM_ID == temp_value)						// PPMID�Σ�����PPM����
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
		timer0_control.pDisableInterrupt(TIMER8_IMR_OI);								// ��ֹϵͳ�ж�

		if (servo_control.last_velocity[temp_value] == temp_velocity)
		{
			temp_state = FALSE;
		}
		else
		{
			servo_control.last_velocity[temp_value] = temp_velocity;
		}

		timer0_control.pEnableInterrupt(TIMER8_IMR_OI);									// ϵͳ�ж�
	}
	else
	{
		temp_state = FALSE;
	}

	return temp_state;
}


/*************************************************************************************************************************
** ��������:			SetServoRegPosition
**
** ��������:			���ý��λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoRegPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))					// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoPosition
**
** ��������:			���ý��λ�ã�����ִ�С�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-11
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ

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
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))					// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoSyncPosition
**
** ��������:			���ý��λ�ã�����SyncAction��ͬ��ִ�С�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoSyncPosition(uint8 id, uint16 position, uint16 speed)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = CompareServoPosition(id, position, speed);

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)					// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmPosition)(id - 0xE0, position, speed);		// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ

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
	if(SERVO_PPM_ID != temp_value)													// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoRegVelocity
**
** ��������:			���ý���ٶȡ�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 direction, uint16 velocity
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoRegVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}
		
		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// ���ü��ٶ�Ϊ���

		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoRegVelocity
**
** ��������:			���ý���ٶȡ�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 direction, uint16 velocity
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = id;

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// ���ü��ٶ�Ϊ���

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		ppm_control.pRunPpm(id - 0xE0);
	}

	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoSyncVelocity
**
** ��������:			���ý���ٶȡ�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 direction, uint16 velocity
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoSyncVelocity(uint8 id, uint8 direction, uint16 velocity)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_state = CompareServoVelocity(id, direction, velocity);

	if (FALSE == temp_state)
	{
		return TRUE;																// �б�Ϊ�ظ����ͣ����Ա���ָ����ز����ɹ�
	}

	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmVelocity)(id - 0xE0, direction, velocity);		//

		if(FALSE == temp_state)
		{
			return FALSE;
		}

		temp_state = (ppm_control.pSetPpmAcceleration)(id - 0xE0, 0x3FF);				// ���ü��ٶ�Ϊ���

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

	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
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
** ��������:			SetServoLimit
**
** ��������:			���ý��λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoAngleLimit(uint8 id, uint16 cw, uint16 ccw)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, cw, ccw);					// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
		
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
	{
		temp_state = ttlbus_control.pWriteWord2(id, 0x06, cw, ccw);									// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
	
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			SetServoId
**
** ��������:			���ý��λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 SetServoId(uint8 id, uint8 new_id)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if((SERVO_PPM_ID == temp_value))										// PPMID�Σ�����PPM����
	{																		
		return FALSE;
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))			// BSU��ID�Σ�����BSU���߷���
	{
		temp_state = ttlbus_control.pWriteByte(id, 0x03, new_id);						// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
		
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			GetServoPosition
**
** ��������:			��ȡ���λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoPosition(uint8 id, uint16* position)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)														// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pGetPpmPosition)(id - 0xE0, position);				// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
		
		
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))						// BSU��ID�Σ�����BSU���߷���
	{

		temp_state = ttlbus_control.pReadWord(id, 0x24, position);									// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
		

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			GetServoLastPosition
**
** ��������:			��ȡ�����һ�θ���λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoLastPosition(uint8 id, uint16* last_position)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if ((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))			// PPMID�Σ�����PPM����
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
** ��������:			CalcServoVelocity
**
** ��������:
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint16 position, uint16 ret_time, uint16* calc_velocity, uint16* calc_time;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			GetServoLoad
**
** ��������:			��ȡ��㸺�ء�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoLoad(uint8 id, uint16* load)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;

	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		return FALSE;
	}
	if((SERVO_PPM_ID != temp_value) )													// BSU��ID�Σ�����BSU���߷���
	{

		temp_state = ttlbus_control.pReadWord(id, 0x28, load);										// ��ȡLoadֵ��

		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}

	return TRUE;
}


/*************************************************************************************************************************
** ��������:			GetServoLimit
**
** ��������:			��ȡ���λ�����ơ�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetServoAngleLimit(uint8 id, uint16* cw, uint16* ccw)
{
	uint8 temp_value = 0;
	uint8 temp_state = 0;
	
	temp_value = (id >> 4);

	if(SERVO_PPM_ID == temp_value)														// PPMID�Σ�����PPM����
	{
		temp_state = (ppm_control.pGetPpmLimit)(id - 0xE0, cw, ccw);					// ��ȡcw��ccw
	
		if(FALSE == temp_state)
		{
			return FALSE;
		}
	}
	
	if(SERVO_PPM_ID != temp_value)														// BSU��ID�Σ�����BSU���߷���
	{
		temp_state = ttlbus_control.pReadWord2(id, 0x06, cw, ccw);									// ��ȡ�ڵ�
		
		if(ERR_OK != temp_state)
		{
			return FALSE;
		}
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			ActionServo
**
** ��������:			���ý��λ�á�
**					���óɹ�����TRUE�����ɹ�����FALSE��
**                      	                 
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionServo(void)
{
	uint8 temp_state = 0;
	
	temp_state = (ppm_control.pRunPpm)(0x1E);					// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
		
	if(FALSE == temp_state)
	{
		return FALSE;
	}

	temp_state = ttlbus_control.pAction();				// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ

	if(ERR_OK != temp_state)
	{
		return FALSE;
	}
	
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			ActionSyncServo
**
** ��������:			�ڵ㶯��
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ActionSyncServo(void)
{
	ttlbus_control.pActionSync();
	ttlbus_control.sync_update_state = TRUE;				// TRUEʱ�������Control����

	return TRUE;
}


/*************************************************************************************************************************
** ��������:			SetServoMode
**
** ��������:			���ýڵ㹤��ģʽ���ж��ģʽ�����ģʽ��ж��ģʽ��
**					���óɹ�����TRUE�����ɹ�����FALSE��
**
** �������:			uint8 id, uint8 mode;
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-23
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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

	if((SERVO_PPM_ID == temp_value) || (BSU_BROADCASTING_ID == id))						// PPMID�Σ�����PPM����
	{
		switch(temp_mode)
		{
			case SERVO_MODE_SERVO:
			{
				temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, 0x0000, 0x03FF);		// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ

				break;
			}
			case SERVO_MODE_MOTO:
			{
				temp_state = (ppm_control.pSetPpmLimit)(id - 0xE0, 0x0000, 0x0000);		// ����λ������ͬʱΪ0��ת��Ϊ���ģʽ(�����ڿ��Ƶ��)

				if (FALSE != temp_state)
				{
					ppm_control.pSetPpmVelocity(id - 0xE0, 1, 0);
					_delay_ms(60);														// �ȴ�3���������
					ppm_control.pStopPpm(id - 0xE0);									// PPM���ֹͣ
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
	if((SERVO_PPM_ID != temp_value) || (BSU_BROADCASTING_ID == id))							// BSU��ID�Σ�����BSU���߷���
	{
		switch(temp_mode)
		{
			case SERVO_MODE_SERVO:
			{
				temp_state = ttlbus_control.pWriteWord2(id, 0x06, 0x0000, 0x03FF);			// ����λ�����ƴ򿪵����ת��Ϊ���ģʽ
				_delay_ms(20);

				break;
			}
			case SERVO_MODE_MOTO:
			{
				temp_state = ttlbus_control.pWriteWord2(id, 0x06, 0x0000, 0x0000);			// ����λ������ͬʱΪ0��ת��Ϊ���ģʽ(�����ڿ��Ƶ��)
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
** ��������:			InitTtlbusSyncMode
**
** ��������:			ͬ��д����ģʽ��ʼ��
**
**
** �������:			uint8* p_mapping, uint8 amount
** ����ֵ:			uint8;
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-10-13
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
													���ƽṹ������
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
**														�ļ�����
*************************************************************************************************************************/
