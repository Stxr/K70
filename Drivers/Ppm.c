/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			Ppm.c
** ����޶�����:		2009-03-06
** ���汾:			1.0
** ����:				ʹ��16λTIMER�Ŀ���PWMģʽʵ��8·PPM����(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-07
** �汾:				1.0
** ����:				ʹ��16λTIMER�Ŀ���PWMģʽʵ��8·PPM����(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-08-25
** �汾:				1.1
** ����:				�������ģʽ�д��ڵĴ��󣬴���ԭ�����ڽǶ������ڵ��ģʽʱ��Ȼ������Ϊ0��
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
*************************************************************************************************************************/
#include "Drivers/Ppm.h"


/*************************************************************************************************************************
                                                	�ڲ��ṹ�嶨��
*************************************************************************************************************************/
static PPM_STATE_STRUCT ppm_state[8];									// ����ppm��״̬�ṹ�����飬����ΪPPMͨ���ţ����ڿ���PPM״̬
static PPM_STATE_STRUCT ppm_buffer_state[8];							// ����ppm�Ļ���ṹ�����顣


/*************************************************************************************************************************
** ��������:			PPM_POSITION_CONVERSION
**
** ��������:			����ת���������ú������ڽ��û�����ķ�Χ��0-0x3FF��λ������ת��ΪPPM_MIN_VAL-PPM_MAX_VAL��ϵͳ����
**
**					                 
** �������:		    ta(output),val(input)
** ����ֵ:		    uint8
**
** ʹ�ú����:		None
** ʹ��ȫ�ֱ���:	    g_recv_spi0_state;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmPositionConversion(ta, val)	ta = PPM_MIN_VAL + (uint16)(((((uint32)(PPM_MAX_VAL - PPM_MIN_VAL) << 4) / 0x03FF) * (uint32)val) >> 4)


/*************************************************************************************************************************
** ��������:			PpmOutvalueConversion
**
** ��������:			����ת���������ú������ڽ�0-PPM_MAX_VAL��ϵͳ����ת��Ϊ�û�����ķ�Χ��0-0x3FF��λ������
**
**					                 
** �������:		    ta(output),val(input)
** ����ֵ:			uint8
**
** ʹ�ú����:		None
** ʹ��ȫ�ֱ���:	    g_recv_spi0_state;
**
** ���ú���:			None
**
** ������:			����
** ��������:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmOutvalueConversion(ta, val)	ta = (uint16)( ( ((uint32)0x03FF << 12) / (uint32)(PPM_MAX_VAL - PPM_MIN_VAL)) * (uint32)(val - PPM_MIN_VAL) >> 12)


/*************************************************************************************************************************
** ��������:			PPM_SPEED_CONVERSION
**
** ��������:		    ����ת���������ú������ڽ��û�����ķ�Χ��0-0x3FF�Ķ���ٶ�����ת��Ϊ0-PPM_MAX_SPEED��ϵͳ����
**
**					                 
** �������:		    ta(output),val(input)
** ����ֵ:		    	uint8
**
** ʹ�ú����:        None
** ʹ��ȫ�ֱ���:	    g_recv_spi0_state;
**
** ���ú���:			None
**
** ������:		    	����
** ��������:			2008-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
#define PpmSpeedConversion(ta, val)		ta = (PPM_MAX_SPEED / 0x03FF) * (uint16)val


/*************************************************************************************************************************
** ��������:			SetPpmPosition
**
** ��������:		    ����ppmͨ���ٶȡ�λ�ò�������������ִ�У�����Щ���ûᱻ���浽����ṹ�塣
**                      
**                      
**                      
**					                 
** �������:		    uint8 channel(ͨ�����), uint16 position, uint16 speed
** ����ֵ:		    	void
**
** ʹ�ú����:        None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:		    	����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			SetPpmAcceleration
**
** ��������:		    ����ppmͨ�����ٶȲ�������������ִ�У�����Щ���ûᱻ���浽����ṹ�塣
**                      
**                      
**                      
**					                 
** �������:		    uint8 channel(ͨ�����), uint16 position, uint16 speed
** ����ֵ:		    	void
**
** ʹ�ú����:        None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:		    	����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			SetPpmVelocity
**
** ��������:			����ppmͨ���ٶȡ�λ�ò�������������ִ�У�����Щ���ûᱻ���浽����ṹ�塣
**
**					                 
** �������:			uint8 channel(ͨ�����), uint16 position, uint16 speed
** ����ֵ:			void
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:			����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
				
				if(0 == direction)						// ˳ʱ��
				{
					PpmPositionConversion(ppm_buffer_state[count].target_position, (0x200 - (temp_velocity >> 1)));
				}
				else									// ��ʱ��
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
			
			if(0 == direction)						// ˳ʱ��
			{
				PpmPositionConversion(ppm_buffer_state[channel].target_position, (0x200 - (temp_velocity >> 1)));
			}
			else									// ��ʱ��
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
** ��������:			SetPpmLimit
**
** ��������:			����ppm��λ
**                      
**                                           
**					                 
** �������:		    uint8 channel(ͨ�����), uint16 cw, uint16 ccw,
** ����ֵ:			uint8,
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:			����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			����
** �޶�����:			2009-08-25
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
			PpmPositionConversion(ppm_buffer_state[channel].cw_limit, 0x0000);			// ȥ���Ƕ�����
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
** ��������:			GetPpmLimit
**
** ��������:			��ȡppm��λ
**                      
**                                           
**					                 
** �������:			uint8 channel(ͨ�����),  uint16* cw, uint16* ccw,
** ����ֵ:			void
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:			����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			GetPpmPosition
**
** ��������:		    ��ȡppmλ�õ�ǰֵ
**                      
**                                           
**					                 
** �������:		    uint8 channel(ͨ�����), uint16* position,
** ����ֵ:		    	void
**
** ʹ�ú����:        None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			PpmPositionConversion; PpmSpeedConversion;
**
** ������:		    	����
** ��������:			2009-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
** ��������:			RunPpm
**
** ��������:		    ppm����ִ�У��������е����ݵ���״̬������
**                      
**                      
**                      
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			None;
**
** ������:		    	����
** ��������:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
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
				
			ppm_buffer_state[count].enable_torque = TRUE;						// Ť�����ʹ��
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
		
		ppm_buffer_state[channel].enable_torque = TRUE;							// ʹ��Ť��
	}
	else
	{
		return FALSE;
	}
	return TRUE;
}


/*************************************************************************************************************************
** ��������:			StopPpm
**
** ��������:		    ppm����ִ�У��������е����ݵ���״̬������
**                      
**                      
**                      
**					                 
** �������:		    void
** ����ֵ:			void
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 StopPpm(uint8 channel)
{
	uint8 count = 0;
	
	if(PPM_BROADCASTING == channel)
	{
		for(count = 0; count < 8; count++)
		{
			ppm_buffer_state[count].enable_torque = FALSE;						// Ť�������ʹ��
		}
		return TRUE;
	}
	else if(channel <= 7)
	{
		ppm_buffer_state[channel].enable_torque = FALSE;						// ��ʹ��Ť��
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*************************************************************************************************************************
** ��������:			SIGNAL(PPM_COMPARE_INT)
**
** ��������:		    Timer�ıȽ��жϷ����������ڽ�ͨ������
**
**					                 
** �������:		    void
** ����ֵ:			void
**
** ʹ�ú����:		None;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None;
**
** ������:			����
** ��������:			2009-03-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			����
** �޶�����:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
SIGNAL(PPM_COMPARE_INT)
{
	#if PPM_PORT_ALONE
	
		switch(ppm_control.channel)								
		{
			case 0:																	// ����Ӧͨ���������0
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
	
		SetReg(PPM_PORT, 0x00);													// ��ȫ��ͨ���������0
		
	#endif
}
	

/*************************************************************************************************************************
** ��������:			IsrPpmOverFlow
**
** ��������:		    �ڶ�ʱ��2.5ms����ж��н���ͨ��ѡ�����ݼ��㡢���档
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        None
** ʹ��ȫ�ֱ���:	    None
**
** ���ú���:
**
** ������:		    	����
** ��������:			2008-03-19
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void IsrPpmOverFlow(void)
{
	int16 calc_difference;

	uint8 temp_channel = 0;
	
	temp_channel = ppm_control.channel;
	
	temp_channel++;																			// ���PPMͨ��
	
	if(temp_channel > 7)
	{
		temp_channel = 0;																	// ͨ����Ŵ���7���0
	}
	
	#if PPM_PORT_ALONE
		switch(ppm_control.channel)															// ����Ӧͨ���������1
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

		SetReg(PPM_PORT, 0x00);																// ����ȫ��ͨ�������ӿɿ���
		
		if(TRUE == ppm_buffer_state[temp_channel].enable_torque)
		{
			SetRegBit(PPM_PORT, ppm_control.channel_Mapping[temp_channel]);
		}
		
	#endif

	calc_difference = ppm_state[temp_channel].target_position
	- ppm_state[temp_channel].current_position;												// Ŀ��λ��ֵ�뵱ǰλ��ֵ���
	
	
	if(0 < calc_difference)																	// ���С��Ŀ��λ��
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
	else if(0 > calc_difference)															// �������Ŀ��λ��
	{
		calc_difference = 0 - calc_difference;												// ȡ��ֵ�ľ���ֵ
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
** ��������:			InitPpm
**
** ��������:			ppm������ݳ�ʼ�����κ�������Timer�ĳ�ʼ����
**                      
**                      
**					                 
** �������:			void
** ����ֵ:				void
**
** ʹ�ú����:		PPM_MIDDLE_VAL;
** ʹ��ȫ�ֱ���:		g_recv_spi0_state;
**
** ���ú���:			None
**
** ������:				����
** ��������:			2009-08-07
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitPpm(void)
{
	SetReg(ppm_control.state, 0);											// ��ʼ��״̬�Ĵ���
	
	if(!GetRegBit(PPM_TIMER_STATE, INIT_COMPLETE))
	{
		ClrRegBits(ppm_control.state, ERR_MASK);
		SetRegBits(ppm_control.state, ERR_DEPENDENCE);						// �ô������ͱ�־λ
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
		ppm_buffer_state[count].enable_torque = FALSE;						// Ť�������ʹ��
		ppm_buffer_state[count].mode = PPM_SERVO_MODE;
	}
	
	
	#if PPM_PORT_ALONE
		
	#else

		ppm_control.channel_Mapping[0] = 1;									// ��ʼ��ͨ��ӳ���
		ppm_control.channel_Mapping[1] = 0;
		ppm_control.channel_Mapping[2] = 3;
		ppm_control.channel_Mapping[3] = 2;
		ppm_control.channel_Mapping[4] = 5;
		ppm_control.channel_Mapping[5] = 4;
		ppm_control.channel_Mapping[6] = 7;
		ppm_control.channel_Mapping[7] = 6;

	#endif
	
	ppm_control.channel = 0;												// ͨ����ų�ʼ��
	
	#if PPM_PORT_ALONE														// �˿ڳ�ʼ��
	
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
	

	ppm_control.pSetPpmPosition = SetPpmPosition;							// λ�����ú���ָ��
	ppm_control.pRunPpm = RunPpm;											// λ�ñ��ִ�к���ָ��
	
	ppm_control.pSetPpmLimit = SetPpmLimit;
	ppm_control.pGetPpmLimit = GetPpmLimit;
	ppm_control.pGetPpmPosition = GetPpmPosition;
	ppm_control.pStopPpm = StopPpm;
	ppm_control.pSetPpmAcceleration = SetPpmAcceleration;
	ppm_control.pSetPpmVelocity = SetPpmVelocity;

	SetRegBit(ppm_control.state, INIT_COMPLETE);							// ��λ��ʼ����ɱ�־
}


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/
PPM_CONTROL_STRUCT ppm_control = { .pInit = InitPpm };


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
