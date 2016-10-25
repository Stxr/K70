/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			Gpio.c
** ����޶�����:		2009-03-17
** ���汾:			1.0
** ����:				Gpio���ƺ����ӿڣ�����ȷ��GPIO��IoPort��ӳ���ϵ(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-17
** �汾:				1.0
** ����:				�Խӿڵķ��򼰵�ƽ������ֱ��ӳ��AVR����Ĺ��ܣ����ṩ��51��MCU�Ľӿڿ��Ʒ����ļ���(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-26
** �汾:				1.1
** ����:				�Բ������������˸��£��������Ч�ʡ�����ӳ���
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:			
** �޶�����:	    
** �汾:		    
** ����:            
**
*************************************************************************************************************************/
#include "Drivers/Gpio.h"


/*************************************************************************************************************************
** ��������:			SetGpioDdr
**
** ��������:			���ö���õ�12BitGpio�ķ���1Ϊ�����0Ϊ���롣�Ҷ��롣
**                      
**					                 
** �������:			uint16 word;
** ����ֵ:			void
**
** ʹ�ú����:		��غ궨��,��չGpio.h;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit; 
**
** ������:			����
** ��������:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			SetGpioPort
**
** ��������:			���ö���õ�12BitGpio�����ֵ���Ҷ��롣
**					��DDR����Ϊ����ʱ��1����GPIO����ʹ�ܣ�0��������Ϊ���衣
**					��DDR����Ϊ���ʱ��1��������ߵ�ƽ(����迹����2KR)��0��������͵�ƽ(�����С��20mA)��
**					                 
** �������:			uint16 word;
** ����ֵ:			void;
**
** ʹ�ú����:		��غ궨��,��չGpio.h;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit; 
**
** ������:			����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			SetOutputPort
**
** ��������:			���ö���õ�12BitGpio�����ֵ���Ҷ��롣
**					��DDR����Ϊ����ʱ����λ������Ч��
**					��DDR����Ϊ���ʱ��1��������ߵ�ƽ(����迹����2KR)��0��������͵�ƽ(�����С��20mA)��
**					                 
** �������:			uint16 word;
** ����ֵ:			void;
**
** ʹ�ú����:		��غ궨��,��չGpio.h;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit; 
**
** ������:			����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetOutputPort(uint16 word)
{
	uint16 temp_word = 0;
	
	temp_word = (word & gpio_control.direction_mask);
	
	SetGpioPort(temp_word);
}


/*************************************************************************************************************************
** ��������:			SetOutputBit
**
** ��������:			����1BitGpio�����ֵ��
**					��DDR����Ϊ����ʱ����λ������Ч��
**					��DDR����Ϊ���ʱ��1��������ߵ�ƽ(����迹����2KR)��0��������͵�ƽ(�����С��20mA)��
**					                 
** �������:			uint16 word;
** ����ֵ:			void;
**
** ʹ�ú����:		��غ궨��,��չGpio.h;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit; 
**
** ������:			����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			GetGpioPort
**
** ��������:		    ��ȡ����õ�12BitGpio����ֵ���Ҷ��롣
**                      ��DDR����Ϊ����ʱ��1����ߵ�ƽ��0����͵�ƽ��
**						��DDR����Ϊ���ʱ�����״̬������Ϊ����״̬���ɼ���
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        ��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit; 
**
** ������:		    	����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			GetInputPort
**
** ��������:		    ��ȡ����õ�12BitGpio����ֵ���Ҷ��롣
**                      ��DDR����Ϊ����ʱ��1����ߵ�ƽ��0����͵�ƽ��
**						��DDR����Ϊ���ʱ���ö˿ڲɼ�ֵ��Ϊ0��
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        ��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit; 
**
** ������:		    	����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void GetInputPort(uint16* val)
{
	uint16 temp_value = 0;

	GetGpioPort(&temp_value);
	
	*val = temp_value & ~gpio_control.direction_mask;
}


/*************************************************************************************************************************
** ��������:			GetInputBit
**
** ��������:		    ��ȡ1BitGpio����ֵ
**                      ��DDR����Ϊ����ʱ��1����ߵ�ƽ��0����͵�ƽ��
**						��DDR����Ϊ���ʱ���������
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        ��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit; 
**
** ������:		    	����
** ��������:			2009-03-26
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			IsrGpioSampling
**
** ��������:		    �ڶ�ʱ�ж��н������������ÿ2.5msһ�Ρ����峤��Ϊ8�����������˲����쳣�Ĳ�����
**                      
**						
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        ��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit; 
**
** ������:		    	����
** ��������:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
** ��������:			GetInputFilte
**
** ��������:		    ��ȡ�˲��������ֵ
**                      
**						
**					                 
** �������:		    void
** ����ֵ:		    	void
**
** ʹ�ú����:        ��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:	    None;
**
** ���ú���:			ClrRegBit; 
**
** ������:		    	����
** ��������:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
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
	};																								// ͨ��ֵ��������±����ͨ���ţ�ֵ�����ڱ������ڻ�ȡֵΪTRUE�Ĵ�����
	
	for(count = gpio_control.buffer_count; count < gpio_control.buffer_count + 8; count++)			// ����λ��
	{
		if(count > 7)																				// ���������λ�����
		{
			temp_buffer_count = count - 8;
		}
		else
		{
			temp_buffer_count = count;
		}
		
		for(temp_channel = 0; temp_channel < 12; temp_channel++)		// ͨ�����
		{
			if(GetRegBit(*(gpio_control.p_buffer_mapping[temp_channel] + temp_buffer_count), gpio_control.bit_mapping[temp_channel]))
			{
				temp_channel_value[temp_channel]++;
			}
		}
	}
	
	// �˲��㷨, 
	
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
** ��������:			GetInputFilteBit
**
** ��������:			��ȡ�˲��������ֵ
**                      
**						
**					                 
** �������:			void
** ����ֵ:			void
**
** ʹ�ú����:		��غ궨��,��չGpio.h;
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			ClrRegBit; 
**
** ������:			����
** ��������:			2009-03-29
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 GetInputFilteBit(uint8 channel, uint8* val)
{
	uint8 count = 0;
	uint8 result = 0;
	uint8 temp_buffer_count = 0;
	uint8 temp_channel_value =	0;
																									// ͨ��ֵ��������±����ͨ���ţ�ֵ�����ڱ������ڻ�ȡֵΪTRUE�Ĵ�����
	if(channel > 11)
	{
		return FALSE;
	}
	
	
	for(count = gpio_control.buffer_count; count < gpio_control.buffer_count + 8; count++)			// ����λ��
	{
		if(count > 7)																				// ���������λ�����
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
	
	// �˲��㷨, 
	

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
** ��������:			InitGpio
**
** ��������:			Gpio������ݳ�ʼ����
**					����Gpio���ֶ˿�
**                      
**					                 
** �������:			void
** ����ֵ:			void
**
** ʹ�ú����:		��غ궨��,��չGpio.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:			����
** ��������:			2009-03-17
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitGpio(void)
{
	SetReg(gpio_control.state , 0);									// ��ʼ��״̬�Ĵ���
	
	//��ʼ��gpio��ӳ���
	
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
	SetGpioPort(0x0000);											// ��ʼ�����������ƽ
	
	gpio_control.buffer_count = 0;									// ��ʼ��������λ�ü�����

	gpio_control.pSetGpioDdr = SetGpioDdr;							// ��ʼ������ָ��
	gpio_control.pSetGpioPort = SetGpioPort;
	gpio_control.pSetOutputPort = SetOutputPort;
	gpio_control.pSetOutputBit = SetOutputBit;
	gpio_control.pGetGpioPort = GetGpioPort;
	gpio_control.pGetInputPort = GetInputPort;
	gpio_control.pGetInputBit = GetInputBit;
	gpio_control.pGetInputFilteBit = GetInputFilteBit;
	gpio_control.pGetInputFilte = GetInputFilte;
	
	SetRegBit(gpio_control.state , INIT_COMPLETE);					// ��λ��ʼ����ɱ�־
}


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/
GPIO_CONTROL_STRUCT gpio_control = { .pInit = InitGpio };


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
