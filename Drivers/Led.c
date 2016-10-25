/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			Led.c
** ����޶�����:		2009-03-18
** ���汾:			1.0
** ����:				LED�Ĳ����Ľӿڶ���(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-03-18
** �汾:				1.0
** ����:				��ʼ��LED�Ȳ���
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
#include "Drivers/Led.h"


/*************************************************************************************************************************
** ��������:			SetLedBit
**
** ��������:			��LED���е������ơ�
**                      
**					    
** �������:			uint8 val;
** ����ֵ:			void
**
** ʹ�ú����:		��غ궨��,�μ�LED.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:			����
** ��������:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetLedBit(uint8 val)
{
	#if LED_PORT_ALONE	
	#else
		SetRegBit(LED_PORT, LED_BIT_BOTTOM + val);
	#endif
}
	

/*************************************************************************************************************************
** ��������:			ClrLedBit
**
** ��������:			��LED���е������ơ�
**                      
**					    
** �������:			uint8 val;
** ����ֵ:			void
**
** ʹ�ú����:		��غ궨��,�μ�LED.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:			����
** ��������:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ClrLedBit(uint8 val)
{
	#if LED_PORT_ALONE	
	#else
		ClrRegBit(LED_PORT, LED_BIT_BOTTOM + val);
	#endif
}
	



/*************************************************************************************************************************
** ��������:			ChangeLedBit
**
** ��������:			��LED���е������ơ�
**                      
**					    
** �������:			uint8 val;
** ����ֵ:				void
**
** ʹ�ú����:		��غ궨��,�μ�LED.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:				����
** ��������:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ChangeLedBit(uint8 val)
{
	#if LED_PORT_ALONE
	#else

		ChangeRegBit(LED_PORT, LED_BIT_BOTTOM + val);

	#endif
}
	


/*************************************************************************************************************************
** ��������:			SetLedGroup
**
** ��������:			���ĸ�������LED���г�����ơ�
**                      
**					    
** �������:			uint8 val;
** ����ֵ:				void
**
** ʹ�ú����:		��غ궨��,�μ�LED.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:				����
** ��������:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void SetLedGroup(uint8 val)
{
	#if LED_PORT_ALONE
	#else

		SetReg(LED_PORT, (val << LED_BIT_BOTTOM) & LED_MASK);
	
	#endif
	
}

/*************************************************************************************************************************
** ��������:			InitLed
**
** ��������:			��ʼ��4��������LED����LED�Ĳ�������Ӧ�÷���API����ɣ�
**					���ǿ��ǵ����������ڶ�ϵͳ���е��Եģ�����Ӧ�þ����ܽӽ��ײ㣬���Է�������������ӿڲ㡣
**                      				    
** �������:			void;
** ����ֵ:			void;
**
** ʹ�ú����:		��غ궨��,�μ�LED.h; 
** ʹ��ȫ�ֱ���:		None;
**
** ���ú���:			None 
**
** ������:			����
** ��������:			2009-03-18
**------------------------------------------------------------------------------------------------------------------------
** �޶���:              
** �޶�����:            
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitLed(void)
{
	#if LED_PORT_ALONE
	#else

		SetReg(led_control.state, 0);									// ��ʼ��״̬�Ĵ���
		
		SetRegBits(LED_DDR, LED_MASK);
		ClrRegBits(LED_PORT, LED_MASK);
		led_control.pSetLedGroup = SetLedGroup;
		led_control.pClrLedBit = ClrLedBit;
		led_control.pSetLedBit = SetLedBit;
		led_control.pChangeLedBit = ChangeLedBit;
		
		SetRegBit(led_control.state, INIT_COMPLETE);					// ��λ��ʼ����ɱ�־

	#endif
}
	


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/
LED_CONTROL_STRUCT led_control = { .pInit = InitLed };


/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/