/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			CommandParser.h
** ����޶�����:		2009-03-19
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
#ifndef COMMANDPARSER_H_
#define COMMANDPARSER_H_

#include "ConfigTypes.h"
#include "Shell/SystemUnit.h"
#include "Shell/SignalUnit.h"


#define		COMMANDPARSER_GLOBALS


#ifndef   COMMANDPARSER_GLOBALS
     #define COMMANDPARSER_EXT     
#else 
     #define COMMANDPARSER_EXT  extern
#endif 


/*************************************************************************************************************************
                                                         �ṹ�嶨��
*************************************************************************************************************************/
typedef uint8 (tCommandParse)(COMMUNICATION_PORT_STRUCT* p_communication_str);

typedef struct
{
	tCommandParse* pCommandParse;
}COMMAND_CONTROL_STRUCT;

typedef struct SSaynControl
{
	uint8 adc_single_sayn_time;				//��·AD�첽��ѯ
	uint8 adc_single_sayn_count;
	uint8 adc_single_sayn_functional;
	uint8 adc_single_sayn_method;
	uint8 adc_single_sayn_length;
	uint8 adc_single_sayn_parameter[2];

	uint8 adc_multi_sayn_time;				//��·AD�첽��ѯ
	uint8 adc_multi_sayn_count;
	uint8 adc_multi_sayn_functional;
	uint8 adc_multi_sayn_method;
	uint8 adc_multi_sayn_length;
	uint8 adc_multi_sayn_parameter[2];

	uint8 adc_all_sayn_time;				//����AD�첽��ѯ
	uint8 adc_all_sayn_count;
	uint8 adc_all_sayn_functional;
	uint8 adc_all_sayn_method;
	uint8 adc_all_sayn_length;
	uint8 adc_all_sayn_parameter[2];

	uint8 io_sayn_time;						//IO�첽��ѯ
	uint8 io_sayn_count;
	uint8 io_sayn_functional;
	uint8 io_sayn_method;
	uint8 io_sayn_length;
	uint8 io_sayn_parameter[2];

	uint8 servo_get_position_time;
	uint8 servo_get_position_count;
	uint8 servo_get_position_num;
	uint8 servo_get_position_functional;
	uint8 servo_get_position_method;
	uint8 servo_get_position_length;
	uint8 servo_get_position_parameter[50];  	//�洢���ID��ʱ��

	uint8 servo_set_position_num;
	uint8 servo_set_position_functional;
	uint8 servo_set_position_method;
	uint8 servo_set_position_length;
	uint8 servo_set_position_parameter[151];	//�洢���ID��λ��

	uint8 servo_set_velocity_num;
	uint8 servo_set_velocity_functional;
	uint8 servo_set_velocity_method;
	uint8 servo_set_velocity_length;
	uint8 servo_set_velocity_parameter[151];	//�洢���ID��λ��

	uint8 servo_set_mode_num;
	uint8 servo_set_mode_functional;
	uint8 servo_set_mode_method;
	uint8 servo_set_mode_length;
	uint8 servo_set_mode_parameter[60]; 		//�洢���ID��ģʽ

	uint8 servo_set_torque_num;
	uint8 servo_set_torque_functional;
	uint8 servo_set_torque_method;
	uint8 servo_set_torque_length;
	uint8 servo_set_torque_parameter[60];		//�洢���ID��ģʽ

}SAYN_CONTROL_STRUCT;


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/
COMMANDPARSER_EXT COMMAND_CONTROL_STRUCT command_control;
COMMANDPARSER_EXT SAYN_CONTROL_STRUCT sayn_communications_control;

/*************************************************************************************************************************
                                                         ��������
*************************************************************************************************************************/
extern uint8 GetAdcSamplingAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id);
extern uint8 GetAdcMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 adc_id);
extern uint8 GetAdcAllAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str);
extern uint8 GetIoInputAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str);

extern uint8 GetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter);
extern uint8 SetServoPositionMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter);
extern uint8 SetServoModeMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter);
extern uint8 SetServoTorqueMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter);
extern uint8 SetServoVelocityMultiAtTimes(COMMUNICATION_PORT_STRUCT* p_communication_str,uint8 * p_parameter);
#endif
/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/
