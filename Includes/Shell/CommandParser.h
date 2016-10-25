/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			CommandParser.h
** 最后修订日期:		2009-03-19
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
                                                         结构体定义
*************************************************************************************************************************/
typedef uint8 (tCommandParse)(COMMUNICATION_PORT_STRUCT* p_communication_str);

typedef struct
{
	tCommandParse* pCommandParse;
}COMMAND_CONTROL_STRUCT;

typedef struct SSaynControl
{
	uint8 adc_single_sayn_time;				//单路AD异步查询
	uint8 adc_single_sayn_count;
	uint8 adc_single_sayn_functional;
	uint8 adc_single_sayn_method;
	uint8 adc_single_sayn_length;
	uint8 adc_single_sayn_parameter[2];

	uint8 adc_multi_sayn_time;				//多路AD异步查询
	uint8 adc_multi_sayn_count;
	uint8 adc_multi_sayn_functional;
	uint8 adc_multi_sayn_method;
	uint8 adc_multi_sayn_length;
	uint8 adc_multi_sayn_parameter[2];

	uint8 adc_all_sayn_time;				//所有AD异步查询
	uint8 adc_all_sayn_count;
	uint8 adc_all_sayn_functional;
	uint8 adc_all_sayn_method;
	uint8 adc_all_sayn_length;
	uint8 adc_all_sayn_parameter[2];

	uint8 io_sayn_time;						//IO异步查询
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
	uint8 servo_get_position_parameter[50];  	//存储舵机ID和时间

	uint8 servo_set_position_num;
	uint8 servo_set_position_functional;
	uint8 servo_set_position_method;
	uint8 servo_set_position_length;
	uint8 servo_set_position_parameter[151];	//存储舵机ID和位置

	uint8 servo_set_velocity_num;
	uint8 servo_set_velocity_functional;
	uint8 servo_set_velocity_method;
	uint8 servo_set_velocity_length;
	uint8 servo_set_velocity_parameter[151];	//存储舵机ID和位置

	uint8 servo_set_mode_num;
	uint8 servo_set_mode_functional;
	uint8 servo_set_mode_method;
	uint8 servo_set_mode_length;
	uint8 servo_set_mode_parameter[60]; 		//存储舵机ID和模式

	uint8 servo_set_torque_num;
	uint8 servo_set_torque_functional;
	uint8 servo_set_torque_method;
	uint8 servo_set_torque_length;
	uint8 servo_set_torque_parameter[60];		//存储舵机ID和模式

}SAYN_CONTROL_STRUCT;


/*************************************************************************************************************************
                                                       控制结构体声明
*************************************************************************************************************************/
COMMANDPARSER_EXT COMMAND_CONTROL_STRUCT command_control;
COMMANDPARSER_EXT SAYN_CONTROL_STRUCT sayn_communications_control;

/*************************************************************************************************************************
                                                         函数声明
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
**                                                      文件结束
*************************************************************************************************************************/
