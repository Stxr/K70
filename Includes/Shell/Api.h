/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			Api.h
** ����޶�����:		2009-05-27
** ���汾:			1.0
** ����:				Fractal���Ӧ�ó���ӿڣ�������NorthStar����ͼ���뻷��������
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-05-27
** �汾:				1.0
** ����:				��Ҫʵ�ֶ�CommandActuator�еĹ��ܺ������б�׼����װ
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
#ifndef API_H_
#define API_H_

#include "ConfigTypes.h"

#include "Drivers/Adc.h"
#include "Drivers/Led.h"
#include "Drivers/Gpio.h"

#include "Shell/ServoUnit.h"
#include "Shell/ClockUnit.h"

#include "Apps/SystemInit.h"

//������С�з���ӵ�ͷ�ļ�
#include "Drivers/Uart.h"
//������С�з���ӵ�ͷ�ļ�
#define		API_GLOBALS

#ifndef   API_GLOBALS
     #define API_EXT     
#else 
     #define API_EXT  extern
#endif 


/*************************************************************************************************************************
                                                         �ṹ�嶨��
*************************************************************************************************************************/


/*************************************************************************************************************************
                                                       ���ƽṹ������
*************************************************************************************************************************/


/*************************************************************************************************************************
                                                         ��������
*************************************************************************************************************************/
API_EXT void MFInit(void);															// ϵͳ��ʼ��

//Delay
API_EXT void DelayMS(int32 inMS);												// ���뼶��ʱ

//Digi. IO
API_EXT void MFSetPortDirect(uint32 inData);									// ����IO����
API_EXT int32 MFGetDigiInput(int32 inID);										// ��ȡIO����
API_EXT void MFDigiOutput(int32 inID,int32 inVal);								// ����IO���

//AD
API_EXT int32 MFGetAD(int32 inID);												// ��ȡADֵ

//Servo
API_EXT void MFSetServoMode(int32 inID,int32 inMode);							// ���ģʽ
API_EXT void MFSetServoPos(int32 inID,int32 inPos,int32 inSpeed);				// ���λ������
API_EXT void MFSetServoRotaSpd(int32 inID,int32 inSpeed);						// ������ģʽ�ٶ�����
API_EXT int32 MFGetServoPos(int32 inID);										// ��ȡ���λ��
API_EXT void MFServoAction();													// �˶�ָ��ִ��
API_EXT void MFInitServoMapping(uint8*, uint8);
//������С�зд�ĺ���
API_EXT void MFInitUart(void);
API_EXT void Uart0SetDCB(uint8 mode, uint32 baud);
API_EXT void Uart0Send(uint8 val);
API_EXT void Uart1SetDCB(uint8 mode, uint32 baud);
API_EXT void Uart1Send(uint8 val);
#endif
/*************************************************************************************************************************
**                                                      �ļ�����
*************************************************************************************************************************/