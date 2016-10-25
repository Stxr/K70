/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			VirtualUart.h
** ����޶�����:		2009-08-20
** ���汾:			1.0
** ����:				����Uart�ı�׼��ɽӿ�(API)
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2009-08-20
** �汾:				1.0
** ����:				�����豸�ĳ�ʼ�������������������õ�
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
#ifndef VIRTUALUART_H_
#define VIRTUALUART_H_

#include "ConfigTypes.h"
#include "Lib/Queue.h"
#include "Drivers/Uart.h"
#include "Drivers/Spi.h"
#include "Drivers/Led.h"

#define VIRTUALUART_GLOBALS


#ifndef   VIRTUALUART_GLOBALS
     #define VIRTUALUART_EXT
#else 
     #define VIRTUALUART_EXT  extern
#endif


/*************************************************************************************************************************
 * 													Ӳ���궨��
*************************************************************************************************************************/


/*************************************************************************************************************************
 * 													�ṹ�嶨��
*************************************************************************************************************************/


/*************************************************************************************************************************
 * 													���ƽṹ������
*************************************************************************************************************************/
VIRTUALUART_EXT UART_CONTROL_STRUCT uart2_control;


#endif
/*************************************************************************************************************************
**													�ļ�����
*************************************************************************************************************************/